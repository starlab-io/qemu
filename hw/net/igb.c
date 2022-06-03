/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation
 *
 * Intel developer's manual for these devices:
 * http://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82576eg-gbe-datasheet.pdf
 *
 * Copied and edited from the E1000e QEMU emulation (e1000e.c) by Knut Omang.
 *
 * Implementation to make igb functional for interrupts,
 * TX/RX packet transmission, and VF implementation by Alex Olson.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/range.h"
#include "sysemu/sysemu.h"
#include "net/net.h"
#include "linux/virtio_net.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "igb.h"
#include "igb_pf_reg.h"
#include "igb_common.h"
#include "igb_pf_core.h"
#include "igb_vf_core.h"
#include "igb_pf_txrx.h"
#include "igb_vf_txrx.h"

#include "trace.h"
#include "qapi/error.h"

#define IGB_MMIO_BAR 0
#define IGB_MSIX_BAR 3

#define IGB_MSIX_VECTORS_PF 10
#define IGB_MSIX_VECTORS_VF 3

/* PCie Configuration Space Offsets */
#define IGB_AER_OFFSET       0x100
#define IGB_ARI_OFFSET       0x150
#define IGB_CAP_SRIOV_OFFSET 0x160

/* VF OFFSET/STRIDE are described in
 * 9.6.4.6 - SR-IOV VF RID Mapping Register FVO/VFS fields
 */
#define IGB_VF_OFFSET 0x80
#define IGB_VF_STRIDE 2

#define IGB_PF_MMIO_SIZE (128 * 1024)
#define IGB_FLASH_SIZE   (128 * 1024)
#define IGB_PF_MSIX_SIZE ( 16 * 1024)
#define IGB_IO_SIZE      (32)

#define IGB_VF_MMIO_SIZE ( 16 * 1024)
#define IGB_VF_MSIX_SIZE ( 16 * 1024)

#define IGB_TOTAL_VFS 8
#define TYPE_IGB_VF "igb_vf"

#define IGB_VF(obj) OBJECT_CHECK(IgbVfState, (obj), TYPE_IGB_VF)

struct IgbPfState {
    PCIDevice parent_obj;
    NICState *nic;
    NICConf conf;

    MemoryRegion mmio;
    MemoryRegion flash;
    MemoryRegion io;
    MemoryRegion msix;

    uint32_t ioaddr;

    IgbPfCore core;
};

typedef struct IgbVfState {
    PCIDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion msix;

    IgbVfCore core;
} IgbVfState;




/*
 * NOTE: this isn't really used much since can_receive always returns true
 */
void
igb_start_recv(IgbPfCore *core)
{
    for (int i = 0; i < core->owner_nic->conf->peers.queues; i++) {
        qemu_flush_queued_packets(qemu_get_subqueue(core->owner_nic, i));
    }
}

IgbPfCore *
igb_get_pf_core(IgbVfCore *core)
{
    IgbPfState *s = IGB_PF(core->owner->exp.sriov_vf.pf);
    return &s->core;
}

IgbVfCore *
igb_get_vf_core(IgbPfCore *core, unsigned vf)
{
    const int num_vf = core->owner->exp.sriov_pf.num_vfs;
    assert(vf < num_vf);
    return &IGB_VF(core->owner->exp.sriov_pf.vf[vf])->core;
}

int
igb_get_vf_num(IgbVfCore *core)
{
    return core->owner->exp.sriov_vf.vf_number;
}

unsigned
igb_get_vf_count(IgbPfCore *core)
{
    return core->owner->exp.sriov_pf.num_vfs;
}


static void igb_pf_write_config(PCIDevice *d, uint32_t address,
                                uint32_t val, int len)
{
    trace_igb_write_config(address, val, len);
    pci_default_write_config(d, address, val, len);

    IgbPfState *s = IGB_PF(d);
    if (range_covers_byte(address, len, PCI_COMMAND) &&
            (d->config[PCI_COMMAND] & PCI_COMMAND_MASTER)) {
        igb_start_recv(&s->core);
    }
}


static uint64_t igb_pf_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbPfState *s = opaque;
    return igb_pf_core_read(&s->core, addr, size);
}

static void igb_pf_mmio_write(void *opaque, hwaddr addr,
                              uint64_t val, unsigned size)
{
    IgbPfState *s = opaque;
    igb_pf_core_write(&s->core, addr, val, size);
}

static uint64_t igb_vf_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbVfState *s = opaque;
    return igb_vf_core_read(&s->core, addr, size);
}

static void igb_vf_mmio_write(void *opaque, hwaddr addr,
                              uint64_t val, unsigned size)
{
    IgbVfState *s = opaque;
    igb_vf_core_write(&s->core, addr, val, size);
}

/* This code is not yet verified since Linux doesn't use it, but looks reasonable */
static bool igb_io_get_reg_index(IgbPfState *s, uint32_t *idx)
{
    if (s->ioaddr < 0x1FFFF) {
        *idx = s->ioaddr;
        return true;
    }

    if (s->ioaddr < 0x7FFFF) {
        trace_igb_wrn_io_addr_undefined(s->ioaddr);
        return false;
    }

    if (s->ioaddr < 0xFFFFF) {
        trace_igb_wrn_io_addr_flash(s->ioaddr);
        return false;
    }

    trace_igb_wrn_io_addr_unknown(s->ioaddr);
    return false;
}

static uint64_t igb_pf_io_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbPfState *s = opaque;
    uint32_t idx = 0;
    uint64_t val;

    switch (addr) {
    case IGB_IOADDR:
        trace_igb_io_read_addr(s->ioaddr);
        return s->ioaddr;
    case IGB_IODATA:
        if (igb_io_get_reg_index(s, &idx)) {
            val = igb_pf_core_read(&s->core, idx, sizeof(val));
            trace_igb_io_read_data(idx, val);
            return val;
        }
        return 0;
    default:
        trace_igb_wrn_io_read_unknown(addr);
        return 0;
    }
}

static void igb_pf_io_write(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    IgbPfState *s = opaque;
    uint32_t idx = 0;

    switch (addr) {
    case IGB_IOADDR:
        trace_igb_io_write_addr(val);
        s->ioaddr = (uint32_t) val;
        return;
    case IGB_IODATA:
        if (igb_io_get_reg_index(s, &idx)) {
            trace_igb_io_write_data(idx, val);
            igb_pf_core_write(&s->core, idx, val, sizeof(val));
        }
        return;
    default:
        trace_igb_wrn_io_write_unknown(addr);
        return;
    }
}

static const MemoryRegionOps pf_mmio_ops = {
    .read = igb_pf_mmio_read,
    .write = igb_pf_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};


static const MemoryRegionOps vf_mmio_ops = {
    .read = igb_vf_mmio_read,
    .write = igb_vf_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};



static const MemoryRegionOps pf_io_ops = {
    .read = igb_pf_io_read,
    .write = igb_pf_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static bool igb_nc_can_receive(NetClientState *nc)
{
    /*
     *  With PF and (possibly) multiple VF,
     *  it would be a bit tricky to implement this properly.
     *  Instead, we always return true
     */
    return true;
}


void igb_broadcast_pkt(igb_send_context_t *context, const struct iovec *iov,
                       int iovcnt)
{
    IgbPfCore *pf_core = context->core;
    const int source   = context->source;
    const bool has_vnet = (source == IGB_SOURCE_QEMU) ? pf_core->has_vnet : false;
    uint8_t mac_dst[ETH_ALEN]= {};
    iov_to_buf(iov, iovcnt, 0, mac_dst, sizeof(mac_dst));
    const bool bcast = !!(mac_dst[0] & 1);
    uint32_t bitmap=0;
    igb_pf_mac_check(pf_core, mac_dst, &bitmap);

    /* Send to PF: */
    if (source != IGB_SOURCE_PF) {
        const uint32_t pf_bit = BIT(igb_get_vf_count(pf_core));
        if (igb_pf_promisc(pf_core) || bcast || (pf_bit & bitmap)) {
            igb_pf_receive_iov(pf_core, iov, iovcnt, has_vnet);
        }
    }

    /* Send to VF: */
    for (unsigned vf = 0; vf < igb_get_vf_count(pf_core); ++vf) {
        if (pf_core->mac[VFRE] & BIT(vf) && source != vf) {

            const uint32_t vf_bit = BIT(vf);
            if (bcast || (vf_bit & bitmap)) {
                IgbVfCore *vf_core = igb_get_vf_core(pf_core, vf);
                igb_vf_receive_iov(vf_core, iov, iovcnt, has_vnet);
            }
        }
    }

    /* Send to outside world: */
    if (source != IGB_SOURCE_QEMU) {
        NetClientState *nc = qemu_get_queue(pf_core->owner_nic);

        struct iovec iov2[iovcnt + 1];
        const int iov2cnt = iovcnt + 1;
        struct virtio_net_hdr hdr = {};
        for (int k = 0; k < iovcnt; ++k) {
            iov2[k + 1] = iov[k];
        }
        iov2[0].iov_base = &hdr;
        iov2[0].iov_len  = pf_core->has_vnet ? sizeof(hdr) : 0;

        if (bcast || !bitmap) {
            qemu_sendv_packet(nc, iov2, iov2cnt);
        }
    }
}

static ssize_t igb_nc_receive_iov(NetClientState *nc, const struct iovec *iov,
                                  int iovcnt)
{
    IgbPfState *s = qemu_get_nic_opaque(nc);
    igb_send_context_t context = {
        .core = &s->core,
        .source = IGB_SOURCE_QEMU,
    };

    igb_broadcast_pkt(&context, iov, iovcnt);
    return iov_size(iov, iovcnt);
}

static ssize_t
igb_nc_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    const struct iovec iov = {
        .iov_base = (uint8_t *)buf,
        .iov_len = size
    };

    return igb_nc_receive_iov(nc, &iov, 1);
}

static void
igb_pf_set_link_status(NetClientState *nc)
{
    IgbPfState *s = qemu_get_nic_opaque(nc);
    trace_igb_link_status_changed(nc->link_down);
    igb_pf_core_set_link_status(&s->core, nc->link_down);
}


static NetClientInfo net_igb_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = igb_nc_can_receive,
    .receive = igb_nc_receive,
    .receive_iov = igb_nc_receive_iov,
    .link_status_changed = igb_pf_set_link_status,
};


static int igb_add_pm_capability(PCIDevice *pdev, uint8_t offset, uint16_t pmc)
{
    Error *local_err = NULL;
    int ret = pci_add_capability(pdev, PCI_CAP_ID_PM, offset,
                                 PCI_PM_SIZEOF, &local_err);

    if (local_err) {
        error_report_err(local_err);
        return ret;
    }

    pci_set_word(pdev->config + offset + PCI_PM_PMC,
                 PCI_PM_CAP_VER_1_1 |
                 pmc);

    pci_set_word(pdev->wmask + offset + PCI_PM_CTRL,
                 PCI_PM_CTRL_STATE_MASK |
                 PCI_PM_CTRL_PME_ENABLE |
                 PCI_PM_CTRL_DATA_SEL_MASK);

    pci_set_word(pdev->w1cmask + offset + PCI_PM_CTRL,
                 PCI_PM_CTRL_PME_STATUS);

    return ret;
}

static void igb_init_net_peer(IgbPfState *s, PCIDevice *pci_dev,
                              uint8_t *macaddr)
{
    DeviceState *dev = DEVICE(pci_dev);

    s->nic = qemu_new_nic(&net_igb_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id, s);

    trace_igb_mac_set_permanent(MAC_ARG(macaddr));
    memcpy(s->core.permanent_mac, macaddr, sizeof(s->core.permanent_mac));


    /* Network setup */
    qemu_format_nic_info_str(qemu_get_queue(s->nic), macaddr);
    s->core.has_vnet = true;
    for (int i = 0; i < s->conf.peers.queues; i++) {
        NetClientState *nc;
        nc = qemu_get_subqueue(s->nic, i);
        if (!nc->peer || !qemu_has_vnet_hdr(nc->peer)) {
            s->core.has_vnet = false;
            trace_igb_cfg_support_virtio(false);
            goto after_vnet_handling;
        }
    }

    for (int i = 0; i < s->conf.peers.queues; i++) {
        NetClientState *nc;
        nc = qemu_get_subqueue(s->nic, i);
        qemu_set_vnet_hdr_len(nc->peer, sizeof(struct virtio_net_hdr));
        qemu_using_vnet_hdr(nc->peer, true);
    }
after_vnet_handling:
    return;

}

/* EEPROM (NVM) contents documented in section 6.1, table 6-1:
 * and in 6.10 Software accessed words.
 *
 * TODO: Need to walk through this, names in comments are ok up to 0x4F
 */
static const uint16_t igb_eeprom_template[IGB_EEPROM_SIZE] = {
    /*        Address        |    Compat.    | ImRev |Compat.|OEM sp.*/
    0x0000, 0x0000, 0x0000, 0x0d14, 0xffff, 0x2010, 0xffff, 0xffff,
    /*      PBA      |ICtrl1 | SSID  | SVID  | DevID |-------|ICtrl2 */
    0x1040, 0xffff, 0x046b, 0x484c, 0x108e, 0x10c9, 0x0000, 0xf14b,
    /* SwPin0| DevID | EESZ  |-------|ICtrl3 |PCI-tc | MSIX  | APtr  */
    0xe30c, 0x10c9, 0x6000, 0x0000, 0x8c01, 0x0014, 0x4a40, 0x0060,
    /* PCIe Init. Conf 1,2,3 |PCICtrl| LD1,3 |DDevID |DevRev | LD0,2 */
    0x6cf6, 0xd7b0, 0x0a7e, 0x8403, 0x4784, 0x10a6, 0x0001, 0x4602,
    /* SwPin1| FunC  |LAN-PWR|ManHwC |ICtrl3 | IOVct |VDevID |-------*/
    0xe30c, 0x2020, 0x1ae5, 0x004a, 0x8401, 0x00f7, 0x10ca, 0x0000,
    /*---------------| LD1,3 | LD0,2 | ROEnd | ROSta | Wdog  | VPD   */
    0x0000, 0x0000, 0x4784, 0x4602, 0x0000, 0x0000, 0x0000, 0xffff,
    /* PCSet0| Ccfg0 |PXEver |IBAcap |PCSet1 | Ccfg1 |iSCVer | ??    */
    0x0100, 0x4000, 0x131f, 0x4013, 0x0100, 0x4000, 0xffff, 0xffff,
    /* PCSet2| Ccfg2 |PCSet3 | Ccfg3 | ??    |AltMacP| ??    |CHKSUM */
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x00e0, 0xffff, 0xb73b,
    /* ArbEn |-------| ImuID | ImuID |-------------------------------*/
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /*----------------------- Reserved ------------------------------*/
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    /* Word 0x50 - 0x5XX (sec.6.5) */
};


static void pci_igb_pf_realize(PCIDevice *d, Error **errp)
{
    int v;
    int ret;
    IgbPfState *igb = IGB_PF(d);
    uint8_t *macaddr;

    trace_igb_cb_pci_realize();

    d->config_write = igb_pf_write_config;

    d->config[PCI_CACHE_LINE_SIZE] = 0x10;
    d->config[PCI_INTERRUPT_PIN] = 1;

    /* BAR0: MMIO */
    memory_region_init_io(&igb->mmio, OBJECT(d), &pf_mmio_ops, igb,
                          "igb-pf-mmio", IGB_PF_MMIO_SIZE);
    pci_register_bar(d, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &igb->mmio);

    /* BAR1: flash memory (dummy) */
    memory_region_init(&igb->flash, OBJECT(d),
                       "igb-pf-flash", IGB_FLASH_SIZE);
    pci_register_bar(d, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &igb->flash);

    /* BAR2: I/O ports */
    memory_region_init_io(&igb->io, OBJECT(d), &pf_io_ops, igb,
                          "igb-pf-io", IGB_IO_SIZE);
    pci_register_bar(d, 2, PCI_BASE_ADDRESS_SPACE_IO, &igb->io);

    /* BAR3: MSIX table (PF - 16KB) */
    memory_region_init(&igb->msix, OBJECT(d), "igb-pf-msix", IGB_PF_MSIX_SIZE);
    pci_register_bar(d, IGB_MSIX_BAR, PCI_BASE_ADDRESS_MEM_TYPE_64, &igb->msix);

    /* Add PCI capabilities in reverse order: */
    ret = pcie_endpoint_cap_init(d, 0xa0);
    if (ret < 0) {
        goto err_pcie_cap;
    }

    ret = msix_init(d, IGB_MSIX_VECTORS_PF, &igb->msix, IGB_MSIX_BAR, 0, &igb->msix,
                    IGB_MSIX_BAR, 0x2000, 0x70, errp);
    if (ret) {
        goto err_msix;
    }

    ret = msi_init(d, 0x50, 1, true, true, errp);
    if (ret < 0) {
        goto err_msi;
    }

    /* TODO: Only initialize the vectors used */
    for (v = 0; v < IGB_MSIX_VECTORS_PF; v++) {
        ret = msix_vector_use(d, v);
        if (ret) {
            goto err_pcie_cap;
        }
    }

    if (igb_add_pm_capability(d, 0x40, PCI_PM_CAP_DSI) < 0) {
        hw_error("Failed to initialize PM capability");
    }

    /* PCIe extended capabilities (in order) */
    ret = pcie_aer_init(d, 1, IGB_AER_OFFSET, 0x40, errp);
    if (ret < 0) {
        goto err_aer;
    }

    pcie_ari_init(d, IGB_ARI_OFFSET, 1);

    pcie_sriov_pf_init(d,                     /* dev 	    */
                       IGB_CAP_SRIOV_OFFSET,  /* IOV support offset */
                       "igb_vf",              /* vfname 	*/
                       IGB_82576_VF_DEV_ID,   /* devid 	    */
                       IGB_TOTAL_VFS,         /* init_vfs 	*/
                       IGB_TOTAL_VFS,         /* total_vfs 	*/
                       IGB_VF_OFFSET,         /* vf_offset 	*/
                       IGB_VF_STRIDE);        /* vf_stride 	*/

    pcie_sriov_pf_init_vf_bar(d, IGB_MMIO_BAR,
                              PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
                              IGB_VF_MMIO_SIZE);

    pcie_sriov_pf_init_vf_bar(d, IGB_MSIX_BAR,
                              PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
                              IGB_VF_MSIX_SIZE);

    /* Create networking backend */
    qemu_macaddr_default_if_unset(&igb->conf.macaddr);
    macaddr = igb->conf.macaddr.a;

    igb_init_net_peer(igb, d, macaddr);

    /* Initialize core */
    igb->core.owner = &igb->parent_obj;
    igb->core.owner_nic = igb->nic;

    igb_pf_core_pci_realize(&igb->core,
                            igb_eeprom_template,
                            sizeof(igb_eeprom_template),
                            macaddr);
    return;
err_aer:
    msi_uninit(d);
err_msi:
    msix_unuse_all_vectors(d);
    msix_uninit(d, &igb->msix, &igb->msix);
err_msix:
    pcie_cap_exit(d);
err_pcie_cap:
    return;
    /* TBD: pci_e1000_uninit(d); */
}

static void pci_igb_pf_uninit(PCIDevice *d)
{
    IgbPfState *igb = IGB_PF(d);
    MemoryRegion *mr = &igb->msix;

    trace_igb_cb_pci_uninit();

    pcie_sriov_pf_exit(d);
    pcie_cap_exit(d);
    qemu_del_nic(igb->nic);
    msix_unuse_all_vectors(d);
    msix_uninit(d, mr, mr);
    msi_uninit(d);
}

static void igb_pf_reset(DeviceState *dev)
{
    PCIDevice *d = PCI_DEVICE(dev);
    IgbPfState *s = IGB_PF(dev);

    trace_igb_cb_qdev_reset();
    pcie_sriov_pf_disable_vfs(d);
    igb_pf_core_reset(&s->core);

    /* TODO: deal with registers that survive reset:
     *
     * RXPBS, TXPBS, SWPBS
     */
}

static int igb_pf_post_load(void *opaque, int version_id)
{
    IgbPfState *s = opaque;
    trace_igb_cb_post_load();
    return igb_pf_core_post_load(&s->core);
}

static void pci_igb_vf_realize(PCIDevice *d, Error **errp)
{
    int v;
    int ret;
    IgbVfState *igb = IGB_VF(d);

    if (*errp) {
        return;
    }

    memory_region_init_io(&igb->mmio, OBJECT(d), &vf_mmio_ops, igb,  "igb-vf-mmio",
                          IGB_VF_MMIO_SIZE);
    pcie_sriov_vf_register_bar(d, IGB_MMIO_BAR, &igb->mmio);

    memory_region_init(&igb->msix, OBJECT(d), "igb_vf-msix", IGB_VF_MSIX_SIZE);
    pcie_sriov_vf_register_bar(d, IGB_MSIX_BAR, &igb->msix);

    ret = msix_init(d, 						/* PCI device */
                    IGB_MSIX_VECTORS_VF, 	/* nentries */

                    &igb->msix, 			/* table_bar */
                    IGB_MSIX_BAR, 			/* table_bar_nr */
                    0, 						/* table_offset */

                    &igb->msix,				/* pba_bar */
                    IGB_MSIX_BAR, 			/* pba_bar_nr */
                    0x400 << 3, 			/* pba_bar offset */

                    0x70, 					/* cap_pos (VF PCIe Configuration Space - MSI-X Capability) */
                    errp);
    if (ret) {
        goto err_msix;
    }

    for (v = 0; v < IGB_MSIX_VECTORS_VF; v++) {
        ret = msix_vector_use(d, v);
        if (ret) {
            goto err_pcie_cap;
        }
    }

    ret = pcie_endpoint_cap_init(d, 0xa0);
    if (ret < 0) {
        goto err_pcie_cap;
    }

    ret = pcie_aer_init(d, 1, IGB_AER_OFFSET, 0x40, errp);
    if (ret < 0) {
        goto err_aer;
    }

    pcie_ari_init(d, IGB_ARI_OFFSET, 1);

    igb->core.owner = &igb->parent_obj;

    igb_vf_core_pci_realize(&igb->core);
    return;

err_aer:
    pcie_cap_exit(d);
err_pcie_cap:
    msix_unuse_all_vectors(d);
    msix_uninit(d, &igb->msix, &igb->msix);
err_msix:
    return;
    /* TBD: pci_e1000_uninit(d); */
}

static void igb_vf_reset(DeviceState *dev)
{
    IgbVfState *s = IGB_VF(dev);

    trace_igb_cb_qdev_reset();
    igb_vf_core_reset(&s->core);
}


static void pci_igb_vf_uninit(PCIDevice *d)
{
    IgbVfState *igb = IGB_VF(d);
    MemoryRegion *mr = &igb->msix;

    pcie_cap_exit(d);
    msix_uninit(d, mr, mr);
}

#define IGB_VMSTATE_VER 2

static const VMStateDescription igb_vmstate_tx_props = {
    .name = "igb-tx-props",
    .version_id = IGB_VMSTATE_VER,
    .minimum_version_id = IGB_VMSTATE_VER,
    .fields = (VMStateField[])
    {
        VMSTATE_UINT16(mss,  igbx_txd_props),
        VMSTATE_UINT16(vlan, igbx_txd_props),
        VMSTATE_END_OF_LIST()
    }
};


static const VMStateDescription igb_vmstate_tx = {
    .name = "igb-tx",
    .version_id = IGB_VMSTATE_VER,
    .minimum_version_id = IGB_VMSTATE_VER,
    .fields = (VMStateField[])
    {
        VMSTATE_STRUCT(props, IgbTx, IGB_VMSTATE_VER, igb_vmstate_tx_props, igbx_txd_props),
        VMSTATE_BOOL(skip_cp,           IgbTx),
        VMSTATE_BOOL(ip_sum_needed,     IgbTx),
        VMSTATE_BOOL(tcpudp_sum_needed, IgbTx),
        VMSTATE_BOOL(cptse,             IgbTx),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription igb_vmstate_rx = {
    .name = "igb-rx",
    .version_id = IGB_VMSTATE_VER,
    .minimum_version_id = IGB_VMSTATE_VER,
    .fields = (VMStateField[])
    {
        VMSTATE_UINT32(descriptor_type, IgbRx),
        VMSTATE_UINT32(packet_size,     IgbRx),
        VMSTATE_UINT32(header_size,     IgbRx),
        VMSTATE_END_OF_LIST()
    }
};


static const VMStateDescription igb_pf_vmstate_core = {
    .name = "igb-pf-core",
    .version_id = IGB_VMSTATE_VER,
    .minimum_version_id = IGB_VMSTATE_VER,
    .fields = (VMStateField[])
    {
        VMSTATE_UINT32_ARRAY(mac, IgbPfCore, IGB_PF_MAC_SIZE),
        VMSTATE_UINT16_2DARRAY(phy, IgbPfCore, IGB_PHY_PAGES, IGB_PHY_PAGE_SIZE),
        VMSTATE_UINT16_ARRAY(eeprom, IgbPfCore, IGB_EEPROM_SIZE),

        VMSTATE_STRUCT_ARRAY(tx, IgbPfCore, IGB_PF_NUM_QUEUES, IGB_VMSTATE_VER, igb_vmstate_tx,
                             IgbTx),
        VMSTATE_STRUCT_ARRAY(rx, IgbPfCore, IGB_PF_NUM_QUEUES, IGB_VMSTATE_VER, igb_vmstate_rx,
                             IgbRx),

        VMSTATE_BOOL(has_vnet, IgbPfCore),
        VMSTATE_UINT8_ARRAY(permanent_mac, IgbPfCore, ETH_ALEN),
		VMSTATE_TIMER_PTR(interrupt_timer, IgbPfCore),
		VMSTATE_UINT32(pending_interrupts, IgbPfCore),

        VMSTATE_END_OF_LIST()
    }
};



static const VMStateDescription igb_pf_vmstate = {
    .name = "igb_pf",
    .version_id = IGB_VMSTATE_VER,
    .minimum_version_id = IGB_VMSTATE_VER,
    .post_load = igb_pf_post_load,
    .fields = (VMStateField[])
    {
        VMSTATE_PCI_DEVICE(parent_obj, IgbPfState),
        VMSTATE_MSIX(parent_obj, IgbPfState),

        VMSTATE_UINT32(ioaddr, IgbPfState),
        VMSTATE_STRUCT(core, IgbPfState, IGB_VMSTATE_VER, igb_pf_vmstate_core, IgbPfCore),
        VMSTATE_END_OF_LIST()
    }
};


static const VMStateDescription igb_vf_vmstate_core = {
    .name = "igb-vf-core",
    .version_id = IGB_VMSTATE_VER,
    .minimum_version_id = IGB_VMSTATE_VER,
    .fields = (VMStateField[])
    {
        VMSTATE_UINT32_ARRAY(mac, IgbVfCore, IGB_VF_MAC_SIZE),
        VMSTATE_STRUCT_ARRAY(tx, IgbVfCore, IGB_VF_NUM_QUEUES, IGB_VMSTATE_VER, igb_vmstate_tx, IgbTx),
        VMSTATE_STRUCT_ARRAY(rx, IgbVfCore, IGB_VF_NUM_QUEUES, IGB_VMSTATE_VER, igb_vmstate_rx, IgbRx),
		VMSTATE_TIMER_PTR(interrupt_timer, IgbVfCore),
		VMSTATE_UINT32(pending_interrupts, IgbVfCore),
        VMSTATE_END_OF_LIST()
    }
};


static const VMStateDescription igb_vf_vmstate = {
    .name = "igb_vf",
    .version_id = IGB_VMSTATE_VER,
    .minimum_version_id = IGB_VMSTATE_VER,
    .fields = (VMStateField[])
    {
        VMSTATE_PCI_DEVICE(parent_obj, IgbVfState),
        VMSTATE_MSIX(parent_obj, IgbVfState),

        VMSTATE_STRUCT(core, IgbVfState, IGB_VMSTATE_VER, igb_vf_vmstate_core, IgbVfCore),
        VMSTATE_END_OF_LIST()
    }
};


static Property igb_pf_properties[] = {
    DEFINE_NIC_PROPERTIES(IgbPfState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static Property igb_vf_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void igb_pf_classinit(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);

    c->realize = pci_igb_pf_realize;
    c->exit = pci_igb_pf_uninit;
    c->vendor_id = PCI_VENDOR_ID_INTEL;
    c->device_id = IGB_DEV_ID_82576;
    c->revision = 1;
    c->romfile = NULL;
    c->class_id = PCI_CLASS_NETWORK_ETHERNET;

    dc->desc = "Intel 82576 GbE Controller";
    dc->reset = igb_pf_reset;
    dc->vmsd = &igb_pf_vmstate;

    device_class_set_props(dc, igb_pf_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}


static void igb_vf_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);

    c->realize = pci_igb_vf_realize;
    c->exit = pci_igb_vf_uninit;
    c->vendor_id = PCI_VENDOR_ID_INTEL;
    c->device_id = IGB_82576_VF_DEV_ID;
    c->revision = 1;
    c->romfile = NULL;
    c->class_id = PCI_CLASS_NETWORK_ETHERNET;

    dc->desc = "Intel 82576 GbE Controller Virtual Function";
    dc->reset = igb_vf_reset;
    dc->vmsd = &igb_vf_vmstate;
    device_class_set_props(dc, igb_vf_properties);
}


static const TypeInfo igb_pf_info = {
    .name = TYPE_IGB_PF,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IgbPfState),
    .class_init = igb_pf_classinit,
    .interfaces = (InterfaceInfo[])
    {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static const TypeInfo igb_vf_info = {
    .name = TYPE_IGB_VF,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IgbVfState),
    .class_init = igb_vf_class_init,
    .interfaces = (InterfaceInfo[])
    {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};


static void igb_register_types(void)
{
    type_register_static(&igb_pf_info);
    type_register_static(&igb_vf_info);
}

type_init(igb_register_types)
