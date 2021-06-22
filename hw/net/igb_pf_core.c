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
#include "qemu/log.h"
#include "net/net.h"
#include "net/tap.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "sysemu/runstate.h"

#include "net_tx_pkt.h"
#include "net_rx_pkt.h"

#include "igb_pf_core.h"
#include "igb_pf_reg.h"
#include "igb_pf_interrupt.h"
#include "igb_pf_phy.h"
#include "igb_pf_txrx.h"
#include "igb_pf_reg_to_name.h"
#include "igb_mailbox.h"

#include "trace.h"


#pragma GCC diagnostic error "-Woverride-init"

static void
igbx_reset_mac_addr(NICState *nic, uint32_t *mac_regs, uint8_t *mac_addr)
{
    int i;

    mac_regs[RAL0] = 0;
    mac_regs[RAH0] = IGB_RAH_AV;
    for (i = 0; i < 4; i++) {
        mac_regs[RAL0] |= mac_addr[i] << (8 * i);
        mac_regs[RAH0] |= (i < 2) ? mac_addr[i + 4] << (8 * i) : 0;
    }

    qemu_format_nic_info_str(qemu_get_queue(nic), mac_addr);
    trace_igb_mac_indicate(MAC_ARG(mac_addr));
}



static void
igb_pf_set_ctrl(IgbPfCore *core, int index, uint32_t val)
{
    trace_igb_pf_core_ctrl_write(index, val);

    /* RST is self clearing */
    core->mac[CTRL] = val & ~IGB_CTRL_RST;

    trace_igb_link_set_params(
        bitfield_extract(val, IGB_CTRL_ASDE),
        bitfield_extract(val, IGB_CTRL_SPD_SEL),
        bitfield_extract(val, IGB_CTRL_FRCSPD),
        bitfield_extract(val, IGB_CTRL_FRCDPX),
        bitfield_extract(val, IGB_CTRL_RFCE),
        bitfield_extract(val, IGB_CTRL_TFCE));

    if (val & IGB_CTRL_RST) {
        trace_igb_pf_core_ctrl_sw_reset();
        igbx_reset_mac_addr(core->owner_nic, core->mac, core->permanent_mac);
    }

    /*
        if (val & IGB_CTRL_PHY_RST) {
            trace_igb_pf_core_ctrl_phy_reset();
            core->mac[STATUS] |= IGB_STATUS_PHYRA;
        }
    */
}



static inline uint16_t
igb_pf_get_reg_index_with_offset(hwaddr addr)
{
    uint16_t index = (addr & 0x1ffff) >> 2;
    //TODO: move this somewhere else

    typedef struct {
        uint16_t reg_index;
        uint16_t new_reg_index;
    } reg_map_t;
#define ENTRY( addr,  idx) {REG2IDX(addr), idx}
    static const reg_map_t reg_map[] = {
        ENTRY(0x00004, CTRL),
        //ENTRY(0x03810, TDH0),
        //ENTRY(0x03818, TDT0),
    };
#undef ENTRY
    bool loop = false;
    do {
        for (unsigned k = 0; k < ARRAY_SIZE(reg_map); ++k) {
            if (reg_map[k].reg_index == index) {
                index = reg_map[k].new_reg_index;
                loop = true;
            }
        }
    } while (loop);
    return index;
}

static void
igb_pf_set_ctrlext(IgbPfCore *core, int index, uint32_t val)
{
    /* Zero self-clearing bits */
    val &= ~(IGB_CTRL_EXT_ASDCHK | IGB_CTRL_EXT_EE_RST);
    core->mac[CTRL_EXT] = val;

    if (val & IGB_CTRL_EXT_PFRSTD) {
        trace_igb_pf_rstd(igb_get_vf_count(core));
        igb_pf_mailbox_reset_done(core);
    }

}



/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// READ Operations
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

static uint32_t
igb_pf_mac_readreg(IgbPfCore *core, int index)
{
    return core->mac[index];
}

static uint32_t
igb_pf_mac_readzero(IgbPfCore *core, int index)
{
    return 0;
}



static uint32_t
igb_pf_mac_status_read(IgbPfCore *core, int index)
{
    uint32_t res = core->mac[STATUS];
    //TODO: Implement  "Num VFs" and "IOV mode"
    return res;
}

#define igb_getreg(x)    [x] = igb_pf_mac_readreg
typedef uint32_t (*readops)(IgbPfCore *, int);
static const readops igb_macreg_readops[] = {


// General Register Descriptions
    igb_getreg(CTRL),
    [STATUS] = igb_pf_mac_status_read,
    igb_getreg(CTRL_EXT),
    igb_getreg(MDIC),
    igb_getreg(LEDCTL),
    igb_getreg(GPIE),
    igb_getreg(VET),

// Packet Buffers Control Register Descriptions
    igb_getreg(RXPBS),

// EEPROM/Flash Register Descriptions
    igb_getreg(EEC),
    igb_getreg(EERD),

// MNG-EEPROM CSR I/F
    igb_getreg(EEMNGCTL),

// Flow Control Register Descriptions
    igb_getreg(FCAL),
    igb_getreg(FCAH),
    igb_getreg(FCT),
    igb_getreg(FCTTV),
    igb_getreg(FCRTL),
    igb_getreg(FCRTH),

// PCIe Register Descriptions
    igb_getreg(GCR),

// Semaphore registers
    igb_getreg(SWSM),
    igb_getreg(FWSM),
    igb_getreg(SW_FW_SYNC),


// Interrupt Register Descriptions
    igb_getreg(EIAC),
    igb_getreg(EIAM),
    igb_getreg(EIMS),
    igb_getreg(EICS),
    igb_getreg(IAM),
    igb_getreg(IMC),
    igb_getreg(IMS),
    assign_8(IVAR, igb_pf_mac_readreg),
    igb_getreg(IVAR_MISC),
    assign_25(EITR, igb_pf_mac_icr_read),
    [ICR]     = igb_pf_mac_icr_read,
    [EICR]    = igb_pf_mac_eicr_read,


// MSI-X Table Register Descriptions

// Receive Register Descriptions
    igb_getreg(RCTL),
    igb_getreg(RXCSUM),
    igb_getreg(RFCTL),
    igb_getreg(MRQC),
    igb_getreg(RLPML),


    assign_16(RQDPC,   igb_pf_mac_readreg),
    assign_24(RAL, igb_pf_mac_readreg),
    assign_24(RAH, igb_pf_mac_readreg),

    assign_10(RSSRK, igb_pf_mac_readreg),
    assign_32(RETA,  igb_pf_mac_readreg),

    assign_16(RDBAL,  igb_pf_mac_readreg),
    assign_16(RDBAH,  igb_pf_mac_readreg),
    assign_16(RDLEN,  igb_pf_mac_readreg),
    assign_16(RDH,    igb_pf_mac_readreg),
    assign_16(RDT, 	  igb_pf_mac_readreg),
    assign_16(RXCTL,  igb_pf_mac_readreg),
    assign_16(RXDCTL, igb_pf_mac_readreg),
    assign_16(SRRCTL,  igb_pf_mac_readreg),


// Filtering Register Descriptions
    assign_8(ETQF, igb_pf_mac_readreg),
    assign_8(FTQF, igb_pf_mac_readreg),

// Transmit Register Descriptions
    igb_getreg(TCTL),
    assign_16(TDBAL,  igb_pf_mac_readreg),
    assign_16(TDBAH,  igb_pf_mac_readreg),
    assign_16(TDLEN,  igb_pf_mac_readreg),
    assign_16(TDH,    igb_pf_mac_readreg),
    assign_16(TDT, 	  igb_pf_mac_readreg),
    assign_16(TXCTL,  igb_pf_mac_readreg),
    assign_16(TXDCTL, igb_pf_mac_readreg),
    igb_getreg(DTXCTL),

// DCA Register Descriptions

// Virtualization Register Descriptions
    igb_getreg(DTXSWC),
    igb_getreg(VT_CTL),
    [VFLRE] = igb_pf_mac_readzero,
    assign_8(VMOLR, igb_pf_mac_readreg),
    assign_8(VMVIR, igb_pf_mac_readreg),
    assign_32(VLVF, igb_pf_mac_readreg),
    igb_getreg(RPLOLR),
    igb_getreg(MBVFICR),
    igb_getreg(MBVFIMR),
    igb_getreg(VFRE),
    igb_getreg(VFTE),
    igb_getreg(QDE),

    assign_8(PFMAILBOX, igb_pf_read_mailbox),
    [REG2IDX(IGB_VMBMEM(0)) ... REG2IDX(IGB_VMBMEM(IGB_MAX_VF - 1) + VMBMEM_WORDS_PER_PF - 1)] = igb_pf_read_vmbmem,

// Tx Bandwidth Allocation to VM Register Description

// Timer Register Descriptions

// Time Sync Register Descriptions
    igb_getreg(RXSTMPH),
    igb_getreg(RXSTMPL),
    igb_getreg(SYSTIMH),
    igb_getreg(SYSTIML),
    igb_getreg(TIMINCA),
    igb_getreg(TSYNCRXCTL),
    igb_getreg(TSYNCRXCFG),
    igb_getreg(TSYNCTXCTL),
    igb_getreg(TXSTMPH),
    igb_getreg(TXSTMPL),


// PCS Register Descriptions

// Statistics Register Descriptions
    [REG2IDX(IGB_CRCERRS) ... REG2IDX(IGB_SCVPC)] = igb_pf_mac_readzero,

// Wake Up Control Register Descriptions
    igb_getreg(WUC),
    igb_getreg(WUFC),

// Management Register Descriptions
    igb_getreg(MANC),

// MACSec Register Descriptions

// MACSec Tx Port Statistics

// MACSec Rx Port Statistic

// MACSec Rx SC Statistic Register Descriptions

// MACSec Rx SA Statistic Register Descriptions

// IPsec Registers Description

// Diagnostic Registers Description

// PCIe
    igb_getreg(IOVCTL),

};
enum { IGB_NREADOPS = ARRAY_SIZE(igb_macreg_readops) };

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// WRITE Operations
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
static void
igb_pf_mac_writereg(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[index] = val;
}

static void
igb_pf_mac_write1c(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[index] &= ~val;
}



static void
igb_pf_mac_writeignore(IgbPfCore *core, int index, uint32_t val)
{
}


static void
igb_pf_mac_setmacaddr(IgbPfCore *core, int index, uint32_t val)
{
    uint32_t macaddr[2];

    core->mac[index] = val;

    macaddr[0] = cpu_to_le32(core->mac[RAL0]);
    macaddr[1] = cpu_to_le32(core->mac[RAH0]);
    qemu_format_nic_info_str(qemu_get_queue(core->owner_nic),
                             (uint8_t *) macaddr);

    trace_igb_pf_mac_set_sw(MAC_ARG(macaddr));
}

static void
igb_pf_set_eecd(IgbPfCore *core, int index, uint32_t val)
{
    static const uint32_t ro_bits = IGB_EEC_PRES          |
                                    IGB_EEC_AUTO_RD       |
                                    IGB_EEC_SIZE_EX_MASK;

    core->mac[EEC] = (core->mac[EEC] & ro_bits) | (val & ~ro_bits);
}

static void
igb_pf_set_eerd(IgbPfCore *core, int index, uint32_t val)
{
    uint32_t addr = (val >> IGB_EERW_ADDR_SHIFT) & IGB_EERW_ADDR_MASK;
    uint32_t flags = 0;
    uint32_t data = 0;

    if ((addr < IGB_EEPROM_SIZE) && (val & IGB_EERW_START)) {
        data = core->eeprom[addr];
        flags = IGB_EERW_DONE;
    }

    core->mac[EERD] = flags                         |
                      (addr << IGB_EERW_ADDR_SHIFT) |
                      (data << IGB_EERW_DATA_SHIFT);
}



#define igb_putreg(x)    [x] = igb_pf_mac_writereg
typedef void (*writeops)(IgbPfCore *, int, uint32_t);
static const writeops igb_macreg_writeops[] = {


// General Register Descriptions
    [CTRL]     = igb_pf_set_ctrl,
    [CTRL_EXT] = igb_pf_set_ctrlext,
    [MDIC]     = igb_pf_set_mdic,
    igb_putreg(LEDCTL),
    igb_putreg(GPIE),
    igb_putreg(VET),

// Packet Buffers Control Register Descriptions
    igb_putreg(RXPBS),

// EEPROM/Flash Register Descriptions
    [EEC]      = igb_pf_set_eecd,
    [EERD]     = igb_pf_set_eerd,

// MNG-EEPROM CSR I/F
    igb_putreg(EEMNGCTL),

// Flow Control Register Descriptions
    igb_putreg(FCAL),
    igb_putreg(FCAH),
    igb_putreg(FCT),
    igb_putreg(FCTTV),
    igb_putreg(FCRTL),
    igb_putreg(FCRTH),

// PCIe Register Descriptions
    igb_putreg(GCR),

// Semaphore registers
    igb_putreg(SWSM),
    igb_putreg(FWSM),
    igb_putreg(SW_FW_SYNC),

// Interrupt Register Descriptions
    [EIAC]	   = igb_pf_set_eiac,
    [EIAM]	   = igb_pf_set_eiam,
    [EICR] 	   = igb_pf_set_eicr,
    [EICS] 	   = igb_pf_set_eics,
    [EIMC] 	   = igb_pf_set_eimc,
    [EIMS] 	   = igb_pf_set_eims,
    [IMC]      = igb_pf_set_imc,
    [IMS]      = igb_pf_set_ims,
    igb_putreg(IAM),
    assign_8(IVAR, igb_pf_mac_writereg),
    [IVAR_MISC] = igb_pf_mac_writereg,
    assign_25(EITR, igb_pf_mac_writereg),


// MSI-X Table Register Descriptions

// Receive Register Descriptions
    [RCTL]     = igb_pf_set_rctl,

    // The actual registers aren't contigious, so below is done in 3 parts
    [RAL0  ... RAH0 ]      = igb_pf_mac_setmacaddr,
    [RAL1  ... RAH15]      = igb_pf_mac_writereg,
    [RAL16 ... RAH23]      = igb_pf_mac_writereg,
    assign_10(RSSRK,  igb_pf_mac_writereg),
    assign_32(RETA,   igb_pf_mac_writereg),
    igb_putreg(RXCSUM),
    igb_putreg(RFCTL),
    igb_putreg(MRQC),
    igb_putreg(RLPML),

    assign_16(RQDPC,     	igb_pf_mac_writereg),

    assign_16(RDBAL,  igb_pf_mac_writereg),
    assign_16(RDBAH,  igb_pf_mac_writereg),
    assign_16(RDLEN,  igb_pf_mac_writereg),
    assign_16(RDH,    igb_pf_mac_writereg),
    assign_16(RDT, 	  igb_pf_set_rdt),
    assign_16(RXCTL,  igb_pf_mac_writereg),
    assign_16(RXDCTL, igb_pf_mac_writereg),
    assign_16(SRRCTL, igb_pf_set_srrctl),

    // Unicast/Multicast Table Arrays (we don't care about them)
    [REG2IDX(IGB_UTA)  ... REG2IDX(IGB_UTA  + 4 * 127)] = igb_pf_mac_writereg,
    [REG2IDX(IGB_MTA)  ... REG2IDX(IGB_MTA  + 4 * 127)] = igb_pf_mac_writereg,
    [REG2IDX(IGB_VFTA) ... REG2IDX(IGB_VFTA + 4 * 127)] = igb_pf_mac_writereg,

// Filtering Register Descriptions
    assign_8(ETQF, igb_pf_mac_writereg),
    assign_8(FTQF, igb_pf_mac_writereg),

// Transmit Register Descriptions
    [TCTL]     = igb_pf_set_tctl,

    assign_16(TDBAL,  igb_pf_mac_writereg),
    assign_16(TDBAH,  igb_pf_mac_writereg),
    assign_16(TDLEN,  igb_pf_mac_writereg),
    assign_16(TDH,    igb_pf_mac_writereg),
    assign_16(TDT, 	  igb_pf_set_tdt),
    assign_16(TXCTL,  igb_pf_mac_writereg),
    assign_16(TXDCTL, igb_pf_mac_writereg),

    igb_putreg(DTXCTL),


// DCA Register Descriptions

// Virtualization Register Descriptions
    igb_putreg(DTXSWC),
    igb_putreg(VT_CTL),
    [VFLRE] = igb_pf_mac_writeignore,
    assign_8(VMOLR, igb_pf_mac_writereg),
    assign_8(VMVIR, igb_pf_mac_writereg),
    assign_32(VLVF, igb_pf_mac_writereg),
    igb_putreg(RPLOLR),
    [MBVFICR] = igb_pf_mac_write1c,
    igb_putreg(MBVFIMR),
    igb_putreg(VFRE),
    igb_putreg(VFTE),
    igb_putreg(QDE),

    assign_8(PFMAILBOX, igb_pf_set_mailbox),
    [REG2IDX(IGB_VMBMEM(0)) ... REG2IDX(IGB_VMBMEM(IGB_MAX_VF - 1) + VMBMEM_WORDS_PER_PF - 1)] = igb_pf_set_vmbmem,

// Tx Bandwidth Allocation to VM Register Description

// Timer Register Descriptions

// Time Sync Register Descriptions
    igb_putreg(RXSTMPH),
    igb_putreg(RXSTMPL),
    igb_putreg(SYSTIMH),
    igb_putreg(SYSTIML),
    igb_putreg(TIMINCA),
    igb_putreg(TSYNCRXCTL),
    igb_putreg(TSYNCRXCFG),
    igb_putreg(TSYNCTXCTL),
    igb_putreg(TXSTMPH),
    igb_putreg(TXSTMPL),


// PCS Register Descriptions

// Statistics Register Descriptions

// Wake Up Control Register Descriptions
    igb_putreg(WUC),
    igb_putreg(WUFC),

// Management Register Descriptions
    igb_putreg(MANC),

// MACSec Register Descriptions

// MACSec Tx Port Statistics

// MACSec Rx Port Statistic

// MACSec Rx SC Statistic Register Descriptions

// MACSec Rx SA Statistic Register Descriptions

// IPsec Registers Description

// Diagnostic Registers Description

    // PCIe
    igb_putreg(IOVCTL),

};
enum { IGB_NWRITEOPS = ARRAY_SIZE(igb_macreg_writeops) };

void
igb_pf_core_write(IgbPfCore *core, hwaddr addr, uint64_t val, unsigned size)
{
    uint16_t index = igb_pf_get_reg_index_with_offset(addr);

    if (index < IGB_NWRITEOPS && igb_macreg_writeops[index]) {
        // normal case
        igb_macreg_writeops[index](core, index, val);
    } else if (index < IGB_NREADOPS && igb_macreg_readops[index]) {
        // read-only
        trace_igb_wrn_regs_write_ro(index << 2, igb_pf_reg2name(addr), val);
    } else {
        // unknown
        igb_pf_mac_writereg(core, index, val);
        trace_igb_wrn_regs_write_unknown(index << 2, igb_pf_reg2name(addr), val);
    }
}

uint64_t
igb_pf_core_read(IgbPfCore *core, hwaddr addr, unsigned size)
{
    uint64_t val;
    uint16_t index = igb_pf_get_reg_index_with_offset(addr);


    if (index < IGB_NREADOPS && igb_macreg_readops[index]) {
        //normal
        val = igb_macreg_readops[index](core, index);
        return val;
    } else {
        trace_igb_wrn_regs_read_unknown(index << 2, igb_pf_reg2name(addr));
        //igb_abort("unimplemented register read :  addr=%8lx\n", addr);
        return igb_pf_mac_readreg(core, index);
    }
    return 0;
}


static
void
igbx_core_prepare_eeprom(uint16_t       *eeprom,
                         const uint16_t *templ,
                         uint32_t        templ_size,
                         uint16_t        dev_id,
                         const uint8_t  *macaddr)
{
    uint16_t checksum = 0;
    int i;

    memmove(eeprom, templ, templ_size);

    for (i = 0; i < 3; i++) {
        eeprom[i] = (macaddr[2 * i + 1] << 8) | macaddr[2 * i];
    }

    eeprom[11] = eeprom[13] = dev_id;

    for (i = 0; i < EEPROM_CHECKSUM_REG; i++) {
        checksum += eeprom[i];
    }

    checksum = (uint16_t) EEPROM_SUM - checksum;

    eeprom[EEPROM_CHECKSUM_REG] = checksum;

}


void
igb_pf_core_pci_realize(IgbPfCore     *core,
                        const uint16_t *eeprom_templ,
                        uint32_t        eeprom_size,
                        const uint8_t  *macaddr)
{
    for (int i = 0; i < IGB_PF_NUM_QUEUES; i++) {
        net_tx_pkt_init(&core->tx[i].tx_pkt, core->owner,  IGB_MAX_TX_FRAGS,
                        false /*vnet*/);
    }


    igbx_core_prepare_eeprom(core->eeprom,
                             eeprom_templ,
                             eeprom_size,
                             PCI_DEVICE_GET_CLASS(core->owner)->device_id,
                             macaddr);

    // TX:
    igb_pf_core_prepare_tx(core);

    // RX:
    net_rx_pkt_init(&core->rx_pkt, false /*vnet*/);
    igb_pf_core_prepare_rx(core);
}

void
igb_pf_core_pci_uninit(IgbPfCore *core)
{
    for (int i = 0; i < IGB_PF_NUM_QUEUES; i++) {
        net_tx_pkt_reset(core->tx[i].tx_pkt);
        net_tx_pkt_uninit(core->tx[i].tx_pkt);
    }
    net_rx_pkt_uninit(core->rx_pkt);
}


static const uint32_t igb_pf_mac_reg_init[] = {

// General Register Descriptions
    [CTRL]          = IGB_CTRL_FD | IGB_CTRL_SPD_1000,
    [STATUS]        = IGB_STATUS_SPEED_1000 | IGB_STATUS_LU | IGB_STATUS_FD,
    [SWSM]          = 0,
    [VET]			= 0x81008100,


// Packet Buffers Control Register Descriptions
// EEPROM/Flash Register Descriptions
    [EEC]           = IGB_EEC_AUTO_RD | IGB_EEC_PRES,
    [EERD]          = IGB_EERW_DONE,

// MNG-EEPROM CSR I/F
    [EEMNGCTL]      = BIT(31),

// Flow Control Register Descriptions
// PCIe Register Descriptions
// Semaphore registers
// Interrupt Register Descriptions
// MSI-X Table Register Descriptions

// Receive Register Descriptions
    [RLPML]        = 0x2600,
    assign_16(RXDCTL, BIT(16)),
//    [RXCSUM]        = IGB_RXCSUM_IPOFLD | IGB_RXCSUM_TUOFLD,

// Filtering Register Descriptions
// Transmit Register Descriptions
// DCA Register Descriptions
// Virtualization Register Descriptions
    [MBVFIMR] = 0xFF,
// Tx Bandwidth Allocation to VM Register Description
// Timer Register Descriptions
// Time Sync Register Descriptions
// PCS Register Descriptions
// Statistics Register Descriptions
// Wake Up Control Register Descriptions
// Management Register Descriptions
// MACSec Register Descriptions
// MACSec Tx Port Statistics
// MACSec Rx Port Statistic
// MACSec Rx SC Statistic Register Descriptions
// MACSec Rx SA Statistic Register Descriptions
// IPsec Registers Description
// Diagnostic Registers Description
// Virtual function Register Descriptions

};

void
igb_pf_core_reset(IgbPfCore *core)
{
    igb_pf_core_phy_reset(core);
    memset(core->mac, 0, sizeof core->mac);

    if (sizeof(igb_pf_mac_reg_init) > sizeof(core->mac)) {
        igb_abort("sizeof igb_pf_mac_reg_init > mac ");
    }
    memmove(core->mac, igb_pf_mac_reg_init, sizeof igb_pf_mac_reg_init);

    igbx_reset_mac_addr(core->owner_nic, core->mac, core->permanent_mac);

    for (int i = 0; i < ARRAY_SIZE(core->tx); i++) {
        net_tx_pkt_reset(core->tx[i].tx_pkt);
        memset(&core->tx[i].props, 0, sizeof(core->tx[i].props));
        core->tx[i].skip_cp = false;
    }
}

int
igb_pf_core_post_load(IgbPfCore *core)
{
    NetClientState *nc = qemu_get_queue(core->owner_nic);

    /* nc.link_down can't be migrated, so infer link_down according
     * to link status bit in core.mac[STATUS].
     */
    nc->link_down = (core->mac[STATUS] & IGB_STATUS_LU) == 0;

    return 0;
}

