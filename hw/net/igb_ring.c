/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation
 *
 * Copyright (c) 2021
 *
 * Author:
 *   Alex Olson, Star Lab
 * (parts derived/adopted from e1000e_core.c)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "igb_ring.h"
#include "igb_common.h"
#include "igb_shared_reg_constants.h"
#include "net_tx_pkt.h"
#include "net_rx_pkt.h"
#include "net/tap.h"


static bool
igb_ring_empty(const Igb_RingInfo *r)
{
    return *r->dh == *r->dt ||
           *r->dt >= *r->dlen / IGB_RING_DESC_LEN;
}

static uint64_t
igb_ring_base(const Igb_RingInfo *r)
{
    uint64_t bah = *r->dbah;
    uint64_t bal = *r->dbal;

    return (bah << 32) + bal;
}

static uint64_t
igb_ring_head_descr(const Igb_RingInfo *r)
{
    return igb_ring_base(r) + IGB_RING_DESC_LEN * *r->dh;
}

static void
igb_ring_advance(const Igb_RingInfo *r, uint32_t count)
{
    *r->dh += count;

    if (*r->dh * IGB_RING_DESC_LEN >= *r->dlen) {
        *r->dh = 0;
    }
}

/* ####### ######     #    #     #  #####  #     #   ###   ####### */
/*    #    #     #   # #   ##    # #     # ##   ##    #       #   */
/*    #    #     #  #   #  # #   # #       # # # #    #       #   */
/*    #    ######  #     # #  #  #  #####  #  #  #    #       #   */
/*    #    #   #   ####### #   # #       # #     #    #       #   */
/*    #    #    #  #     # #    ## #     # #     #    #       #   */
/*    #    #     # #     # #     #  #####  #     #   ###      #   */


static void
igb_setup_tx_offloads(IgbTx *tx)
{
#if 0
    fprintf(stderr, "%s: cptse=%d ip=%d tcp/udp=%d mss=%u\n", __FUNCTION__,
            tx->cptse,
            tx->ip_sum_needed,
            tx->tcpudp_sum_needed,
            tx->props.mss);
#endif

    if (tx->cptse) {
        net_tx_pkt_build_vheader(tx->tx_pkt, true, true, tx->props.mss);
        net_tx_pkt_update_ip_checksums(tx->tx_pkt);
        // stats TSCTC not handled here
        return;
    }

    if (tx->tcpudp_sum_needed) {
        net_tx_pkt_build_vheader(tx->tx_pkt, false, true, 0);
    }

    if (tx->ip_sum_needed) {
        net_tx_pkt_update_ip_hdr_checksum(tx->tx_pkt);
    }
}


static void
igb_parse_tx_ctx_descr(struct igb_adv_tx_context_desc *d, igbx_txd_props *props)
{
#if 0
    fprintf(stderr,
            "-----------------------------------------------------------------\n");
    fprintf(stderr,
            "TX CTX: vlan_macip_lens=%08x type_tucmd_mlhl=%08x mss_l4len_idx=%08x\n",
            d->vlan_macip_lens,
            d->type_tucmd_mlhl,
            d->mss_l4len_idx);
#endif
    le32_to_cpus(&d->vlan_macip_lens);
    le32_to_cpus(&d->ipsec_rsvd);
    le32_to_cpus(&d->type_tucmd_mlhl);
    le32_to_cpus(&d->mss_l4len_idx);

    //
    //NOTE: VLANS, SNAP, IPSEC, IPv6 not handled!
    //

    // 4th word:
    const uint32_t mss     = bitfield_extract(d->mss_l4len_idx,
                                              IGB_ADVTXD_MSSL4LEN_MSS);
    props->mss = mss;



    const uint32_t vlan  = bitfield_extract(d->vlan_macip_lens,
                                            IGB_ADVTXD_VLAN_MASK);
    props->vlan = vlan;
#if 0
    // 1st word:
    const uint32_t iplen   = bitfield_extract(d->vlan_macip_lens,
                                              IGB_ADVTXD_IPLEN_MASK);
    const uint32_t maclen  = bitfield_extract(d->vlan_macip_lens,
                                              IGB_ADVTXD_MACLEN_MASK);
    // VLAN not handled

    // 2nd word:
    // IPSEC not handled

    // 3rd word:
    const uint32_t l4t     = d->type_tucmd_mlhl & IGB_ADVTXD_TUCMD_L4T_MASK;
    const uint32_t is_ipv4 = bitfield_extract(d->type_tucmd_mlhl,
                                              IGB_ADVTXD_TUCMD_IPV4);

    // 4th word:
    const uint32_t idx     = bitfield_extract(d->mss_l4len_idx,
                                              IGB_ADVTXD_MSSL4LEN_IDX);
    const uint32_t l4len   = bitfield_extract(d->mss_l4len_idx,
                                              IGB_ADVTXD_MSSL4LEN_L4LEN);

    fprintf(stderr,
            "TX CTX:  maclen=%u iplen=%u ipv4=%u l4t_tcpcs=%u | idx=%u l4len=%u mss=%u\n",
            maclen,
            iplen,

            is_ipv4,
            (l4t & IGB_ADVTXD_TUCMD_L4T_MASK) == IGB_ADVTXD_TUCMD_L4T_TCP,

            idx,
            l4len,
            mss
           );
    fprintf(stderr,
            "-----------------------------------------------------------------\n");
#endif
}



static bool
igb_process_tx_desc(IgbTx *tx,
                    struct igb_tx_desc *dp,
                    igb_tx_func_t tx_func,
                    void *tx_arg)
{
    const uint32_t dtype = le32_to_cpu(dp->word2_type) & (IGB_TXD_CMD_DEXT |
                                                          IGB_TXD_DTYP_MASK);

    switch (dtype) {
    default:
        igb_abort("Unexpected TX dtype %x\n", dtype);
    case (IGB_TXD_CMD_DEXT | IGB_TXD_DTYP_CONTEXT): {
        /* context descriptor */
        struct igb_adv_tx_context_desc  *cdesc = ((struct igb_adv_tx_context_desc *)dp);
        igb_parse_tx_ctx_descr(cdesc, &tx->props);
        return true;
    }
    break;

    case (IGB_TXD_CMD_DEXT | IGB_TXD_DTYP_DATA): {
        const struct igb_adv_tx_data_desc  *dd = (struct igb_adv_tx_data_desc *)dp;

        const uint32_t txd_lower = le32_to_cpu(dd->lower.data);
        const uint32_t txd_upper = le32_to_cpu(dd->upper);

        unsigned int dtalen = txd_lower & 0xffff;
        const bool eop = txd_lower & IGB_TXD_CMD_EOP;

        tx->ip_sum_needed     |= bitfield_extract(txd_upper, IGB_TXD_POPTS_IXSM);
        tx->tcpudp_sum_needed |= bitfield_extract(txd_upper, IGB_TXD_POPTS_TXSM);
        tx->cptse             |= (txd_lower & IGB_TXD_CMD_TSE) ? 1 : 0;

        const uint64_t addr = le64_to_cpu(dd->buffer_addr);
        //skip_cp is really an error case, also  net_tx_* routines return false on ERROR

        // add fragment
        if (!tx->skip_cp && !net_tx_pkt_add_raw_fragment(tx->tx_pkt, addr, dtalen)) {
            tx->skip_cp = true;
            fprintf(stderr, "IGB ERROR: failed to add fragement!\n");
        }
#if 0
        const uint32_t paylen = bitfield_extract(txd_upper, IGB_TXD_PAYLEN_MASK);
        fprintf(stderr,
                "TX DD: upper=%08x || dtlen=%u eop=%d skip_cp=%d ip_sum_needed=%d tcpudp_sum=%d TSE=%d paylen=%d\n",
                txd_upper,
                dtalen,
                eop,
                tx->skip_cp,
                tx->ip_sum_needed,
                tx->tcpudp_sum_needed,
                tx->cptse,
                paylen);
#endif

        if (eop) {
            if (!tx->skip_cp && net_tx_pkt_parse(tx->tx_pkt)) {

#if 1
                // Unlike 82575, TX in the 82576 isn't affected by CTRL.VLE
                const bool is_vlan_txd = ((txd_lower & IGB_TXD_CMD_VLE) != 0);
                if (is_vlan_txd) {
                    //const uint16_t vet = le16_to_cpu(core->mac[VET]);
//TODO: if we really cared, get vlan ethertype from PF->mac[VET]
                    const uint16_t vet = 0x8100;
                    const uint16_t vlan = tx->props.vlan;
                    net_tx_pkt_setup_vlan_header_ex(tx->tx_pkt, vlan, vet);
                }
#endif

                igb_setup_tx_offloads(tx);
                net_tx_pkt_dump(tx->tx_pkt);
                tx_func(tx->tx_pkt, tx_arg); // Send packet,  statistics not implemented
            } else {
                fprintf(stderr, "IGB ERROR: failed to parse/tx packet!!\n");
            }

            // reset state:
            //memset(&tx->props, 0, sizeof(tx->props));
            tx->skip_cp = false;
            tx->ip_sum_needed = false;
            tx->tcpudp_sum_needed = false;
            tx->cptse = false;
            net_tx_pkt_reset(tx->tx_pkt);

            return true; //do writeback

        }
    }
    break;

    case 0: {
        /* legacy descriptor */
        // Technically possible, but igb Linux drivers do not use them.
        tx->cptse = 0;
        igb_abort("I was not expecting to see a legacy descriptor!");
    }
    break;
    };

    return false;
}

static void
igb_txdesc_writeback(PCIDevice *owner, dma_addr_t base,
                     struct igb_adv_tx_data_desc *dp, int queue_idx)
{
    uint32_t txd_lower = le32_to_cpu(dp->lower.data);

    if (!(txd_lower & IGB_TXD_CMD_RS)) {
        return;
    }
    uint32_t txd_upper = le32_to_cpu(dp->upper) | IGB_TXD_STAT_DD;
    dp->upper = cpu_to_le32(txd_upper);
    pci_dma_write(owner,
                  base + ((char *)&dp->upper - (char *)dp),
                  &dp->upper, sizeof(dp->upper));
}

bool
igb_xmit_ring(IgbTx *tx, const Igb_RingInfo *txi, PCIDevice *owner, igb_tx_func_t tx_func,
              void *tx_arg)
{
    bool do_interrupt = false;

    while (!igb_ring_empty(txi)) {
        struct igb_tx_desc desc = {};
        dma_addr_t base = igb_ring_head_descr(txi);
        do_interrupt = true;

        pci_dma_read(owner, base, &desc, sizeof(desc));

        bool wb;
        wb = igb_process_tx_desc(tx, &desc,  tx_func, tx_arg);
        if (wb) {
            igb_txdesc_writeback(owner, base, (struct igb_adv_tx_data_desc*)&desc,
                                 txi->idx);
        }
        igb_ring_advance(txi, 1);
    }

    return do_interrupt;
}


/* ######  #######  #####  #######   ###   #     # ####### */
/* #     # #       #     # #          #    #     # #       */
/* #     # #       #       #          #    #     # #       */
/* ######  #####   #       #####      #    #     # #####   */
/* #   #   #       #       #          #     #   #  #       */
/* #    #  #       #     # #          #      # #   #       */
/* #     # #######  #####  #######   ###      #    ####### */



static bool
igb_has_rxbufs(const Igb_RingInfo *r, size_t rxbuf_size, size_t total_size)
{
    int bufs;
    /* Fast-path short packets */
    if (total_size <= rxbuf_size) {
        return *r->dh != *r->dt;
    }

    if (*r->dh < *r->dt) {
        bufs = *r->dt - *r->dh;
    } else if (*r->dh > *r->dt) {
        //NOTE: dlen is in bytes, not descriptor count
        bufs = *r->dlen / IGB_RING_DESC_LEN - (*r->dh - *r->dt);
    } else {
        return false;
    }
    return total_size <= bufs * rxbuf_size;
}



static void
igb_read_rx_desc(PCIDevice *d, const Igb_RingInfo *rxi,
                 union igb_adv_rx_desc *desc)
{
    hwaddr base = igb_ring_head_descr(rxi);
    const uint64_t mask = 0xfffffffffffffffe;
    pci_dma_read(d, base, &desc->read, sizeof(desc->read));
    le64_to_cpus(&desc->read.pkt_addr);
    le64_to_cpus(&desc->read.hdr_addr);
    desc->read.pkt_addr &= mask;
    desc->read.hdr_addr &= mask;
}

static void
igb_write_rx_desc(PCIDevice *d, const Igb_RingInfo *rxi,
                  union igb_adv_rx_desc *desc, const bool is_rss)
{
    hwaddr base = igb_ring_head_descr(rxi);

    le16_to_cpus(&desc->wb.lower.lo_dword.pkt_info);
    le16_to_cpus(&desc->wb.lower.lo_dword.hdr_info);
    if (is_rss) {
        le32_to_cpus(&desc->wb.lower.hi_dword.rss);
    } else {
        le16_to_cpus(&desc->wb.lower.hi_dword.csum_ip.ip_id);
        le16_to_cpus(&desc->wb.lower.hi_dword.csum_ip.csum);
    }
    le32_to_cpus(&desc->wb.upper.status_error);
    le16_to_cpus(&desc->wb.upper.length);
    le16_to_cpus(&desc->wb.upper.vlan);

    pci_dma_write(d, base, &desc->read, sizeof(desc->read));
}




#if 0
static void igb_dump_iov(const struct iovec *iov_list, const unsigned iov_count,
                         const char *name)
{
    unsigned total = 0;
    fprintf(stderr, "IGB %10s: ", name);
    for (unsigned i = 0; i < iov_count; ++i) {
        for (unsigned k = 0; k < iov_list[i].iov_len; ++k) {
            fprintf(stderr, "%02hhx ", ((uint8_t*)iov_list[i].iov_base)[k]);
            ++total;
            if ((total % 32) == 0) {
                fprintf(stderr, "\n");
            } else if ((total % 16) == 0) {
                fprintf(stderr, " ");
            }
        }
    }
    if ((total % 32) != 0) {
        fprintf(stderr, "\n");
    }
}
#endif


typedef struct {
    const struct iovec *cur;
    off_t               offset;  // cur iov read offset
} iov_ptr_t;

static void
iovptr_advance(iov_ptr_t *readptr, off_t sz)
{
    readptr->offset += sz;
    if (readptr->offset >= readptr->cur->iov_len) {
        readptr->cur++;
        readptr->offset = 0;
    }
}

static off_t
iovptr_remain(const iov_ptr_t *readptr)
{
    return readptr->cur->iov_len - readptr->offset;
}

static void
igb_write_packet_to_guest(PCIDevice *d,
                          struct NetRxPkt *pkt,
                          const IgbRx *rx,
                          const Igb_RingInfo *rxi,
                          const uint32_t total_size)
{
    if (igb_ring_empty(rxi)) {
        igb_abort("RX: ring%u is full?\n", rxi->idx);
        return;
    }

    struct iovec *iov_list   = net_rx_pkt_get_iovec(pkt);
    //const unsigned iov_count = net_rx_pkt_get_iovec_len(pkt);

    uint32_t total_remain = total_size;
    const uint32_t packet_buf_size = rx->packet_size; // for each buffer

    iov_ptr_t readptr = {.cur = iov_list, .offset = 0 };

    bool hdr_done;

    switch (rx->descriptor_type) {
    case IGB_SRRCTL_DESCTYPE_ADV_ONEBUF:
        hdr_done = true; // this actually causes the logic to be skipped
        break;

    case IGB_SRRCTL_DESCTYPE_HDR_SPLIT_ALWAYS:
        hdr_done = false;
        break;

    default:
        igb_abort("Unexpected RX DESCTYPE: %u!\n",
                  bitfield_extract(rx->descriptor_type, IGB_SRRCTL_DESCTYPE_MASK));
    }


    // while we haven't written the whole packet yet
    while (total_remain) {

        // read receive descriptor:
        union igb_adv_rx_desc desc;
        igb_read_rx_desc(d, rxi, &desc);
        uint32_t hdr_written = 0;

        /*
         * This code really assumes that the HW is in the "always split" mode which igbvf uses for MTU > 2K
         * Only the 1st descriptor uses the header.   If we see that there is a header buffer,
         * chuck the full beginning of the packet into there.
         *
         * For Linux guest, the exact header boundaries dont' matter as
         * the whole thing gets put back into the same skb anyway...
         */
        if (!hdr_done) {
            hdr_done = true;
            uint32_t hdr_remain = rx->header_size;
            hwaddr   hdr_addr = desc.read.hdr_addr;

            while (hdr_remain && total_remain) {
                const off_t cur_remain = iovptr_remain(&readptr);
                const off_t to_copy = MIN(cur_remain, hdr_remain);
                pci_dma_write(d, hdr_addr,  readptr.cur->iov_base + readptr.offset, to_copy);

                total_remain -= to_copy;
                hdr_remain   -= to_copy;
                iovptr_advance(&readptr, to_copy);
                hdr_addr += to_copy;
                hdr_written += to_copy;
            }
        }

        hwaddr dest_addr = desc.read.pkt_addr;
        uint32_t packet_buf_remain = packet_buf_size;

        // while the current destination has space and we're not done:
        while (packet_buf_remain && total_remain) {
            const off_t cur_remain = iovptr_remain(&readptr);
            const off_t to_copy = MIN(cur_remain, packet_buf_remain);

            pci_dma_write(d, dest_addr,  readptr.cur->iov_base + readptr.offset, to_copy);

            dest_addr       += to_copy;
            iovptr_advance(&readptr, to_copy);

            packet_buf_remain -= to_copy;
            total_remain      -= to_copy;
        }

#if 0
        fprintf(stderr,
                "write_to_guest:  total=%u  hdr_done=%u  hdr_size=%u [%u] pkt_buf=%u length=%u total_remain=%u/%u\n",
                total_size,
                hdr_done,
                rx->header_size,
                hdr_written,
                packet_buf_size,
                (packet_buf_size - packet_buf_remain),
                total_remain,
                total_size);
#endif
        // rx desc writeback
        desc.wb.lower.lo_dword.pkt_info = 0; // RSS type, packet type
        desc.wb.lower.lo_dword.hdr_info = hdr_written <<
                                          IGB_RXD_VLAN_SHIFT; // split header, header length
        desc.wb.lower.hi_dword.rss      = 0;
        desc.wb.upper.status_error      = IGB_RXD_STAT_DD | (total_remain == 0 ?
                                                             IGB_RXD_STAT_EOP : 0);
        desc.wb.upper.length            = (packet_buf_size - packet_buf_remain);
        desc.wb.upper.vlan              = 0;

        igb_write_rx_desc(d, rxi, &desc, true);
        igb_ring_advance(rxi, 1);
    }
}





static inline void
igb_rx_fix_l4_csum(struct NetRxPkt *pkt)
{
    if (net_rx_pkt_has_virt_hdr(pkt)) {
        struct virtio_net_hdr *vhdr = net_rx_pkt_get_vhdr(pkt);

        if (vhdr->flags & VIRTIO_NET_HDR_F_NEEDS_CSUM) {
            net_rx_pkt_fix_l4_csum(pkt);
        }
    }
}

ssize_t
igb_rx_pkt_to_guest(PCIDevice *d,
                    struct NetRxPkt *rx_pkt,
                    const struct iovec *iov,
                    int iovcnt,
                    const igb_rx_params_t *params,
                    IgbRx        *rx,
                    const Igb_RingInfo *rxi,
                    const bool has_vnet)
{
    static const int maximum_ethernet_hdr_len = (14 + 4);
    /* Min. octets in an ethernet frame sans FCS */
    static const int min_buf_size = 60;

    uint8_t min_buf[min_buf_size];
    struct iovec min_iov;
    uint8_t *filter_buf;
    size_t size, orig_size;
    size_t iov_ofs = 0;
    size_t total_size;
    ssize_t retval;


    // if the ring is empty, it probably hasn't been setup yet.
    if (! *rxi->dlen) {
        return iov_size(iov, iovcnt);
    }

    if (0 == rx->packet_size) {
        fprintf(stderr, "IGB: RX pkt buf size is zero [f=%u q=%u]!\n",
                params->debug_function, rxi->idx);
        return iov_size(iov, iovcnt);
    }

    /* Pull virtio header in */
    if (has_vnet) {
        net_rx_pkt_set_vhdr_iovec(rx_pkt, iov, iovcnt);
        iov_ofs = sizeof(struct virtio_net_hdr);
    }

    filter_buf = iov->iov_base + iov_ofs;
    orig_size = iov_size(iov, iovcnt);
    size = orig_size - iov_ofs;

    /* Pad to minimum Ethernet frame length */
    if (size < sizeof(min_buf)) {
        iov_to_buf(iov, iovcnt, iov_ofs, min_buf, size);
        memset(&min_buf[size], 0, sizeof(min_buf) - size);

        //RUC stat not updated!

        min_iov.iov_base = filter_buf = min_buf;
        min_iov.iov_len = size = sizeof(min_buf);
        iovcnt = 1;
        iov = &min_iov;
        iov_ofs = 0;
    } else if (iov->iov_len < maximum_ethernet_hdr_len) {
        /* This is very unlikely, but may happen. */
        iov_to_buf(iov, iovcnt, iov_ofs, min_buf, maximum_ethernet_hdr_len);
        filter_buf = min_buf;
    }

    //TODO Discard oversized packets if !LPE and !SBP.
#if 0
    /* Discard oversized packets if !LPE and !SBP. */
    if (e1000x_is_oversized(core->mac, size)) {
        return orig_size;
    }
#endif

    net_rx_pkt_set_packet_type(rx_pkt,
                               get_eth_packet_type(PKT_GET_ETH_HDR(filter_buf)));


    //TODO: receive Filter
#if 0
    if (!igb_receive_filter(core, filter_buf, size)) {
        trace_igb_rx_flt_dropped();
        return orig_size;
    }
#endif

    net_rx_pkt_attach_iovec_ex(rx_pkt, iov, iovcnt, iov_ofs,
                               params->vlan_mode_enabled, params->vlan_ethertype);

    total_size = net_rx_pkt_get_total_len(rx_pkt) + (params->strip_crc ? 0 : 4);

    if (igb_has_rxbufs(rxi, rx->packet_size, total_size)) {
        igb_rx_fix_l4_csum(rx_pkt);

        retval = orig_size;
        igb_write_packet_to_guest(d, rx_pkt, rx, rxi, total_size);
    } else {
        //Overflow!
        retval = 0;
    }

    return retval;
}

