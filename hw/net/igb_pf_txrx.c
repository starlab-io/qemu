/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation
 *
 * Copyright (c) 2021
 *
 * Author:
 *   Alex Olson, Star Lab
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "igb_pf_txrx.h"
#include "igb_common.h"
#include "igb_pf_reg.h"
#include "igb_pf_interrupt.h"
#include "igb_pf_reg.h"
#include "igb.h"
#include "net_tx_pkt.h"
#include "net_rx_pkt.h"

#include "qemu/bitops.h"
#include "qemu/log.h"
#include "trace.h"

/* ####### ######     #    #     #  #####  #     #   ###   ####### */
/*    #    #     #   # #   ##    # #     # ##   ##    #       #   */
/*    #    #     #  #   #  # #   # #       # # # #    #       #   */
/*    #    ######  #     # #  #  #  #####  #  #  #    #       #   */
/*    #    #   #   ####### #   # #       # #     #    #       #   */
/*    #    #    #  #     # #    ## #     # #     #    #       #   */
/*    #    #     # #     # #     #  #####  #     #   ###      #   */




void
igb_pf_core_prepare_tx(IgbPfCore *core)
{
    for (int i = 0; i < IGB_PF_NUM_QUEUES; ++i) {
        Igb_RingInfo *ri = (Igb_RingInfo*)&core->txi[i];

        ri->dbah = &core->mac[REG2IDX(IGB_TDBAH(i))];
        ri->dbal = &core->mac[REG2IDX(IGB_TDBAL(i))];
        ri->dlen = &core->mac[REG2IDX(IGB_TDLEN(i))];
        ri->dh   = &core->mac[REG2IDX(IGB_TDH(i))];
        ri->dt   = &core->mac[REG2IDX(IGB_TDT(i))];
        ri->idx  = i;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////

static void
our_tx_pkt_sendv(struct NetTxPkt *pkt,
                 NetClientState *nc,
                 const struct iovec *iov, int iov_cnt,
                 void *param)
{
    igb_send_context_t *context = (igb_send_context_t*)param;
    igb_broadcast_pkt(context, iov, iov_cnt);
}

// Callback that get executed from igb_xmit_ring()
static void
igb_pf_tx_pkt_send(struct NetTxPkt *tx_pkt, void *_core)
{
    // In reality, there is only one queue...
    IgbPfCore *core = (IgbPfCore*)_core;
    NetClientState *nc = qemu_get_subqueue(core->owner_nic, 0);
    assert(tx_pkt != NULL);

    // Logic for doing loopback skipped (net_tx_pkt_send_loopback not called)
    igb_send_context_t context = {
        .core = core,
        .source = IGB_SOURCE_PF,
    };
    net_tx_pkt_send_ex(tx_pkt, nc, our_tx_pkt_sendv, &context);
}


static void
igb_start_xmit(IgbPfCore *core, IgbTx *tx, const Igb_RingInfo *txi)
{
    bool do_interrupt = false;

    if (!(core->mac[TCTL] & IGB_TCTL_EN)) {
        trace_igb_tx_disabled();
        return;
    }

    do_interrupt = igb_xmit_ring(tx, txi, core->owner, igb_pf_tx_pkt_send, core);

    if (do_interrupt) {
        igb_pf_raise_tx_interrupt(core, txi->idx);
    }
}

static const int array_TDT[] = {
    TDT0,
    TDT1,
    TDT2,
    TDT3,
    TDT4,
    TDT5,
    TDT6,
    TDT7,
    TDT8,
    TDT9,
    TDT10,
    TDT11,
    TDT12,
    TDT13,
    TDT14,
    TDT15
};


void
igb_pf_set_tdt(IgbPfCore *core, int index, uint32_t val)
{
    const unsigned qnum = IGB_LOOKUP_VALUE(index, array_TDT);

    core->mac[index] = val;
    igb_start_xmit(core, &core->tx[qnum], &core->txi[qnum]);
}


void
igb_pf_set_tctl(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[index] = val;

    if (val & IGB_TCTL_EN) {
        for (int qnum = 0; qnum < IGB_PF_NUM_QUEUES; ++qnum) {
            igb_start_xmit(core, &core->tx[qnum], &core->txi[qnum]);
        }
    }
}




/* ######  #######  #####  #######   ###   #     # ####### */
/* #     # #       #     # #          #    #     # #       */
/* #     # #       #       #          #    #     # #       */
/* ######  #####   #       #####      #    #     # #####   */
/* #   #   #       #       #          #     #   #  #       */
/* #    #  #       #     # #          #      # #   #       */
/* #     # #######  #####  #######   ###      #    ####### */



void
igb_pf_core_prepare_rx(IgbPfCore *core)
{
    for (int i = 0; i < IGB_PF_NUM_QUEUES; ++i) {
        Igb_RingInfo *ri = (Igb_RingInfo*)&core->rxi[i];

        ri->dbah = &core->mac[REG2IDX(IGB_RDBAH(i))];
        ri->dbal = &core->mac[REG2IDX(IGB_RDBAL(i))];
        ri->dlen = &core->mac[REG2IDX(IGB_RDLEN(i))];
        ri->dh   = &core->mac[REG2IDX(IGB_RDH(i))];
        ri->dt   = &core->mac[REG2IDX(IGB_RDT(i))];
        ri->idx  = i;
    }
}

static uint32_t
igb_parse_rctl_bufsize(const uint32_t rctl)
{
    switch (rctl & IGB_RCTL_SZ_MASK) {
    case IGB_RCTL_SZ_2048:
        return 2048;
    case IGB_RCTL_SZ_1024:
        return 1024;
    case IGB_RCTL_SZ_512 :
        return  512;
    case IGB_RCTL_SZ_256 :
        return  256;
    }
    abort();
}

static void
igb_parse_srrctl_size(const uint32_t srrctl, uint32_t *sz_packet,
                      uint32_t *sz_header)
{
    *sz_packet = bitfield_extract(srrctl,
                                  IGB_SRRCTL_BSIZEPACKET) * IGB_SRRCTL_BSIZEPACKET_UNIT;
    *sz_header = bitfield_extract(srrctl,
                                  IGB_SRRCTL_BSIZEHEADER) * IGB_SRRCTL_BSIZEHEADER_UNIT;
}

static void
igb_update_rxbuf_sizes(IgbPfCore *core)
{
    uint32_t packet_size = igb_parse_rctl_bufsize(core->mac[RCTL]);

    for (unsigned k = 0; k < IGB_PF_NUM_QUEUES; ++k) {
        const uint32_t srrctl = core->mac[REG2IDX(IGB_SRRCTL(k))];
        igb_parse_srrctl_size(srrctl, &core->rx[k].packet_size,
                              &core->rx[k].header_size);
        core->rx[k].descriptor_type = srrctl &  IGB_SRRCTL_DESCTYPE_MASK;
        if (core->rx[k].packet_size == 0) {
            core->rx[k].packet_size = packet_size;
        }
    }
}

void
igb_pf_set_rctl(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[index] = val;

    igb_update_rxbuf_sizes(core);

    if ((core->mac[RCTL] & IGB_RCTL_EN)) {
        //TODO: start RX if can_receive doesn't always return true
    }

}

void
igb_pf_set_srrctl(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[index] = val;
    //const unsigned qnum = IGB_LOOKUP_VALUE(index, array_SRRCTL);
    igb_update_rxbuf_sizes(core);
}

void
igb_pf_set_rdt(IgbPfCore *core, int index, uint32_t val)
{

    core->mac[index] = val;
    //TODO:  start RX here if can_receive doesn't always return true
}


static void
igb_pf_build_rx_params(IgbPfCore *core, igb_rx_params_t *p)
{
    p->vlan_mode_enabled = bitfield_extract(core->mac[CTRL], IGB_CTRL_VME);
    p->vlan_ethertype    = core->mac[VET] & 0xffff;
    p->pad_small_packets = bitfield_extract(core->mac[RCTL], IGB_RCTL_PSP);
    p->strip_crc         = bitfield_extract(core->mac[RCTL], IGB_RCTL_SECRC);
    p->debug_function = -1;
}


static unsigned igb_pf_determine_rx_queue(IgbPfCore *core)
{
    unsigned ret;
    const uint32_t mrqc = core->mac[MRQC];

    switch (bitfield_extract(mrqc, IGB_MRQC_ENABLE_MASK)) {

    /*
     * Use Def_Q value from MRQ registers
     */
    case 0:  //Disabled
    case 2:  //RSS for 16 queues
    case 6:  //RSS
        // When only PF used, Linux configures RSS-16queues mode.
        ret = bitfield_extract(mrqc, IGB_MRQC_DEFQ_MASK);
        break;


    /* assume VMDq, use VT_CTL to get default queue */
    default:
        ret = bitfield_extract(core->mac[VT_CTL], IGB_VT_CTL_DEF_PL_MASK);
        break;
    }
    return ret;
}
ssize_t
igb_pf_receive_iov(IgbPfCore *core, const struct iovec *iov, int iovcnt,
                   const bool has_vnet)
{
    igb_rx_params_t params;
    igb_pf_build_rx_params(core, &params);

    //check if RX enabled
    if (!(core->mac[RCTL] & IGB_RCTL_EN)) {
        return -1;
    }

    int qnum = igb_pf_determine_rx_queue(core);
    //fprintf(stderr, "PF: RX queue is %d MRQC=%08x VT_CTL=%08x\n", qnum, core->mac[MRQC], core->mac[VT_CTL]);
    ssize_t sz;
    sz = igb_rx_pkt_to_guest(core->owner, core->rx_pkt, iov, iovcnt, &params,
                             &core->rx[qnum], &core->rxi[qnum], has_vnet);

    if (sz) {
        igb_pf_raise_rx_interrupt(core, qnum);
    } else {
        igb_pf_raise_other_interrupt(core, IGB_ICR_RXO);
    }
    return sz;
}
