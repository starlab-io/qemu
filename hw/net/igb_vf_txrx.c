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


#include "igb_vf_txrx.h"
#include "igb_common.h"
#include "igb_vf_reg.h"
#include "igb_vf_interrupt.h"
#include "net_tx_pkt.h"
#include "net_rx_pkt.h"

#include "igb_pf_core.h"
#include "igb.h"

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
igb_vf_core_prepare_tx(IgbVfCore *core)
{
#if IGB_VF_NUM_QUEUES != 2
#error "wrong IGB_VF_NUM_QUEUES ?"
#endif
    Igb_RingInfo *ri;

    //1st queue:
    ri = (Igb_RingInfo*)&core->txi[0];
    ri->dbah = igb_vf_reg(core, IGB_VF_VTTDBAH0);
    ri->dbal = igb_vf_reg(core, IGB_VF_VTTDBAL0);
    ri->dlen = igb_vf_reg(core, IGB_VF_VTTDLEN0);
    ri->dh   = igb_vf_reg(core, IGB_VF_VTTDH0);
    ri->dt   = igb_vf_reg(core, IGB_VF_VTTDT0);
    ri->idx  = 0;

    //2nd queue:
    ri =(Igb_RingInfo*) &core->txi[1];
    ri->dbah = igb_vf_reg(core, IGB_VF_VTTDBAH1);
    ri->dbal = igb_vf_reg(core, IGB_VF_VTTDBAL1);
    ri->dlen = igb_vf_reg(core, IGB_VF_VTTDLEN1);
    ri->dh   = igb_vf_reg(core, IGB_VF_VTTDH1);
    ri->dt   = igb_vf_reg(core, IGB_VF_VTTDT1);
    ri->idx  = 1;
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
igb_vf_tx_pkt_send(struct NetTxPkt *tx_pkt, void *_core)
{
    assert(tx_pkt != NULL);
    IgbVfCore *vf_core = (IgbVfCore*)_core;
    IgbPfCore *pf_core = igb_get_pf_core(vf_core);
    NetClientState *nc = qemu_get_queue(pf_core->owner_nic);
    igb_send_context_t context = {
        .core = pf_core,
        .source = igb_get_vf_num(vf_core),
    };
    // Logic for doing loopback skipped (net_tx_pkt_send_loopback not called)
    net_tx_pkt_send_ex(tx_pkt, nc, our_tx_pkt_sendv, &context);
}


static void
igb_vf_start_xmit(IgbVfCore *core, IgbTx *tx, const Igb_RingInfo *txi)
{
    bool do_interrupt;

    do_interrupt = igb_xmit_ring(tx, txi, core->owner, igb_vf_tx_pkt_send, core);

    if (do_interrupt) {
        igb_vf_raise_tx_interrupt(core, txi->idx);
    }
}

static const int array_TDT[] = {
    REG2IDX(IGB_VF_VTTDT0),
    REG2IDX(IGB_VF_VTTDT1),
};


void
igb_vf_set_tdt(IgbVfCore *core, int index, uint32_t val)
{
    const unsigned qnum = IGB_LOOKUP_VALUE(index, array_TDT);

    core->mac[index] = val;
    igb_vf_start_xmit(core, &core->tx[qnum], &core->txi[qnum]);
}


/* ######  #######  #####  #######   ###   #     # ####### */
/* #     # #       #     # #          #    #     # #       */
/* #     # #       #       #          #    #     # #       */
/* ######  #####   #       #####      #    #     # #####   */
/* #   #   #       #       #          #     #   #  #       */
/* #    #  #       #     # #          #      # #   #       */
/* #     # #######  #####  #######   ###      #    ####### */


void
igb_vf_core_prepare_rx(IgbVfCore *core)
{
#if IGB_VF_NUM_QUEUES != 2
#error "wrong IGB_VF_NUM_QUEUES ?"
#endif
    Igb_RingInfo *ri;

    //1st queue:
    ri = (Igb_RingInfo*)&core->rxi[0];
    ri->dbah = igb_vf_reg(core, IGB_VF_VTRDBAH0);
    ri->dbal = igb_vf_reg(core, IGB_VF_VTRDBAL0);
    ri->dlen = igb_vf_reg(core, IGB_VF_VTRDLEN0);
    ri->dh   = igb_vf_reg(core, IGB_VF_VTRDH0);
    ri->dt   = igb_vf_reg(core, IGB_VF_VTRDT0);
    ri->idx  = 0;

    //2nd queue:
    ri = (Igb_RingInfo*)&core->rxi[1];
    ri->dbah = igb_vf_reg(core, IGB_VF_VTRDBAH1);
    ri->dbal = igb_vf_reg(core, IGB_VF_VTRDBAL1);
    ri->dlen = igb_vf_reg(core, IGB_VF_VTRDLEN1);
    ri->dh   = igb_vf_reg(core, IGB_VF_VTRDH1);
    ri->dt   = igb_vf_reg(core, IGB_VF_VTRDT1);
    ri->idx  = 1;
}

/*
static const int array_RDT[] = {
    REG2IDX(IGB_VF_VTRDT0),
    REG2IDX(IGB_VF_VTRDT1),
};
*/

static const int array_SRRCTL[] = {
    REG2IDX(IGB_VF_VTSRRCTL0),
    REG2IDX(IGB_VF_VTSRRCTL1),
};

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
igb_vf_update_rxbuf_sizes(IgbVfCore *core)
{
    for (unsigned k = 0; k < IGB_VF_NUM_QUEUES; ++k) {
        const uint32_t srrctl = core->mac[array_SRRCTL[k]];
        igb_parse_srrctl_size(srrctl, &core->rx[k].packet_size,
                              &core->rx[k].header_size);
        core->rx[k].descriptor_type = srrctl &  IGB_SRRCTL_DESCTYPE_MASK;
    }
}


void
igb_vf_set_srrctl(IgbVfCore *core, int index, uint32_t val)
{
    core->mac[index] = val;

    //const unsigned qnum = IGB_LOOKUP_VALUE(index, array_SRRCTL);
    //fprintf(stderr, "VF[%u]  SRRCTL[%u] <= %08x\n", igb_get_vf_num(core), qnum, val);

    igb_vf_update_rxbuf_sizes(core);
}

void
igb_vf_set_rdt(IgbVfCore *core, int index, uint32_t val)
{
    core->mac[index] = val;
	igb_start_recv(igb_get_pf_core(core));
}


static void
igb_vf_build_rx_params(IgbVfCore *core, igb_rx_params_t *p)
{
    p->vlan_mode_enabled = false;
    p->vlan_ethertype    = 0x8100;
    p->pad_small_packets = false;
    p->strip_crc         = true;
    p->debug_function =  igb_get_vf_num(core);
}


ssize_t
igb_vf_receive_iov(IgbVfCore *core, const struct iovec *iov, int iovcnt,
                   const bool has_vnet)
{
    igb_rx_params_t params;
    igb_vf_build_rx_params(core, &params);

    int qnum = 0;
    ssize_t sz;
    sz = igb_rx_pkt_to_guest(core->owner, core->rx_pkt, iov, iovcnt, &params,
                             &core->rx[qnum], &core->rxi[qnum], has_vnet);

    if (sz) {
        if (qnum != -1) {
            igb_vf_raise_rx_interrupt(core, qnum);
        } else {
            // dropped
        }
    } else {
        // overrun
    }
    return sz;
}
