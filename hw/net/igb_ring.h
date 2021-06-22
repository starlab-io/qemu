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


#ifndef HW_NET_IGB_RING_H
#define HW_NET_IGB_RING_H

#include "igb_common.h"
#include "hw/pci/pci.h"

typedef struct Igb_RingInfo_st {
    uint32_t *dbah;
    uint32_t *dbal;
    uint32_t *dlen;
    uint32_t *dh;
    uint32_t *dt;
    int idx;
} Igb_RingInfo;

typedef struct igbx_txd_props {
    /* We could derive start/end/offset values for IP checksum & TCP/UDP
       Checksum, however the "net_tx" abstraction layer doesn't need them
       as it parses packets on its own.

     *   *css - checksum start   (place to start from)
     *   *cso - checksum offset  (place to write)
     *   *cse - checksum end     (place for checksum to stop, all zeros mean EOP)
     */

    /* IP Checksum */
    //uint8_t  ipcss;
    //uint8_t  ipcso;
    //uint16_t ipcse;

    /* TCP/UDP Checksum */
    //uint8_t  tucss;
    //uint8_t  tucso;
    //uint16_t tucse;

    /* TCP Segmentation Offload (TSO) */
    //uint32_t paylen;    // Total TCP payload size
    //uint8_t  l4len;     // length of header (as a prototype)
    uint16_t mss;
    uint16_t vlan;
    //int8_t   ip;        // IPv4=1, IPv6=0
    //int8_t   tcp;       // TCP=1,  UDP=0
} igbx_txd_props;


typedef struct IgbTx_st {
    igbx_txd_props props;        //parsed TX context descriptor

    bool     skip_cp;            // internal state
    bool     ip_sum_needed;      //TXD POPTS value
    bool     tcpudp_sum_needed;  //TXD POPTS value
    bool     cptse;              //Do Checksum & segmentation
    struct NetTxPkt *tx_pkt;
} IgbTx;

typedef struct IgbRx_st {
    uint32_t descriptor_type;  //masked from srrctl
    uint32_t packet_size;
    uint32_t header_size;
} IgbRx;



/* ####### ######     #    #     #  #####  #     #   ###   ####### */
/*    #    #     #   # #   ##    # #     # ##   ##    #       #   */
/*    #    #     #  #   #  # #   # #       # # # #    #       #   */
/*    #    ######  #     # #  #  #  #####  #  #  #    #       #   */
/*    #    #   #   ####### #   # #       # #     #    #       #   */
/*    #    #    #  #     # #    ## #     # #     #    #       #   */
/*    #    #     # #     # #     #  #####  #     #   ###      #   */




typedef void (*igb_tx_func_t)(struct NetTxPkt *pkt, void *arg);

bool
igb_xmit_ring(IgbTx *tx, const Igb_RingInfo *txi, PCIDevice *owner, igb_tx_func_t tx_func, void *tx_arg);

/* ######  #######  #####  #######   ###   #     # ####### */
/* #     # #       #     # #          #    #     # #       */
/* #     # #       #       #          #    #     # #       */
/* ######  #####   #       #####      #    #     # #####   */
/* #   #   #       #       #          #     #   #  #       */
/* #    #  #       #     # #          #      # #   #       */
/* #     # #######  #####  #######   ###      #    ####### */

typedef struct {
    bool     vlan_mode_enabled;
    uint16_t vlan_ethertype;
    bool     pad_small_packets;
    bool     strip_crc;
    int      debug_function;
    //TODO:  LongPacket Enable?
} igb_rx_params_t;




struct NetRxPkt;

ssize_t
igb_rx_pkt_to_guest(PCIDevice *d,
                    struct NetRxPkt *rx_pkt,
                    const struct iovec *iov,
                    int iovcnt,
                    const igb_rx_params_t *params,
                    IgbRx *rx,
                    const Igb_RingInfo *rxi,
                    const bool has_vnet);

#endif
