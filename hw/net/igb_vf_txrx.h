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

#ifndef HW_NET_IGB_VF_TXRX_H
#define HW_NET_IGB_VF_TXRX_H


#include "qemu/osdep.h"
#include "igb_vf_core.h"

/* ####### ######     #    #     #  #####  #     #   ###   ####### */
/*    #    #     #   # #   ##    # #     # ##   ##    #       #   */
/*    #    #     #  #   #  # #   # #       # # # #    #       #   */
/*    #    ######  #     # #  #  #  #####  #  #  #    #       #   */
/*    #    #   #   ####### #   # #       # #     #    #       #   */
/*    #    #    #  #     # #    ## #     # #     #    #       #   */
/*    #    #     # #     # #     #  #####  #     #   ###      #   */



void
igb_vf_core_prepare_tx(IgbVfCore *core);

void
igb_vf_set_tdt(IgbVfCore *core, int index, uint32_t val);

/* ######  #######  #####  #######   ###   #     # ####### */
/* #     # #       #     # #          #    #     # #       */
/* #     # #       #       #          #    #     # #       */
/* ######  #####   #       #####      #    #     # #####   */
/* #   #   #       #       #          #     #   #  #       */
/* #    #  #       #     # #          #      # #   #       */
/* #     # #######  #####  #######   ###      #    ####### */


void
igb_vf_core_prepare_rx(IgbVfCore *core);

void
igb_vf_set_rdt(IgbVfCore *core, int index, uint32_t val);

void
igb_vf_set_srrctl(IgbVfCore *core, int index, uint32_t val);

ssize_t
igb_vf_receive_iov(IgbVfCore *core, const struct iovec *iov, int iovcnt,
                   const bool has_vnet);

#endif
