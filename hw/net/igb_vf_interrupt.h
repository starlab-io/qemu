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

#ifndef HW_NET_IGB_VF_INTERRUPT_H
#define HW_NET_IGB_VF_INTERRUPT_H


#include "qemu/osdep.h"
#include "igb_vf_core.h"

// INTERRUPTS:

void
igb_vf_raise_tx_interrupt(IgbVfCore *core, unsigned queue);

void
igb_vf_raise_rx_interrupt(IgbVfCore *core, unsigned queue);

void
igb_vf_raise_mailbox_interrupt(IgbVfCore *core);


// READS:

uint32_t
igb_vf_mac_eicr_read(IgbVfCore *core, int index);


// WRITES:

void
igb_vf_set_eicr(IgbVfCore *core, int index, uint32_t val);


void
igb_vf_set_eimc(IgbVfCore *core, int index, uint32_t val);


void
igb_vf_set_eiam(IgbVfCore *core, int index, uint32_t val);


void
igb_vf_set_eiac(IgbVfCore *core, int index, uint32_t val);


void
igb_vf_set_eims(IgbVfCore *core, int index, uint32_t val);

void
igb_vf_set_eics(IgbVfCore *core, int index, uint32_t val);

void
igb_vf_interrupt_timer(void *);
#endif
