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

#ifndef HW_NET_IGB_PF_INTERRUPT_H
#define HW_NET_IGB_PF_INTERRUPT_H


#include "qemu/osdep.h"
#include "igb_pf_core.h"

// INTERRUPTS:

bool
igb_is_msix_mode(IgbPfCore *core);


void
igb_pf_raise_tx_interrupt(IgbPfCore *core, unsigned queue);

void
igb_pf_raise_rx_interrupt(IgbPfCore *core, unsigned queue);

void
igb_pf_raise_other_interrupt(IgbPfCore *core, uint32_t icr);

void
igb_pf_raise_mailbox_interrupt(IgbPfCore *core, unsigned vf);


// READS:

uint32_t
igb_pf_mac_icr_read(IgbPfCore *core, int index);


uint32_t
igb_pf_mac_eicr_read(IgbPfCore *core, int index);


// WRITES:

void
igb_pf_set_eicr(IgbPfCore *core, int index, uint32_t val);


void
igb_pf_set_imc(IgbPfCore *core, int index, uint32_t val);

void
igb_pf_set_ims(IgbPfCore *core, int index, uint32_t val);


void
igb_pf_set_eimc(IgbPfCore *core, int index, uint32_t val);


void
igb_pf_set_eiam(IgbPfCore *core, int index, uint32_t val);


void
igb_pf_set_eiac(IgbPfCore *core, int index, uint32_t val);


void
igb_pf_set_eims(IgbPfCore *core, int index, uint32_t val);

void
igb_pf_set_eics(IgbPfCore *core, int index, uint32_t val);


void
igb_pf_interrupt_timer(void *opaque);

#endif
