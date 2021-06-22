/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation -- Mailbox
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

#ifndef HW_NET_IGB_MAILBOX_H
#define HW_NET_IGB_MAILBOX_H

#include "qemu/osdep.h"
#include "igb.h"

void
igb_pf_set_mailbox(IgbPfCore *core, int index, uint32_t val);

uint32_t
igb_pf_read_mailbox(IgbPfCore *core, int index);


void
igb_vf_set_mailbox(IgbVfCore *core, int index, uint32_t val);

uint32_t
igb_vf_read_mailbox(IgbVfCore *core, int index);


void
igb_vf_set_vmbmem(IgbVfCore *core, int index, uint32_t val);

uint32_t
igb_vf_read_vmbmem(IgbVfCore *core, int index);


void
igb_pf_set_vmbmem(IgbPfCore *core, int index, uint32_t val);

uint32_t
igb_pf_read_vmbmem(IgbPfCore *core, int index);


void
igb_vf_mailbox_realize(IgbVfCore *core);

void
igb_pf_mailbox_reset_done(IgbPfCore *core);

#endif
