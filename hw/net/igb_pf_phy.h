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

#ifndef HW_NET_IGB_PF_PHY_H
#define HW_NET_IGB_PF_PHY_H


#include "qemu/osdep.h"
#include "igb_pf_core.h"

void
igb_pf_set_mdic(IgbPfCore *core, int index, uint32_t val);

void
igb_pf_core_phy_reset(IgbPfCore *core);
#endif
