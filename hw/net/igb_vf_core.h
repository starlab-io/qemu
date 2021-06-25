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


#ifndef HW_NET_IGB_VF_CORE_H
#define HW_NET_IGB_VF_CORE_H

#include "exec/hwaddr.h"
#include "net/net.h"
#include "igb_common.h"

#include "igb_ring.h"

#define IGB_VF_MAC_SIZE    16384
#define IGB_VF_NUM_QUEUES  2

typedef struct IgbVfCore IgbVfCore;
struct IgbVfCore {
    uint32_t mac[IGB_VF_MAC_SIZE];

    IgbTx         tx[IGB_VF_NUM_QUEUES];
    IgbRx         rx[IGB_VF_NUM_QUEUES];
    Igb_RingInfo  txi[IGB_VF_NUM_QUEUES];
    Igb_RingInfo  rxi[IGB_VF_NUM_QUEUES];

	// Crude Interrupt Throttling
	QEMUTimer *interrupt_timer;
	uint32_t pending_interrupts;

    struct NetRxPkt *rx_pkt;

    PCIDevice *owner;
};

void
igb_vf_core_write(IgbVfCore *core, hwaddr addr, uint64_t val, unsigned size);

uint64_t
igb_vf_core_read(IgbVfCore *core, hwaddr addr, unsigned size);

void
igb_vf_core_pci_realize(IgbVfCore *core);


void
igb_vf_core_reset(IgbVfCore *core);


void
igb_vf_core_pci_uninit(IgbVfCore *core);

// ------------------------------------------------------------


uint32_t *
igb_vf_reg(IgbVfCore *core, uint32_t offset);

void
igb_vf_mac_writereg(IgbVfCore *core, int index, uint32_t val);


#endif
