/*
* Core code for QEMU igb emulation
*
* Software developer's manuals:
* http://www.intel.com/content/dam/doc/datasheet/82574l-gbe-controller-datasheet.pdf
*
* Copyright (c) 2015 Ravello Systems LTD (http://ravellosystems.com)
* Developed by Daynix Computing LTD (http://www.daynix.com)
*
* Authors:
* Dmitry Fleytman <dmitry@daynix.com>
* Leonid Bloch <leonid@daynix.com>
* Yan Vugenfirer <yan@daynix.com>
*
* Based on work done by:
* Nir Peleg, Tutis Systems Ltd. for Qumranet Inc.
* Copyright (c) 2008 Qumranet
* Based on work done by:
* Copyright (c) 2007 Dan Aloni
* Copyright (c) 2004 Antony T Curtis
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HW_NET_IGB_PF_CORE_H
#define HW_NET_IGB_PF_CORE_H

#include "exec/hwaddr.h"
#include "net/net.h"
#include "igb_ring.h"

#define IGB_PHY_PAGE_SIZE    (0x20)
#define IGB_PHY_PAGES        (0x07)
#define IGB_PF_MAC_SIZE      (0x8000)
#define IGB_EEPROM_SIZE      (128) //TODO: how big is this?

#define IGB_PF_MSIX_VEC_NUM     (25)
#define IGB_PF_NUM_QUEUES       (16)

#include "igb_ring.h"


typedef struct IgbPfCore IgbPfCore;
struct IgbPfCore {
    uint32_t mac[IGB_PF_MAC_SIZE];
    uint16_t phy[IGB_PHY_PAGES][IGB_PHY_PAGE_SIZE];
    uint16_t eeprom[IGB_EEPROM_SIZE];

    // Transmit
    IgbTx tx[IGB_PF_NUM_QUEUES];
    IgbRx rx[IGB_PF_NUM_QUEUES];
    const Igb_RingInfo rxi[IGB_PF_NUM_QUEUES];
    const Igb_RingInfo txi[IGB_PF_NUM_QUEUES];

    // Receive
    struct NetRxPkt *rx_pkt;

    bool has_vnet; //for external interface only

    uint8_t permanent_mac[6];

    // ---------------------
    NICState *owner_nic;
    PCIDevice *owner;
};

void
igb_pf_core_write(IgbPfCore *core, hwaddr addr, uint64_t val, unsigned size);

uint64_t
igb_pf_core_read(IgbPfCore *core, hwaddr addr, unsigned size);

void
igb_pf_core_pci_realize(IgbPfCore      *regs,
                        const uint16_t *eeprom_templ,
                        uint32_t        eeprom_size,
                        const uint8_t  *macaddr);

void
igb_pf_core_reset(IgbPfCore *core);

int
igb_pf_core_post_load(IgbPfCore *core);

void
igb_pf_core_pci_uninit(IgbPfCore *core);

bool
igb_pf_can_receive(IgbPfCore *core);

void
igb_pf_core_set_link_status(IgbPfCore *core, bool link_down);
#endif
