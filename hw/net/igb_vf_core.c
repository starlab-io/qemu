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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/pci/pci.h"

#include "net_tx_pkt.h"
#include "net_rx_pkt.h"

#include "igb.h"
#include "igb_vf_core.h"
#include "igb_pf_core.h"

#include "igb_vf_reg.h"
#include "igb_vf_interrupt.h"
#include "igb_mailbox.h"

#include "igb_vf_txrx.h"

#include "igb_pf_reg_constants.h"
#include "igb_pf_interrupt.h"
#include "igb_vf_reg_to_name.h"

#include "trace.h"

#pragma GCC diagnostic error "-Woverride-init"

uint32_t *
igb_vf_reg(IgbVfCore *core, uint32_t offset)
{
    assert((offset + 3) < IGB_VF_MAC_SIZE);
    return &core->mac[REG2IDX(offset)];
}

static void
igb_vf_set_ctrl(IgbVfCore *core, int index, uint32_t val)
{
    if (val & IGB_VF_VTCTRL_RST) {

        //fprintf(stderr, "VTCTRL_RST: VF=%u\n", igb_get_vf_num(core));
        //RESET queue enable and interrupt registers
        *igb_vf_reg(core, IGB_VF_VTTXDCTL0) = 0;
        *igb_vf_reg(core, IGB_VF_VTTXDCTL1) = 0;

        *igb_vf_reg(core, IGB_VF_VTRXDCTL0) = 0;
        *igb_vf_reg(core, IGB_VF_VTRXDCTL1) = 0;

        *igb_vf_reg(core, IGB_VF_VTRXDCTL1) = 0;

        *igb_vf_reg(core, IGB_VF_VTEICR) = 0;
        *igb_vf_reg(core, IGB_VF_VTEIMC) = 0;
        *igb_vf_reg(core, IGB_VF_VTIVAR0) = 0;
        *igb_vf_reg(core, IGB_VF_VTIVAR_MISC) = 0;

    }
}

static inline uint16_t
igb_vf_get_reg_index_with_offset(hwaddr addr)
{
    uint16_t index = (addr & 0x1ffff) >> 2;
    return index;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// READ Operations
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

static uint32_t
igb_vf_mac_readreg(IgbVfCore *core, int index)
{
    return core->mac[index];
}

static uint32_t
igb_vf_mac_read_as_zero(IgbVfCore *core, int index)
{
    return 0;
}

static uint32_t
igb_vf_mac_read_status(IgbVfCore *core, int index)
{
    IgbPfCore *pf_core = igb_get_pf_core(core);
    uint32_t ret = pf_core->mac[REG2IDX(IGB_STATUS)];
    return ret;
}


#define igb_getreg(x)    [x] = igb_vf_mac_readreg
typedef uint32_t (*readops)(IgbVfCore *, int);
static const readops igb_macreg_readops[] = {
    [REG2IDX(IGB_VF_VTCTRL)]   = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VTSTATUS)]   = igb_vf_mac_read_status,
    [REG2IDX(IGB_VF_VTFRTIMER)] = igb_vf_mac_read_as_zero,


    // Statistics
    [REG2IDX(IGB_VF_VFGPRC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFGORC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFMPRC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFGPTC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFGOTC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFGOTLBC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFGPTLBC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFGORLBC)] = igb_vf_mac_read_as_zero,
    [REG2IDX(IGB_VF_VFGPRLBC)] = igb_vf_mac_read_as_zero,

    // Interrupts
    [REG2IDX(IGB_VF_VTEICS)]   = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTEIMS)]   = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTEIMC)]   = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTEIAC)]   = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTEIAM)]   = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTEICR)]   = igb_vf_mac_eicr_read,

    [REG2IDX(IGB_VF_VTIVAR0)]  = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTIVAR_MISC)]  = igb_vf_mac_readreg,

    [REG2IDX(IGB_VF_VTEITR0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTEITR1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTEITR2)] = igb_vf_mac_readreg,

    // Mailbox
    [REG2IDX(IGB_VF_VTMAILBOX)] = igb_vf_read_mailbox,
    [REG2IDX(IGB_VF_VMBMEM) ... REG2IDX(IGB_VF_VMBMEM) + VMBMEM_WORDS_PER_PF - 1] = igb_vf_read_vmbmem,

    // Transmit
    [REG2IDX(IGB_VF_VTTDBAL0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDBAH0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDLEN0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDH0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTXCTL0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDT0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTXDCTL0)] = igb_vf_mac_readreg,

    [REG2IDX(IGB_VF_VTTDBAL1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDBAH1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDLEN1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDH1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTXCTL1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTDT1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTTXDCTL1)] = igb_vf_mac_readreg,

    // Receive
    [REG2IDX(IGB_VF_VTRDBAL0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDBAH0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDLEN0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTSRRCTL0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDH0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDT0)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRXDCTL0)] = igb_vf_mac_readreg,

    [REG2IDX(IGB_VF_VTRDBAL1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDBAH1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDLEN1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTSRRCTL1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDH1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRDT1)] = igb_vf_mac_readreg,
    [REG2IDX(IGB_VF_VTRXDCTL1)] = igb_vf_mac_readreg,

};
enum { IGB_NREADOPS = ARRAY_SIZE(igb_macreg_readops) };

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// WRITE Operations
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

void
igb_vf_mac_writereg(IgbVfCore *core, int index, uint32_t val)
{
    core->mac[index] = val;
}

#define igb_putreg(x)    [x] = igb_vf_mac_writereg
typedef void (*writeops)(IgbVfCore *, int, uint32_t);
static const writeops igb_macreg_writeops[] = {

    [REG2IDX(IGB_VF_VTCTRL)]   = igb_vf_set_ctrl,

    // Interrupts
    [REG2IDX(IGB_VF_VTEICS)]   = igb_vf_set_eics,
    [REG2IDX(IGB_VF_VTEIMS)]   = igb_vf_set_eims,
    [REG2IDX(IGB_VF_VTEIMC)]   = igb_vf_set_eimc,
    [REG2IDX(IGB_VF_VTEIAC)]   = igb_vf_set_eiac,
    [REG2IDX(IGB_VF_VTEIAM)]   = igb_vf_set_eiam,
    [REG2IDX(IGB_VF_VTEICR)]   = igb_vf_set_eicr,

    [REG2IDX(IGB_VF_VTIVAR0)]  = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTIVAR_MISC)]  = igb_vf_mac_writereg,

    [REG2IDX(IGB_VF_VTMAILBOX)]   = igb_vf_set_mailbox,
    [REG2IDX(IGB_VF_VMBMEM) ... REG2IDX(IGB_VF_VMBMEM) + VMBMEM_WORDS_PER_PF - 1] = igb_vf_set_vmbmem,


    [REG2IDX(IGB_VF_VTEITR0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTEITR1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTEITR2)] = igb_vf_mac_writereg,


    // Transmit Register Descriptions
    [REG2IDX(IGB_VF_VTTDBAL0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDBAH0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDLEN0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDH0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTXCTL0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDT0)] = igb_vf_set_tdt,
    [REG2IDX(IGB_VF_VTTXDCTL0)] = igb_vf_mac_writereg,

    [REG2IDX(IGB_VF_VTTDBAL1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDBAH1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDLEN1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDH1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTXCTL1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTTDT1)] = igb_vf_set_tdt,
    [REG2IDX(IGB_VF_VTTXDCTL1)] = igb_vf_mac_writereg,

    // Receive
    [REG2IDX(IGB_VF_VTRDBAL0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTRDBAH0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTRDLEN0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTSRRCTL0)] = igb_vf_set_srrctl,
    [REG2IDX(IGB_VF_VTRDH0)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTRDT0)] = igb_vf_set_rdt,
    [REG2IDX(IGB_VF_VTRXDCTL0)] = igb_vf_mac_writereg,

    [REG2IDX(IGB_VF_VTRDBAL1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTRDBAH1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTRDLEN1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTSRRCTL1)] = igb_vf_set_srrctl,
    [REG2IDX(IGB_VF_VTRDH1)] = igb_vf_mac_writereg,
    [REG2IDX(IGB_VF_VTRDT1)] = igb_vf_set_rdt,
    [REG2IDX(IGB_VF_VTRXDCTL1)] = igb_vf_mac_writereg,

};
enum { IGB_NWRITEOPS = ARRAY_SIZE(igb_macreg_writeops) };

void
igb_vf_core_write(IgbVfCore *core, hwaddr addr, uint64_t val, unsigned size)
{
    uint16_t index = igb_vf_get_reg_index_with_offset(addr);

    //fprintf(stderr, "VF%u REG[%04lx] <= %08lx {%s}\n", igb_get_vf_num(core), addr, val, igb_vf_reg2name(addr));

    if (index < IGB_NWRITEOPS && igb_macreg_writeops[index]) {
        igb_macreg_writeops[index](core, index, val);
    } else if (index < IGB_NREADOPS && igb_macreg_readops[index]) {
        igb_abort("VF: read-only     register write:  addr=%8lx <= %8lx\n", addr, val);
        trace_igb_wrn_regs_write_ro(index << 2, igb_vf_reg2name(addr), val);
    } else {
        trace_igb_wrn_regs_write_unknown(index << 2, igb_vf_reg2name(addr), val);
        if (getenv("PERMISSIVE_VF") == NULL) {
            igb_abort("VF: unimplemented register write:  addr=%8lx <= %8lx\n", addr, val);
        } else {
            igb_vf_mac_writereg(core, index, val);
        }
    }
}

uint64_t
igb_vf_core_read(IgbVfCore *core, hwaddr addr, unsigned size)
{
    uint64_t val;
    uint16_t index = igb_vf_get_reg_index_with_offset(addr);
    uint32_t ret;


    if (index < IGB_NREADOPS && igb_macreg_readops[index]) {
        val = igb_macreg_readops[index](core, index);
        //trace_igb_vf_core_read(index << 2, igb_vf_reg2name(addr), val);
        ret = val;
    } else {
        trace_igb_wrn_regs_read_unknown(index << 2, igb_vf_reg2name(addr));

        if (getenv("PERMISSIVE_VF") == NULL) {
            igb_abort("VF: unimplemented register read :  addr=%8lx\n", addr);
        } else {
            ret = igb_vf_mac_readreg(core, index);
        }
    }
    //if (addr != IGB_VF_VTMAILBOX)
    //fprintf(stderr, "VF%u REG[%04lx] => %08x {%s}\n", igb_get_vf_num(core), addr, ret, igb_vf_reg2name(addr));
    return ret;
}


void
igb_vf_core_pci_realize(IgbVfCore *core)
{
    // TX:
    for (int i = 0; i < IGB_VF_NUM_QUEUES; i++) {
        net_tx_pkt_init(&core->tx[i].tx_pkt, core->owner,  IGB_MAX_TX_FRAGS,
                        false /*vnet*/);
    }

    // RX:
    net_rx_pkt_init(&core->rx_pkt, false /*vnet*/);
    igb_vf_core_prepare_tx(core);
    igb_vf_core_prepare_rx(core);

	// Crude Interrupt Throttling:
	core->interrupt_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, igb_vf_interrupt_timer, core);
	core->pending_interrupts = 0;
}

void
igb_vf_core_reset(IgbVfCore *core)
{
    memset(core->mac, 0, sizeof core->mac);
    igb_vf_mailbox_realize(core);
}


void
igb_vf_core_pci_uninit(IgbVfCore *core)
{
    for (int i = 0; i < IGB_VF_NUM_QUEUES; i++) {
        net_tx_pkt_reset(core->tx[i].tx_pkt);
        net_tx_pkt_uninit(core->tx[i].tx_pkt);
    }
    net_rx_pkt_uninit(core->rx_pkt);
}

