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

#include "igb_vf_interrupt.h"
#include "igb_common.h"
#include "igb_vf_reg.h"
#include "igb.h"

#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "trace.h"

#define VF_INT_MASK 0x7  // 3 LSBs

#define VF_MAILBOX_CAUSE 4  // INT_Alloc[4]

/*
 * Returns the requested IVAR entry
 */
static uint8_t
igb_vf_get_ivar_byte(IgbVfCore *core, unsigned idx)
{
    const uint32_t addr_ivar[2] = {IGB_VF_VTIVAR0, IGB_VF_VTIVAR_MISC};
    assert(idx <= 4);
    const uint32_t regval = *igb_vf_reg(core, addr_ivar[idx / 4]);

    uint8_t ret = (regval >> ((idx % 4) * 8)) & 0xff;
    return ret;
}


/*
 * Map RX Queue # to MSIX Interrupt cause
 */
static uint32_t
igb_vf_get_rxq_cause(IgbVfCore *core, unsigned rxq)
{
    assert(rxq < 2);
    return rxq * 2; // different than PF
}

/*
 * Map TX Queue # to MSIX Interrupt cause
 */
static uint32_t
igb_vf_get_txq_cause(IgbVfCore *core, unsigned txq)
{
    assert(txq < 2);
    return txq * 2 + 1; // different than PF
}

/*
 * Raise a single interrupt by number
 */
static void
igb_vf_raise_single(IgbVfCore *core, uint32_t intnum)
{
    const uint32_t intvector = BIT(intnum);
    uint32_t *eicr = igb_vf_reg(core, IGB_VF_VTEICR);
    const uint32_t *eims = igb_vf_reg(core, IGB_VF_VTEIMS);

    *eicr |= intvector;

    if (*eims & intvector) {
        trace_igb_vf_interrupt_notify(igb_get_vf_num(core), intnum);
        msix_notify(core->owner, intnum);

        // handle Extended Interrupt Acknowledge auto-mask
        if (igb_is_eiame_enabled(igb_get_pf_core(core))) {
            const uint32_t *eiam = igb_vf_reg(core, IGB_VF_VTEIAM);
            igb_vf_set_eimc(core, IGB_VF_VTEIMC, intvector & *eiam);
        }
    } else {
        trace_igb_vf_interrupt_not_raised(igb_get_vf_num(core), intvector, *eims);
    }
}

static void
igb_vf_raise_single_cause(IgbVfCore *core, uint32_t cause)
{
    const uint8_t ivar 		= igb_vf_get_ivar_byte(core, cause);
    const uint8_t intnum	= ivar & IGB_IVAR_INT_ALLOC;
    const bool    intvalid  = ivar & IGB_IVAR_VALID;

    if (intvalid) {
        trace_igb_vf_interrupt_raise_cause(igb_get_vf_num(core), cause, ivar, intnum,
                                           *igb_vf_reg(core, IGB_VF_VTEIMS));
        igb_vf_raise_single(core, intnum);
    } else {
        trace_igb_vf_interrupt_raise_cause_invalid(igb_get_vf_num(core), cause);
    }
}

/*
 * Raises a TX queue interrupt
 */
void
igb_vf_raise_tx_interrupt(IgbVfCore *core, unsigned queue)
{
    uint32_t cause = igb_vf_get_txq_cause(core, queue);
    igb_vf_raise_single_cause(core, cause);
}

/*
 * Raises a RX queue interrupt
 */
void
igb_vf_raise_rx_interrupt(IgbVfCore *core, unsigned queue)
{
    uint32_t cause = igb_vf_get_rxq_cause(core, queue);
    igb_vf_raise_single_cause(core, cause);
}

/*
 * Raises a mailbox interrupt
 */
void
igb_vf_raise_mailbox_interrupt(IgbVfCore *core)
{
    igb_vf_raise_single_cause(core, VF_MAILBOX_CAUSE);
}

/*
 * Set Extended Interrupt Cause Register (EICR)
 */
void
igb_vf_set_eicr(IgbVfCore *core, int index, uint32_t val)
{
    val &= VF_INT_MASK;
    uint32_t *eicr = igb_vf_reg(core, IGB_VF_VTEICR);
    *eicr &= ~val; //w1c
}

/* igb EIMC (Extended Interrupt Mask Clear)
 * Clear corresponding bit in EIMS
 */
void
igb_vf_set_eimc(IgbVfCore *core, int index, uint32_t val)
{
    val &= VF_INT_MASK;
    uint32_t *eims = igb_vf_reg(core, IGB_VF_VTEIMS);
    *eims &= ~val;
}

/* igb EIAM (Extended Interrupt Auto Mask Enable, spec p.352, 504) */
void
igb_vf_set_eiam(IgbVfCore *core, int index, uint32_t val)
{
    val &= VF_INT_MASK;
    uint32_t *eiam = igb_vf_reg(core, IGB_VF_VTEIAM);
    *eiam = val;
}

/* igb EIAC (Extended Interrupt Auto Clear) */
void
igb_vf_set_eiac(IgbVfCore *core, int index, uint32_t val)
{
    val &= VF_INT_MASK;
    uint32_t *eiac = igb_vf_reg(core, IGB_VF_VTEIAC);
    *eiac = val;
}

/* igb EIMS (Extended Interrupt Mask Set, spec p.352, 503)
 * Enables interrupts by writing 1's.
 */
void
igb_vf_set_eims(IgbVfCore *core, int index, uint32_t val)
{
    val &= VF_INT_MASK;
    const uint32_t *eicr = igb_vf_reg(core, IGB_VF_VTEICR);
    uint32_t *eims = igb_vf_reg(core, IGB_VF_VTEIMS);
    uint32_t eims_new = *eims | val;
    uint32_t triggered = eims_new & ~*eims & *eicr;

    *eims = eims_new;

    if (triggered) {
        for (unsigned intnum = 0; intnum < 32; ++intnum) {
            const uint32_t v = BIT(intnum) & triggered;
            if (!v) {
                continue;
            }
            igb_vf_raise_single(core, intnum);
        }
    }
}

/* igb EICS (Extended Interrupt Cause Set)
 * Enables interrupts by writing 1's.
 */
void
igb_vf_set_eics(IgbVfCore *core, int index, uint32_t val)
{
    val &= VF_INT_MASK;
    *igb_vf_reg(core, IGB_VF_VTEICR) |= val; //add to cause
    //*igb_vf_reg(core, IGB_VF_VTEIMS) |= val; //add to mask, manual is unclear when interrupt mask is disabled

    // Raise each bit set:
    for (unsigned intnum = 0; intnum < 32; ++intnum) {
        const uint32_t v = BIT(intnum) & val;
        if (!v) {
            continue;
        }
        igb_vf_raise_single(core, intnum);
    }
}

/*
 * Read Extended Interrupt Cause Register (EICR)
 */
uint32_t
igb_vf_mac_eicr_read(IgbVfCore *core, int index)
{
    uint32_t *eicr = igb_vf_reg(core, IGB_VF_VTEICR);
    uint32_t ret = *eicr;
    *eicr = 0; // read-to-clear
    return ret;
}

