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

#include "igb_pf_interrupt.h"
#include "igb_common.h"
#include "igb_pf_reg.h"
#include "igb.h"

#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "trace.h"

#define PF_OTHER_CAUSE 31

/*
 * Returns true if device is in MSI-X mode.  (This implementation doesn't handle other modes!)
 */
bool
igb_is_msix_mode(IgbPfCore *core)
{
    return bitfield_extract(core->mac[GPIE], IGB_GPIE_MSIX_MODE);
}

/*
 * Returns the requested IVAR entry
 */
static uint8_t
igb_pf_get_ivar_byte(IgbPfCore *core, unsigned idx)
{
    const unsigned idx_ivar[8] = {IVAR0, IVAR1, IVAR2, IVAR3, IVAR4, IVAR5, IVAR6, IVAR7};
    uint32_t regval;
    if (idx != PF_OTHER_CAUSE) {
        if (idx >= 30) {
            igb_abort("idx=%u\n", idx);
        }
        assert(idx < 30);
        regval = core->mac[idx_ivar[idx / 4]];
    } else {
        regval = core->mac[IVAR_MISC];
        idx = 1;
    }
    uint8_t ret = (regval >> ((idx % 4) * 8)) & 0xff;
    return ret;
}


/*
 * Map RX Queue # to MSIX Interrupt cause
 */
static uint32_t
igb_pf_get_rxq_cause(IgbPfCore *core, unsigned rxq)
{
    if (!igb_is_msix_mode(core)) {
        return 0;
    }
    assert(rxq < IGB_PF_NUM_QUEUES);
    return (rxq < 8) ? rxq * 4 : (rxq - 8) * 4 + 2;
}

/*
 * Map TX Queue # to MSIX Interrupt cause
 */
static uint32_t
igb_pf_get_txq_cause(IgbPfCore *core, unsigned txq)
{
    if (!igb_is_msix_mode(core)) {
        return 0;
    }
    assert(txq < IGB_PF_NUM_QUEUES);
    return (txq < 8) ? txq * 4 + 1 : (txq - 8) * 4 + 3;
}

/*
 * Limit interrupts to 1 group per millisecond.
 * This seems to avoid issues in 'user' mode networking
 * (where 500ms latencies occur without any type of interrupt moderation.
 * The e1000e emulation has the same problem when moderation is disabled.
 *
 * Rather than implement a fully realistic design similar to HW,
 * this takes a shortcut...
 */
void igb_pf_interrupt_timer(void *opaque)
{
    IgbPfCore *core = opaque;

    if (core->pending_interrupts) {

        for(uint32_t intnum=0; intnum<32; ++intnum) {

            if (BIT(intnum) & core->pending_interrupts) {
                trace_igb_pf_interrupt_notify(intnum);
                msix_notify(core->owner, intnum);
                core->pending_interrupts &= ~BIT(intnum);
            }
        }
        timer_mod(core->interrupt_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
    }
}

/*
 * Raise a single interrupt by number
 */
static void
igb_pf_raise_single(IgbPfCore *core, uint32_t intnum)
{
    const uint32_t intvector = BIT(intnum);
    core->mac[EICR] |= intvector;

    if (core->mac[EIMS] & intvector) {
        core->pending_interrupts |= BIT(intnum);
        if (!timer_pending(core->interrupt_timer) ) {
            igb_pf_interrupt_timer(core);
        }

        // handle Extended Interrupt Acknowledge auto-mask
        if (igb_is_eiame_enabled(core)) {
            igb_pf_set_eimc(core, REG2IDX(IGB_EIMC), intvector & core->mac[EIAM]);
        }
    } else {
        trace_igb_pf_interrupt_not_raised(intvector, core->mac[EIMS]);
    }
}

bool
igb_is_eiame_enabled(IgbPfCore *core)
{
    return !!(core->mac[IGB_GPIE] & IGB_GPIE_EIAME);
}


/*
 * Maps an interrupt cause to an interrupt number and raises it
 */
static void
igb_pf_raise_single_cause(IgbPfCore *core, uint32_t cause)
{
    const uint8_t ivar      = igb_pf_get_ivar_byte(core, cause);
    const uint8_t intnum    = ivar & IGB_IVAR_INT_ALLOC;
    const bool    intvalid  = ivar & IGB_IVAR_VALID;

    if (intvalid) {
        trace_igb_pf_interrupt_raise_cause(cause, ivar, intnum, core->mac[EIMS]);
        igb_pf_raise_single(core, intnum);
    } else {
        trace_igb_pf_interrupt_raise_cause_invalid(cause);
    }
}

/*
 * Raises a TX queue interrupt
 */
void
igb_pf_raise_tx_interrupt(IgbPfCore *core, unsigned queue)
{
    core->mac[ICR]  |= IGB_IMS_TXDW;
    uint32_t cause = igb_pf_get_txq_cause(core, queue);
    igb_pf_raise_single_cause(core, cause);
}

/*
 * Raises a RX queue interrupt
 */
void
igb_pf_raise_rx_interrupt(IgbPfCore *core, unsigned queue)
{
    core->mac[ICR]  |= IGB_IMS_RXT0;
    uint32_t cause = igb_pf_get_rxq_cause(core, queue);
    igb_pf_raise_single_cause(core, cause);
}

/*
 * Raises an "other" interrupt and updates ICR
 */
void
igb_pf_raise_other_interrupt(IgbPfCore *core, uint32_t icr)
{
    const uint32_t cause = PF_OTHER_CAUSE;
    core->mac[ICR] |= icr;
    if (core->mac[IMS] & icr) {
        igb_pf_raise_single_cause(core, cause);
    } else {
        trace_igb_pf_interrupt_other_not_raised(cause, icr, core->mac[IMS]);
    }
}

/*
 * Raises a mailbox interrupt
 */
void
igb_pf_raise_mailbox_interrupt(IgbPfCore *core, unsigned vf)
{
    if (BIT(vf) & core->mac[MBVFIMR]) {
        igb_pf_raise_other_interrupt(core, IGB_ICR_VMMB);
    } else {
        trace_igb_pf_mailbox_interrupt_disabled(vf, core->mac[MBVFIMR]);
    }
}


/*
 * Set Extended Interrupt Cause Register (EICR)
 */
void
igb_pf_set_eicr(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[EICR] &= ~val; //w1c
}

/*
 * Set Interrupt Mask Clear (IMC) Register
 */
void
igb_pf_set_imc(IgbPfCore *core, int index, uint32_t val)
{
    trace_igb_irq_clear_ims(val, core->mac[IMS], core->mac[IMS] & ~val);

    core->mac[IMS] &= ~val;
}

/*
 * Set Interrupt Mask Set (IMS) Register
 */
void
igb_pf_set_ims(IgbPfCore *core, int index, uint32_t val)
{
    trace_igb_irq_set_ims(val, core->mac[IMS], core->mac[IMS] | val);

    core->mac[IMS] |= val;
}

/* igb EIMC (Extended Interrupt Mask Clear)
 * Clear corresponding bit in EIMS
 */
void
igb_pf_set_eimc(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[EIMS] &= ~val;
}

/* igb EIAM (Extended Interrupt Auto Mask Enable) */
void
igb_pf_set_eiam(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[EIAM] = val;
}

/* igb EIAC (Extended Interrupt Auto Clear) */
void
igb_pf_set_eiac(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[EIAC] = val;
    //TODO: Handling of this during interrupt generation is not yet implemeted!
}

/* igb EIMS (Extended Interrupt Mask Set)
 * Enables interrupts by writing 1's.
 */
void
igb_pf_set_eims(IgbPfCore *core, int index, uint32_t val)
{
    uint32_t eims = core->mac[EIMS];
    uint32_t eims_new = eims | val;
    uint32_t triggered = eims_new & ~eims & core->mac[EICR];

    core->mac[EIMS] =  eims_new;

    //NOTE: The GPIE.PBA_support bit is ignored since we do not implement legacy interrupts
    if (triggered) {
        /* Loop through set bits and generate interrupt.. */
        for (unsigned intnum = 0; intnum < 32; ++intnum) {
            const uint32_t v = BIT(intnum) & triggered;
            if (!v) {
                continue;
            }
            igb_pf_raise_single(core, intnum);
        }
    }
}

/* igb eics (extended interrupt cause set)
 * Enables interrupts by writing 1's.
 */
void
igb_pf_set_eics(IgbPfCore *core, int index, uint32_t val)
{
    core->mac[EICR] |= val; //add to cause
    //core->mac[EIMS] |= val; //add to mask, manual is unclear when interrupt mask is disabled

    // Raise each bit set:
    for (unsigned intnum = 0; intnum < 32; ++intnum) {
        const uint32_t v = BIT(intnum) & val;
        if (!v) {
            continue;
        }
        igb_pf_raise_single(core, intnum);
    }
}

/*
 * Read Interrupt Cause Register (ICR)
 */
uint32_t
igb_pf_mac_icr_read(IgbPfCore *core, int index)
{
    uint32_t ret = core->mac[ICR];

    if ((core->mac[GPIE] & IGB_GPIE_NSICR) ||
            // NSICR documentation is a bit confusing here:
            (core->mac[ICR] & core->mac[ICS])  ||
            (core->mac[IMS] == 0)) {
        core->mac[ICR] = 0;
    }
    return ret;
}

/*
 * Read Extended Interrupt Cause Register (EICR)
 */
uint32_t
igb_pf_mac_eicr_read(IgbPfCore *core, int index)
{
    uint32_t ret = core->mac[EICR];
    core->mac[EICR] = 0; // read-to-clear
    return ret;
}
