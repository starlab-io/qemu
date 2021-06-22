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


#include "igb_mailbox.h"
#include "igb_common.h"
#include "igb_pf_reg.h"
#include "igb_pf_interrupt.h"

#include "igb_pf_core.h"
#include "igb_vf_core.h"
#include "igb_vf_reg.h"
#include "igb_vf_interrupt.h"

#include "qemu/bitops.h"
#include "trace.h"

#define IGB_MBVFICR_VFREQ(vf) BIT( 0 + vf) // request
#define IGB_MBVFICR_VFACK(vf) BIT(16 + vf) // ack
#define IGB_MBVFIMR_VFIM(vf)  BIT(0 + vf)  // interrupt mask

static uint32_t * ptr_pf_mailbox(IgbVfCore *core)
{
    const int vf = igb_get_vf_num(core);
    IgbPfCore *pf_core = igb_get_pf_core(core);
    return &pf_core->mac[REG2IDX(IGB_PFMAILBOX(vf))];
}

static uint32_t * ptr_vf_mailbox(IgbVfCore *core)
{
    return &core->mac[REG2IDX(IGB_VF_VTMAILBOX)];
}

static uint32_t * ptr_pf_vmbem(IgbPfCore *core)
{
    return &core->mac[REG2IDX(IGB_VMBMEM(0))];
}

static uint32_t * ptr_vf_vmbem(IgbVfCore *core)
{
    const int vf = igb_get_vf_num(core);
    IgbPfCore *pf_core = igb_get_pf_core(core);
    return &pf_core->mac[REG2IDX(IGB_VMBMEM(vf))];
}

void
igb_pf_set_vmbmem(IgbPfCore *core, int index, uint32_t val)
{
    uint32_t *vmem = ptr_pf_vmbem(core);
    unsigned off = index - REG2IDX(IGB_VMBMEM(0));

    assert(off < VMBMEM_WORDS_PER_PF * 8);
    vmem[off] = val;
}


void
igb_vf_set_vmbmem(IgbVfCore *core, int index, uint32_t val)
{
    uint32_t *vmem = ptr_vf_vmbem(core);
    unsigned off = index - REG2IDX(IGB_VF_VMBMEM);
    assert(off < VMBMEM_WORDS_PER_PF);
    vmem[off] = val;
}


uint32_t
igb_pf_read_vmbmem(IgbPfCore *core, int index)
{
    uint32_t *vmem = ptr_pf_vmbem(core);
    unsigned off = index - REG2IDX(IGB_VMBMEM(0));

    assert(off < VMBMEM_WORDS_PER_PF * 8);
    return vmem[off];
}


uint32_t
igb_vf_read_vmbmem(IgbVfCore *core, int index)
{
    uint32_t *vmem = ptr_vf_vmbem(core);
    unsigned off = index - REG2IDX(IGB_VF_VMBMEM);
    assert(off < VMBMEM_WORDS_PER_PF);
    return vmem[off];
}

void
igb_pf_set_mailbox(IgbPfCore *core, int index, uint32_t val)
{
    const unsigned vf = (index - REG2IDX(IGB_PFMAILBOX(0)));
    assert(vf < 8);
    IgbVfCore *vf_core = igb_get_vf_core(core, vf);
    uint32_t * const pfmailbox = &core->mac[index];
    uint32_t * const vfmailbox = ptr_vf_mailbox(vf_core);

    trace_igb_mailbox_pf_write(vf, val);

    if (val & IGB_PFMAILBOX_STS) {
        *vfmailbox |= IGB_VFMAILBOX_PFSTS;
        igb_vf_raise_mailbox_interrupt(vf_core);
    }

    if (val & IGB_PFMAILBOX_ACK) {
        *vfmailbox |= IGB_VFMAILBOX_PFACK;
        igb_vf_raise_mailbox_interrupt(vf_core);
    }

    if ((val & IGB_PFMAILBOX_PFU) && !(*vfmailbox & IGB_VFMAILBOX_VFU)) {
        *vfmailbox |= IGB_VFMAILBOX_PFU;
        *pfmailbox |= IGB_PFMAILBOX_PFU;
    } else {
        *vfmailbox &= ~IGB_VFMAILBOX_PFU;
        *pfmailbox &= ~IGB_PFMAILBOX_PFU;
    }

    if (val & IGB_PFMAILBOX_RVFU) {
        *vfmailbox &= ~IGB_VFMAILBOX_VFU;
        *pfmailbox &= ~IGB_PFMAILBOX_VFU;
        core->mac[REG2IDX(IGB_MBVFICR)] &= ~IGB_MBVFICR_VFREQ(vf);
        core->mac[REG2IDX(IGB_MBVFICR)] &= ~IGB_MBVFICR_VFACK(vf);
    }
}

void
igb_vf_set_mailbox(IgbVfCore *core, int index, uint32_t val)
{
    const int vf = igb_get_vf_num(core);
    IgbPfCore *pf_core = igb_get_pf_core(core);
    uint32_t * const vfmailbox = ptr_vf_mailbox(core);
    uint32_t * const pfmailbox = ptr_pf_mailbox(core);

    trace_igb_mailbox_vf_write(vf, val);

    if (val & IGB_VFMAILBOX_REQ) {
        pf_core->mac[REG2IDX(IGB_MBVFICR)] |= IGB_MBVFICR_VFREQ(vf);
        igb_pf_raise_mailbox_interrupt(pf_core, vf);
    }

    if (val & IGB_VFMAILBOX_ACK) {
        pf_core->mac[REG2IDX(IGB_MBVFICR)] |= IGB_MBVFICR_VFACK(vf);
        igb_pf_raise_mailbox_interrupt(pf_core, vf);
    }

    if ((val & IGB_VFMAILBOX_VFU) && !(*pfmailbox & IGB_PFMAILBOX_PFU)) {
        *vfmailbox |= IGB_VFMAILBOX_VFU;
        *pfmailbox |= IGB_PFMAILBOX_VFU;
    } else {
        *vfmailbox &= ~IGB_VFMAILBOX_VFU;
        *pfmailbox &= ~IGB_PFMAILBOX_VFU;
    }
}

uint32_t
igb_pf_read_mailbox(IgbPfCore *core, int index)
{
    const unsigned vf = index - REG2IDX(IGB_PFMAILBOX(0));
    assert(vf < 8);
    IgbVfCore *vf_core = igb_get_vf_core(core, vf);
    uint32_t * const pfmailbox = ptr_pf_mailbox(vf_core);

    trace_igb_mailbox_pf_read(vf, *pfmailbox);
    return *pfmailbox;
}

uint32_t
igb_vf_read_mailbox(IgbVfCore *core, int index)
{
    uint32_t * const vfmailbox = ptr_vf_mailbox(core);
    uint32_t ret = *vfmailbox;

    *vfmailbox &= ~(IGB_VFMAILBOX_PFSTS | IGB_VFMAILBOX_PFACK | IGB_VFMAILBOX_RSTD);

    trace_igb_mailbox_vf_read(igb_get_vf_num(core), ret);
    return ret;
}


void
igb_vf_mailbox_realize(IgbVfCore *core)
{
    uint32_t * const vfmailbox = ptr_vf_mailbox(core);
    uint32_t * const pfmailbox = ptr_pf_mailbox(core);
    *pfmailbox  = 0;
    *vfmailbox = IGB_VFMAILBOX_RSTI;
}

static void
igb_vf_mailbox_reset_done(IgbVfCore *core)
{
    uint32_t * const vfmailbox = ptr_vf_mailbox(core);


    *vfmailbox |= IGB_VFMAILBOX_RSTD;
    *vfmailbox &= ~IGB_VFMAILBOX_RSTI;

    igb_vf_raise_mailbox_interrupt(core);
}

void
igb_pf_mailbox_reset_done(IgbPfCore *core)
{
    for (unsigned vf = 0; vf < igb_get_vf_count(core); ++vf) {
        IgbVfCore *vf_core = igb_get_vf_core(core, vf);
        igb_vf_mailbox_reset_done(vf_core);
        igb_vf_raise_mailbox_interrupt(vf_core);
    }
}
