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

#include "igb_pf_phy.h"
#include "igb_common.h"
#include "igb_pf_reg.h"
#include "igb_pf_interrupt.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "trace.h"

enum { PHY_R = BIT(0),
       PHY_W = BIT(1),
       PHY_RW = PHY_R | PHY_W,
       PHY_ANYPAGE = BIT(2)
     };


static inline void
igb_pf_set_phy_ctrl(IgbPfCore *core, int index, uint16_t val)
{
    /* bits 0-5 reserved; MII_CR_[RESTART_AUTO_NEG,RESET] are self clearing */
    core->phy[0][PHY_CTRL] = val & ~(0x3f |
                                     MII_CR_RESET |
                                     MII_CR_RESTART_AUTO_NEG);

}

static void
igb_pf_set_phy_page(IgbPfCore *core, int index, uint16_t val)
{
    core->phy[0][PHY_PAGE] = val & PHY_PAGE_RW_MASK;
}


static
void(*igb_phyreg_writeops[IGB_PHY_PAGES][IGB_PHY_PAGE_SIZE])
(IgbPfCore *, int, uint16_t) = {
    [0] = {
        [PHY_CTRL]     = igb_pf_set_phy_ctrl,
        [PHY_PAGE]     = igb_pf_set_phy_page,
    }
};


static const char igb_phy_regcap[IGB_PHY_PAGES][0x20] = {
    [0] = {
        [PHY_CTRL]          = PHY_ANYPAGE | PHY_RW,
        [PHY_STATUS]        = PHY_ANYPAGE | PHY_R,
        [PHY_ID1]           = PHY_ANYPAGE | PHY_R,
        [PHY_ID2]           = PHY_ANYPAGE | PHY_R,
        [PHY_AUTONEG_ADV]   = PHY_ANYPAGE | PHY_RW,
        [PHY_LP_ABILITY]    = PHY_ANYPAGE | PHY_R,
        [PHY_AUTONEG_EXP]   = PHY_ANYPAGE | PHY_R,
        [PHY_NEXT_PAGE_TX]  = PHY_ANYPAGE | PHY_RW,
        [PHY_LP_NEXT_PAGE]  = PHY_ANYPAGE | PHY_R,
        [PHY_1000T_CTRL]    = PHY_ANYPAGE | PHY_RW,
        [PHY_1000T_STATUS]  = PHY_ANYPAGE | PHY_R,
        [PHY_EXT_STATUS]    = PHY_ANYPAGE | PHY_R,
        [PHY_PAGE]          = PHY_ANYPAGE | PHY_RW,

        [PHY_COPPER_CTRL1]      = PHY_RW,
        [PHY_COPPER_STAT1]      = PHY_R,
        [PHY_COPPER_CTRL3]      = PHY_RW,
        [PHY_RX_ERR_CNTR]       = PHY_R,
        [PHY_OEM_BITS]          = PHY_RW,
        [PHY_BIAS_1]            = PHY_RW,
        [PHY_BIAS_2]            = PHY_RW,
        [PHY_PAGE_SELECT]       = PHY_RW,
        [PHY_COPPER_INT_ENABLE] = PHY_RW,
        [PHY_COPPER_STAT2]      = PHY_R,
        [PHY_COPPER_CTRL2]      = PHY_RW
    },
};

static bool
igb_phy_reg_check_cap(IgbPfCore *core, uint32_t addr,
                      char cap, uint8_t *page)
{
    *page = (igb_phy_regcap[0][addr] & PHY_ANYPAGE) ? 0 : core->phy[0][PHY_PAGE];

    if (*page >= IGB_PHY_PAGES) {
        return false;
    }

    return igb_phy_regcap[*page][addr] & cap;
}

static void
igb_phy_reg_write(IgbPfCore *core, uint8_t page,
                  uint32_t addr, uint16_t data)
{
    assert(page < IGB_PHY_PAGES);
    assert(addr < IGB_PHY_PAGE_SIZE);

    if (igb_phyreg_writeops[page][addr]) {
        igb_phyreg_writeops[page][addr](core, addr, data);
    } else {
        core->phy[page][addr] = data;
    }
}

void
igb_pf_set_mdic(IgbPfCore *core, int index, uint32_t val)
{
    uint32_t data = val & IGB_MDIC_DATA_MASK;
    uint32_t addr = bitfield_extract(val, IGB_MDIC_REG_MASK);
    uint8_t page;

    if (bitfield_extract(val, IGB_MDIC_PHY_MASK) != 1) { /* phy # */
        val = core->mac[MDIC] | IGB_MDIC_ERROR;
    } else if (val & IGB_MDIC_OP_READ) {
        if (!igb_phy_reg_check_cap(core, addr, PHY_R, &page)) {
            trace_igb_pf_core_mdic_read_unhandled(page, addr);
            val |= IGB_MDIC_ERROR;
        } else {
            val = (val ^ data) | core->phy[page][addr];
            trace_igb_pf_core_mdic_read(page, addr, val);
        }
    } else if (val & IGB_MDIC_OP_WRITE) {
        if (!igb_phy_reg_check_cap(core, addr, PHY_W, &page)) {
            trace_igb_pf_core_mdic_write_unhandled(page, addr);
            val |= IGB_MDIC_ERROR;
        } else {
            trace_igb_pf_core_mdic_write(page, addr, data);
            igb_phy_reg_write(core, page, addr, data);
        }
    }
    core->mac[MDIC] = val | IGB_MDIC_READY;
    /*
        if (val & IGB_MDIC_INT_EN) {
            igb_pf_set_interrupt_cause(core, IGB_ICR_MDAC);
        }
    */
}

void
igb_pf_core_set_link_status(IgbPfCore *core, bool link_down)
{
    uint32_t old_status = core->mac[STATUS];

    if (link_down) {
        core->mac[STATUS] &= ~IGB_STATUS_LU;
        core->phy[0][PHY_STATUS] &= ~MII_SR_LINK_STATUS;
    } else {
        core->mac[STATUS] |=  IGB_STATUS_LU;
        core->phy[0][PHY_STATUS] |= MII_SR_LINK_STATUS;
    }

    if (old_status != core->mac[STATUS]) {
        igb_pf_raise_other_interrupt(core, IGB_ICR_LSC);
    }
}




void
igb_pf_core_phy_reset(IgbPfCore *core)
{
    static const uint16_t
    init [IGB_PHY_PAGES][IGB_PHY_PAGE_SIZE] = {
        [0] = {
            [PHY_CTRL] =   MII_CR_SPEED_SELECT_MSB  |
            MII_CR_FULL_DUPLEX |
            MII_CR_AUTO_NEG_EN,

            [PHY_STATUS] = MII_SR_EXTENDED_CAPS     |
            MII_SR_LINK_STATUS       |
            MII_SR_AUTONEG_CAPS      |
            MII_SR_PREAMBLE_SUPPRESS |
            MII_SR_EXTENDED_STATUS,

            [PHY_ID1]               = (I350_I_PHY_ID >> 16) & 0xffff,
            [PHY_ID2]               = (I350_I_PHY_ID >>  0) & 0xffff,
            [PHY_AUTONEG_ADV]       = 0,
            [PHY_LP_ABILITY]        = 0,
            [PHY_AUTONEG_EXP]       = BIT(2),
            [PHY_NEXT_PAGE_TX]      = BIT(0) | BIT(13),
            [PHY_1000T_CTRL]        = 0,
            [PHY_1000T_STATUS]      = BIT(13) | BIT(12) | BIT(11),
            [PHY_EXT_STATUS]        = BIT(12) | BIT(13),
            [PHY_COPPER_CTRL1]      = BIT(5) | BIT(6) | BIT(8) | BIT(9) | BIT(12) | BIT(13),
            [PHY_COPPER_STAT1]      = BIT(3) | BIT(10) | BIT(11) | BIT(13) | BIT(15)
        },
    };

    assert(sizeof init <= sizeof core->phy);
    memset(core->phy, 0, sizeof core->phy);
    memmove(core->phy, init, sizeof init);
}
