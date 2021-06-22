/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation
 *
 * Copyright (c) 2021
 * Copyright(c) 2009 - 2018 Intel Corporation
 *
 * Author:
 *   Alex Olson, Star Lab
 *
 * Adopted from Linux kernel definitions
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */


#ifndef HW_NET_IGB_VF_REGS_H
#define HW_NET_IGB_VF_REGS_H

#include "igb_shared_reg_constants.h"

#define IGB_VF_VTCTRL       0x00000 /* Device Control - RW */
#define IGB_VF_VTSTATUS     0x00008 /* Device Status - RO */

#define IGB_VF_VTCTRL_RST BIT(26)

/* Statistics registers */
#define IGB_VF_VFGPRC	0x00F10
#define IGB_VF_VFGORC	0x00F18
#define IGB_VF_VFMPRC	0x00F3C
#define IGB_VF_VFGPTC	0x00F14
#define IGB_VF_VFGOTC	0x00F34
#define IGB_VF_VFGOTLBC	0x00F50
#define IGB_VF_VFGPTLBC	0x00F44
#define IGB_VF_VFGORLBC	0x00F48
#define IGB_VF_VFGPRLBC	0x00F40




#define IGB_VF_VTFRTIMER    0x01048 /* Free - RO */
#define IGB_VF_VTEICS       0x01520 /* Ext. Interrupt Cause Set - W0 */
#define IGB_VF_VTEIMS       0x01524 /* Ext. Interrupt Mask Set/Read - RW */
#define IGB_VF_VTEIMC       0x01528 /* Ext. Interrupt Mask Clear - WO */
#define IGB_VF_VTEIAC       0x0152C /* Ext. Interrupt Auto Clear - RW */
#define IGB_VF_VTEIAM       0x01530 /* Ext. Interrupt Ack Auto Clear Mask - RW */
#define IGB_VF_VTEICR       0x01580 /* Ext. Interrupt Cause Read - R/clr */

#define IGB_VF_VTEITR0      0x01680 /* Interrupt Throttle Registers 0-2 */
#define IGB_VF_VTEITR1      0x01684 /* Interrupt Throttle Registers 0-2 */
#define IGB_VF_VTEITR2      0x01688 /* Interrupt Throttle Registers 0-2 */

#define IGB_VF_VTIVAR0      0x01700 /* Interrupt Vector Allocation (array) - RW */
#define IGB_VF_VTIVAR_MISC  0x01740 /* IVAR for "other" causes - RW */
#define IGB_VF_VTPBACL      0x00f04 /* PBA Clear */
#define IGB_VF_VTPSRTYPE    0x00f0c
#define IGB_VF_VTMAILBOX    0x00c40
#define IGB_VF_VMBMEM       0x00800
#define IGB_VMBEM_BYTES 64

#define IGB_VF_VTRDBAL0     0x02800
#define IGB_VF_VTRDBAH0     0x02804
#define IGB_VF_VTRDLEN0     0x02808
#define IGB_VF_VTSRRCTL0    0x0280C
#define IGB_VF_VTRDH0       0x02810
#define IGB_VF_VTRDT0       0x02818
#define IGB_VF_VTRXDCTL0    0x02828
#define IGB_VF_VTTDBAL0     0x03800
#define IGB_VF_VTTDBAH0     0x03804
#define IGB_VF_VTTDLEN0     0x03808
#define IGB_VF_VTTDH0       0x03810
#define IGB_VF_VTTXCTL0     0x03814
#define IGB_VF_VTTDT0       0x03818
#define IGB_VF_VTTXDCTL0    0x03828

#define IGB_VF_VTRDBAL1     0x02900
#define IGB_VF_VTRDBAH1     0x02904
#define IGB_VF_VTRDLEN1     0x02908
#define IGB_VF_VTSRRCTL1    0x0290C
#define IGB_VF_VTRDH1       0x02910
#define IGB_VF_VTRDT1       0x02918
#define IGB_VF_VTRXDCTL1    0x02928
#define IGB_VF_VTTDBAL1     0x03900
#define IGB_VF_VTTDBAH1     0x03904
#define IGB_VF_VTTDLEN1     0x03908
#define IGB_VF_VTTDH1       0x03910
#define IGB_VF_VTTXCTL1     0x03914
#define IGB_VF_VTTDT1       0x03918
#define IGB_VF_VTTXDCTL1    0x03928


#endif
