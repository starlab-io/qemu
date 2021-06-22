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

#ifndef HW_NET_IGB_SHARED_REG_CONSTANTS_H
#define HW_NET_IGB_SHARED_REG_CONSTANTS_H

#define IGB_IVAR_VALID		0x80
#define IGB_IVAR_INT_ALLOC	0x1F


#define IGB_PFMAILBOX_STS  BIT(0)
#define IGB_PFMAILBOX_ACK  BIT(1)
#define IGB_PFMAILBOX_VFU  BIT(2)
#define IGB_PFMAILBOX_PFU  BIT(3)
#define IGB_PFMAILBOX_RVFU BIT(4)

#define IGB_VFMAILBOX_REQ   BIT(0)
#define IGB_VFMAILBOX_ACK   BIT(1)
#define IGB_VFMAILBOX_VFU   BIT(2)
#define IGB_VFMAILBOX_PFU   BIT(3)
#define IGB_VFMAILBOX_PFSTS BIT(4)
#define IGB_VFMAILBOX_PFACK BIT(5)
#define IGB_VFMAILBOX_RSTI  BIT(6)
#define IGB_VFMAILBOX_RSTD  BIT(7)

/* Transmit Descriptor Control fields */
#define IGB_TXDCTL_ENABLE		BIT(25)


#define IGB_SRRCTL_BSIZEPACKET   0x0000007f
#define IGB_SRRCTL_BSIZEPACKET_UNIT   1024
#define IGB_SRRCTL_BSIZEHEADER   0x00000f00
#define IGB_SRRCTL_BSIZEHEADER_UNIT   64

#define IGB_SRRCTL_DESCTYPE_MASK              (BIT(27) | BIT(26) | BIT(25))
#define IGB_SRRCTL_DESCTYPE_ADV_ONEBUF                0x02000000
#define IGB_SRRCTL_DESCTYPE_HDR_SPLIT_ALWAYS          0x0A000000


#define IGB_ICR_RXO           0x00000040 /* rx overrun */

#endif
