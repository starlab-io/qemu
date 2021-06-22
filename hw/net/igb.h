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

#ifndef HW_NET_IGB_H
#define HW_NET_IGB_H

struct  IgbPfState;
typedef struct IgbPfState IgbPfState;

struct  IgbVfState;
typedef struct IgbVfState IgbVfState;

struct IgbPfCore;
typedef struct IgbPfCore IgbPfCore;

struct IgbVfCore;
typedef struct IgbVfCore IgbVfCore;



#define TYPE_IGB_PF "igb"
#define IGB_PF(obj)   OBJECT_CHECK(struct IgbPfState, (obj), TYPE_IGB_PF)


IgbPfCore *
igb_get_pf_core(IgbVfCore *core);

IgbVfCore *
igb_get_vf_core(IgbPfCore *core, unsigned vf);

int
igb_get_vf_num(IgbVfCore *core);

unsigned
igb_get_vf_count(IgbPfCore *core);


bool
igb_is_eiame_enabled(IgbPfCore *core);

#define IGB_SOURCE_QEMU  -2
#define IGB_SOURCE_PF    -1

typedef struct {
    IgbPfCore *core;
    int source;
} igb_send_context_t;
void igb_broadcast_pkt(igb_send_context_t *context, const struct iovec *iov,
                       int iovcnt);


#endif
