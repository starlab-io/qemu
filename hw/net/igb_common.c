/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation -- shared code
 *
 * Author:  Alex Olson (2021)

 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */


#include "qemu/osdep.h"
#include "igb_common.h"

#include <execinfo.h>

unsigned
igb_lookup_value(const int value,
                 const int *array,
                 const unsigned array_size)
{
    for (unsigned k = 0; k < array_size; ++k) {
        if (array[k] == value) {
            return k;
        }
    }
    igb_abort("illegal array value: %x\n", value);
}


void igb_backtrace(void)
{
    void *buffer[20];
    int ret = backtrace(buffer, ARRAY_SIZE(buffer));
    fprintf(stderr, "backtrace: ret=%d\n", ret);
    backtrace_symbols_fd(buffer, ret, 2);
}
