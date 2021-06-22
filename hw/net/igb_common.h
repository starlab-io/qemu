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

#ifndef HW_NET_IGBX_COMMON_H
#define HW_NET_IGBX_COMMON_H

#include "qemu/osdep.h"

void igb_backtrace(void);

#define igb_abort(fmt, ...) do { fprintf(stderr, fmt, ##__VA_ARGS__); igb_backtrace(); abort(); }while(0)


#define REG2IDX(reg) ((reg)>>2)

#define assign_8(x, func) \
	[x##0]  = func, \
	[x##1]  = func, \
	[x##2]  = func, \
	[x##3]  = func, \
	[x##4]  = func, \
	[x##5]  = func, \
	[x##6]  = func, \
	[x##7]  = func


#define assign_9(x, func) \
	assign_8(x, func),    \
	[x##8]  = func


#define assign_10(x, func) \
	assign_9(x, func),    \
	[x##9]  = func

#define assign_16(x, func) \
	assign_10(x, func), \
	[x##10] = func, \
	[x##11] = func, \
	[x##12] = func, \
	[x##13] = func, \
	[x##14] = func, \
	[x##15] = func

#define assign_24(x, func) \
    assign_16(x, func), \
	[x##16] = func, \
	[x##17] = func, \
	[x##18] = func, \
	[x##19] = func, \
	[x##20] = func, \
	[x##21] = func, \
	[x##22] = func, \
	[x##23] = func


#define assign_25(x, func) \
    assign_24(x, func), \
	[x##24] = func

#define assign_32(x, func) \
    assign_25(x, func),  \
	[x##25] = func, \
	[x##26] = func, \
	[x##27] = func, \
	[x##28] = func, \
	[x##29] = func, \
	[x##30] = func, \
	[x##31] = func



#define defreg(x)       x       = (IGB_##x    >> 2)

#define defreg_8(x) \
    x##0  = (IGB_##x(0 ) >> 2), \
    x##1  = (IGB_##x(1 ) >> 2), \
    x##2  = (IGB_##x(2 ) >> 2), \
    x##3  = (IGB_##x(3 ) >> 2), \
    x##4  = (IGB_##x(4 ) >> 2), \
    x##5  = (IGB_##x(5 ) >> 2), \
    x##6  = (IGB_##x(6 ) >> 2), \
    x##7  = (IGB_##x(7 ) >> 2)

#define defreg_10(x) \
    defreg_8(x), \
    x##8  = (IGB_##x(8 ) >> 2), \
    x##9  = (IGB_##x(9 ) >> 2)

#define defreg_16(x) \
    defreg_10(x), \
    x##10 = (IGB_##x(10) >> 2), \
    x##11 = (IGB_##x(11) >> 2), \
    x##12 = (IGB_##x(12) >> 2), \
    x##13 = (IGB_##x(13) >> 2), \
    x##14 = (IGB_##x(14) >> 2), \
    x##15 = (IGB_##x(15) >> 2)

#define defreg_24(x) \
    defreg_16(x), \
    x##16 = (IGB_##x(16) >> 2), \
    x##17 = (IGB_##x(17) >> 2), \
    x##18 = (IGB_##x(18) >> 2), \
    x##19 = (IGB_##x(19) >> 2), \
    x##20 = (IGB_##x(20) >> 2), \
    x##21 = (IGB_##x(21) >> 2), \
    x##22 = (IGB_##x(22) >> 2), \
    x##23 = (IGB_##x(23) >> 2)

#define defreg_25(x) \
    defreg_24(x), \
    x##24 = (IGB_##x(24) >> 2)

#define defreg_32(x) \
    defreg_25(x), \
    x##25 = (IGB_##x(25) >> 2), \
    x##26 = (IGB_##x(26) >> 2), \
    x##27 = (IGB_##x(27) >> 2), \
    x##28 = (IGB_##x(28) >> 2), \
    x##29 = (IGB_##x(29) >> 2), \
    x##30 = (IGB_##x(30) >> 2), \
    x##31 = (IGB_##x(31) >> 2)


unsigned
igb_lookup_value(const int value,
                 const int *array,
                 const unsigned array_size);
#define IGB_LOOKUP_VALUE(value, array) igb_lookup_value(value, array, ARRAY_SIZE(array))



// All IGB TX descriptors are 16 bytes (two 64-bit words in the documentation)
struct igb_tx_desc {
    uint32_t word0;
    uint32_t word1;
    uint32_t word2_type;  // DTYP is here
    uint32_t word3;
};

/* TX Context descriptor (HW) */
struct igb_adv_tx_context_desc {
    uint32_t vlan_macip_lens;
    uint32_t ipsec_rsvd;

    uint32_t type_tucmd_mlhl;
    uint32_t mss_l4len_idx;
};

/*
#define IGB_ADVTXD_IPLEN_MASK     0x000001FF  // vlan_macip_lens : IPLEN
#define IGB_ADVTXD_MACLEN_MASK    0x0000fe00  // vlan_macip_lens : MACLEN
#define IGB_ADVTXD_TUCMD_MASK     0x000FFE00  //
#define IGB_ADVTXD_TUCMD_L4T_MASK 0x00001800  // type_tucmd_mlhl: L4 Packet TYPE
#define IGB_ADVTXD_TUCMD_IPV4     0x00000800  // type_tucmd_mlhl: IP Packet Type: 1=IPv4
#define IGB_ADVTXD_TUCMD_L4T_UDP  0x00000000  // type_tucmd_mlhl: L4 Packet TYPE of UDP
#define IGB_ADVTXD_TUCMD_L4T_TCP  0x00000800  // type_tucmd_mlhl: L4 Packet TYPE of TCP
#define IGB_ADVTXD_TUCMD_L4T_SCTP 0x00001000  // type_tucmd_mlhl: L4 packet TYPE of SCTP
#define IGB_ADVTXD_MSSL4LEN_IDX   (BIT(38-32) | BIT(37-32) | BIT(36-32))
#define IGB_ADVTXD_MSSL4LEN_L4LEN (0xff << (40-32))
*/
#define IGB_ADVTXD_MSSL4LEN_MSS   (0xffff << (48-32))
#define IGB_ADVTXD_VLAN_MASK     0xffff0000

/* Offload data descriptor (HW) */
struct igb_adv_tx_data_desc {
    uint64_t buffer_addr;       /* Address of the descriptor's buffer address */
    union {
        uint32_t data;
        struct {
            uint16_t length;    /* Data buffer length */
            uint8_t typ_len_ext;        /* */
            uint8_t cmd;        /* */
        } flags;
    } lower;
    uint32_t upper;
};



/* Receive Descriptor - Advanced */
union igb_adv_rx_desc {
    struct {
        uint64_t pkt_addr;             /* Packet buffer address */
        uint64_t hdr_addr;             /* Header buffer address */
    } read;
    struct {
        struct {
            struct {
                uint16_t pkt_info;   /* RSS type, Packet type */
                uint16_t hdr_info;   /* Split Head, buf len */
            } lo_dword;
            union {
                uint32_t rss;          /* RSS Hash */
                struct {
                    uint16_t ip_id;    /* IP id */
                    uint16_t csum;     /* Packet Checksum */
                } csum_ip;
            } hi_dword;
        } lower;
        struct {
            uint32_t status_error;     /* ext status/error */
            uint16_t length;           /* Packet length */
            uint16_t vlan;             /* VLAN tag */
        } upper;
    } wb;  /* writeback */
};

static inline uint32_t bitfield_extract(const uint32_t value,
                                        const uint32_t mask)
{
    return (mask & value) >> (__builtin_ctz(mask));
}

#define IGB_RING_DESC_LEN       (16)

#define IGB_DEV_ID_82576    0x10C9
#define IGB_82576_VF_DEV_ID 0x10CA

#define IGB_MAX_TX_FRAGS (64)

#define IGB_MAX_VF  8

/* Transmit Descriptor bit definitions */
#define IGB_TXD_DTYP_MASK  0x00F00000 /* Data Descriptor */
#define IGB_TXD_DTYP_DATA    0x00300000 /* Data Descriptor */
#define IGB_TXD_DTYP_CONTEXT 0x00200000 /* Context Descriptor */

//upper:
#define IGB_TXD_POPTS_IXSM BIT(40-32) /* Insert IP checksum */
#define IGB_TXD_POPTS_TXSM BIT(41-32) /* Insert TCP/UDP checksum */

//#define IGB_TXD_PAYLEN_MASK (0x3ffff << (46-32))
#define IGB_TXD_IDX_MASK    (3 << (36-32))

//lower
#define IGB_TXD_CMD_EOP    0x01000000 /* End of Packet */
//#define IGB_TXD_CMD_IFCS   0x02000000 /* Insert FCS (Ethernet CRC) */
#define IGB_TXD_CMD_RS     0x08000000 /* Report Status */
#define IGB_TXD_CMD_DEXT   0x20000000 /* Descriptor extension (0 = legacy) */
#define IGB_TXD_CMD_VLE    0x40000000 /* Add VLAN tag */
#define IGB_TXD_CMD_TSE    0x80000000 /* TCP/UDP Segmentation Enable */

#define IGB_TXD_STAT_DD    0x00000001 /* Descriptor Done */



/* Receive Descriptor bit definitions */
#define IGB_RXD_STAT_DD       0x01    /* Descriptor Done */
#define IGB_RXD_STAT_EOP      0x02    /* End of Packet */
#define IGB_RXD_VLAN_SHIFT    (21-16)  /* for low_dword.hdr_info*/


#endif
