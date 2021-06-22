/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation
 *
 * Copyright (c) 2021
 *
 * Author:
 *   Alex Olson, Star Lab
 *
 *  Parts derived from Linux kernel  linux-4.19.177/drivers/net/ethernet/intel/igb/e1000_regs.h
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef _IGB_PF_REGS_H_
#define _IGB_PF_REGS_H_

#include "igb_shared_reg_constants.h"

/* I/O-Mapped Access to Internal Registers, Memories, and Flash */
#define IGB_IOADDR 0x00
#define IGB_IODATA 0x04

// EEMNGCTL fields:

/* 82574 EERD/EEWR registers layout */
#define IGB_EERW_START        BIT(0)
#define IGB_EERW_DONE         BIT(1)
#define IGB_EERW_ADDR_SHIFT   2
#define IGB_EERW_ADDR_MASK    ((1L << 14) - 1)
#define IGB_EERW_DATA_SHIFT   16


/* PHY 1000 MII Register/Bit Definitions */
/* PHY Registers defined by IEEE */
#define PHY_CTRL         0x00 /* Control Register */
#define PHY_AUTONEG_EXP  0x06 /* Autoneg Expansion Reg */
#define PHY_NEXT_PAGE_TX 0x07 /* Next Page TX */
#define PHY_LP_NEXT_PAGE 0x08 /* Link Partner Next Page */
#define PHY_EXT_STATUS   0x0F /* Extended Status Reg */

/* 82574-specific registers */
#define PHY_COPPER_CTRL1      0x10 /* Copper Specific Control Register 1 */
#define PHY_COPPER_STAT1      0x11 /* Copper Specific Status Register 1 */
#define PHY_COPPER_INT_ENABLE 0x12  /* Interrupt Enable Register */
#define PHY_COPPER_STAT2      0x13 /* Copper Specific Status Register 2 */
#define PHY_COPPER_CTRL3      0x14 /* Copper Specific Control Register 3 */
#define PHY_COPPER_CTRL2      0x1A /* Copper Specific Control Register 2 */
#define PHY_RX_ERR_CNTR       0x15  /* Receive Error Counter */
#define PHY_PAGE              0x16 /* Page Address (Any page) */
#define PHY_OEM_BITS          0x19 /* OEM Bits (Page 0) */
#define PHY_BIAS_1            0x1d /* Bias Setting Register */
#define PHY_BIAS_2            0x1e /* Bias Setting Register */

/* 82576-specific registers */
#define PHY_PAGE_SELECT        0x1F /* Page Select */

#define PHY_PAGE_RW_MASK 0x7F /* R/W part of page address register */

/* EEPROM Word Offsets */
#define EEPROM_CHECKSUM_REG           0x003F

/* For checksumming, the sum of all words in the EEPROM should equal 0xBABA. */
#define EEPROM_SUM 0xBABA



/******************************************************************************************/
/******************************************************************************************/
/******************************************************************************************/
/******************************************************************************************/


/* General */
#define IGB_CTRL     0x00000  /* Device Control - RW */
#define IGB_STATUS   0x00008  /* Device Status - RO */
#define IGB_EEC      0x00010  /* EEPROM/Flash Control - RW */
#define IGB_EERD     0x00014  /* EEPROM Read - RW */
#define IGB_CTRL_EXT 0x00018  /* Extended Device Control - RW */
#define IGB_MDIC     0x00020  /* MDI Control - RW */
#define IGB_FCAL     0x00028  /* Flow Control Address Low - RW */
#define IGB_FCAH     0x0002C  /* Flow Control Address High -RW */
#define IGB_FCT      0x00030  /* Flow Control Type - RW */
#define IGB_VET      0x00038  /* VLAN Ether Type - RW */
#define IGB_ICR      0x000C0  /* Interrupt Cause Read - R/clr */
#define IGB_ICS      0x000C8  /* Interrupt Cause Set - WO */
#define IGB_IMS      0x000D0  /* Interrupt Mask Set - RW */
#define IGB_IMC      0x000D8  /* Interrupt Mask Clear - WO */
#define IGB_IAM      0x000E0  /* Interrupt Acknowledge Auto Mask */
#define IGB_RCTL     0x00100  /* RX Control - RW */
#define IGB_FCTTV    0x00170  /* Flow Control Transmit Timer Value - RW */
#define IGB_EICR     0x01580  /* Ext. Interrupt Cause Read - R/clr */
#define IGB_EITR(_n) (0x01680 + (0x4 * (_n)))
#define IGB_EICS     0x01520  /* Ext. Interrupt Cause Set - W0 */
#define IGB_EIMS     0x01524  /* Ext. Interrupt Mask Set/Read - RW */
#define IGB_EIMC     0x01528  /* Ext. Interrupt Mask Clear - WO */
#define IGB_EIAC     0x0152C  /* Ext. Interrupt Auto Clear - RW */
#define IGB_EIAM     0x01530  /* Ext. Interrupt Ack Auto Clear Mask - RW */
#define IGB_GPIE     0x01514  /* General Purpose Interrupt Enable - RW */
#define IGB_IVAR(_n) (0x01700 + (0x4 * (_n)))
#define IGB_IVAR_MISC 0x01740 /* IVAR for "other" causes - RW */
#define IGB_TCTL     0x00400  /* TX Control - RW */
#define IGB_LEDCTL   0x00E00  /* LED Control - RW */
#define IGB_EEMNGCTL 0x01010  /* MNG EEprom Control */
#define IGB_FCRTL    0x02160  /* Flow Control Receive Threshold Low - RW */
#define IGB_FCRTH    0x02168  /* Flow Control Receive Threshold High - RW */

/* IEEE 1588 TIMESYNCH */
#define IGB_TSYNCRXCTL 0x0B620 /* Rx Time Sync Control register - RW */
#define IGB_TSYNCTXCTL 0x0B614 /* Tx Time Sync Control register - RW */
#define IGB_TSYNCRXCFG 0x05F50 /* Time Sync Rx Configuration - RW */
#define IGB_RXSTMPL    0x0B624 /* Rx timestamp Low - RO */
#define IGB_RXSTMPH    0x0B628 /* Rx timestamp High - RO */
#define IGB_TXSTMPL    0x0B618 /* Tx timestamp value Low - RO */
#define IGB_TXSTMPH    0x0B61C /* Tx timestamp value High - RO */
#define IGB_SYSTIML    0x0B600 /* System time register Low - RO */
#define IGB_SYSTIMH    0x0B604 /* System time register High - RO */
#define IGB_TIMINCA    0x0B608 /* Increment attributes register - RW */

/* Filtering Registers */
#define IGB_FTQF(_n) (0x59E0 + 4 * (_n))
#define IGB_ETQF(_n)  (0x05CB0 + (4 * (_n))) /* EType Queue Fltr */

#define IGB_RQDPC(_n) (0x0C030 + ((_n) * 0x40))

/* Receive */

#define IGB_RXPBS	0x02404  /* Rx Packet Buffer Size - RW */
/* Convenience macros
 *
 * Note: "_n" is the queue number of the register to be written to.
 *
 * Example usage:
 * IGB_RDBAL_REG(current_rx_queue)
 */
#define IGB_RDBAL(_n)   ((_n) < 4 ? (0x02800 + ((_n) * 0x100)) : (0x0C000 + ((_n) * 0x40)))
#define IGB_RDBAH(_n)   ((_n) < 4 ? (0x02804 + ((_n) * 0x100)) : (0x0C004 + ((_n) * 0x40)))
#define IGB_RDLEN(_n)   ((_n) < 4 ? (0x02808 + ((_n) * 0x100)) : (0x0C008 + ((_n) * 0x40)))
#define IGB_SRRCTL(_n)  ((_n) < 4 ? (0x0280C + ((_n) * 0x100)) : (0x0C00C + ((_n) * 0x40)))
#define IGB_RDH(_n)     ((_n) < 4 ? (0x02810 + ((_n) * 0x100)) : (0x0C010 + ((_n) * 0x40)))
#define IGB_RDT(_n)     ((_n) < 4 ? (0x02818 + ((_n) * 0x100)) : (0x0C018 + ((_n) * 0x40)))
#define IGB_RXDCTL(_n)  ((_n) < 4 ? (0x02828 + ((_n) * 0x100)) : (0x0C028 + ((_n) * 0x40)))
#define IGB_RXCSUM   0x05000  /* RX Checksum Control - RW */
#define IGB_RLPML    0x05004  /* RX Long Packet Max Length */
#define IGB_RFCTL    0x05008  /* Receive Filter Control*/
#define IGB_MTA      0x05200  /* Multicast Table Array - RW Array */
#define IGB_RAL(_i)  (((_i) <= 15) ? (0x05400 + ((_i) * 8)) : (0x054E0 + ((_i - 16) * 8)))
#define IGB_RAH(_i)  (((_i) <= 15) ? (0x05404 + ((_i) * 8)) : (0x054E4 + ((_i - 16) * 8)))
#define IGB_VFTA     0x05600  /* VLAN Filter Table Array - RW Array */
#define IGB_VT_CTL   0x0581C  /* VMDq Control - RW */

#define IGB_VT_CTL_DEF_PL_MASK (BIT(9)|BIT(8)|BIT(7))


/* Transmit */
#define IGB_TDBAL(_n)   ((_n) < 4 ? (0x03800 + ((_n) * 0x100)) : (0x0E000 + ((_n) * 0x40)))
#define IGB_TDBAH(_n)   ((_n) < 4 ? (0x03804 + ((_n) * 0x100)) : (0x0E004 + ((_n) * 0x40)))
#define IGB_TDLEN(_n)   ((_n) < 4 ? (0x03808 + ((_n) * 0x100)) : (0x0E008 + ((_n) * 0x40)))
#define IGB_TDH(_n)     ((_n) < 4 ? (0x03810 + ((_n) * 0x100)) : (0x0E010 + ((_n) * 0x40)))
#define IGB_TDT(_n)     ((_n) < 4 ? (0x03818 + ((_n) * 0x100)) : (0x0E018 + ((_n) * 0x40)))

#define IGB_TXDCTL(_n)  ((_n) < 4 ? (0x03828 + ((_n) * 0x100)) : (0x0E028 + ((_n) * 0x40)))
#define IGB_RXCTL(_n)	  ((_n) < 4 ? (0x02814 + ((_n) * 0x100)) : (0x0C014 + ((_n) * 0x40)))
#define IGB_TXCTL(_n)   ((_n) < 4 ? (0x03814 + ((_n) * 0x100)) : (0x0E014 + ((_n) * 0x40)))

#define IGB_DTXCTL   0x03590  /* DMA TX Control - RW */

// Statistics
#define IGB_CRCERRS  0x04000  /* CRC Error Count - R/clr */

#define IGB_SCVPC       0x04228  /* SerDes/SGMII Code Violation Pkt Count */


#define IGB_WUC      0x05800  /* Wakeup Control - RW */
#define IGB_WUFC     0x05808  /* Wakeup Filter Control - RW */
#define IGB_MANC     0x05820  /* Management Control - RW */

#define IGB_SW_FW_SYNC  0x05B5C /* Software-Firmware Synchronization - RW */
#define IGB_GCR         0x05B00 /* PCI-Ex Control */
#define IGB_SWSM      0x05B50 /* SW Semaphore */
#define IGB_FWSM      0x05B54 /* FW Semaphore */

/* RSS registers */
#define IGB_MRQC      0x05818 /* Multiple Receive Control - RW */
/* MSI-X Allocation Register (_i) - RW */
/* Redirection Table - RW Array */
#define IGB_RETA(_i)  (0x05C00 + ((_i) * 4))
#define IGB_RSSRK(_i) (0x05C80 + ((_i) * 4)) /* RSS Random Key - RW Array */

/* VT Registers */
#define IGB_MBVFICR   0x00C80 /* Mailbox VF Cause - RWC */
#define IGB_MBVFIMR   0x00C84 /* Mailbox VF int Mask - RW */
#define IGB_VFLRE     0x00C88 /* VF Register Events - RWC */
#define IGB_VFRE      0x00C8C /* VF Receive Enables */
#define IGB_VFTE      0x00C90 /* VF Transmit Enables */
#define IGB_QDE       0x02408 /* Queue Drop Enable - RW */
#define IGB_DTXSWC    0x03500 /* DMA Tx Switch Control - RW */
#define IGB_RPLOLR    0x05AF0 /* Replication Offload - RW */
#define IGB_UTA       0x0A000 /* Unicast Table Array - RW */
#define IGB_IOVCTL    0x05BBC /* IOV Control Register */

/* These act per VF so an array friendly macro is used */
#define IGB_PFMAILBOX(_n)    (0x00C00 + (4 * (_n)))
#define IGB_VMBMEM(_n)       (0x00800 + (64 * (_n)))
#define IGB_VMOLR(_n)        (0x05AD0 + (4 * (_n)))
#define IGB_VLVF(_n)         (0x05D00 + (4 * (_n))) /* VLAN VM Filter */
#define IGB_VMVIR(_n)        (0x03700 + (4 * (_n)))

#define VMBMEM_WORDS_PER_PF (64/4)


/* *************************************************************************** */
/* *************************************************************************** */
/* Extended Device Control */
#define IGB_CTRL_EXT_ASDCHK    0x00001000 /* ASDCHK */
#define IGB_CTRL_EXT_EE_RST	   0x00002000 /* EEPROM Reset */

/* Physical Func Reset Done Indication */
#define IGB_CTRL_EXT_PFRSTD	0x00004000

#define IGB_GPIE_NSICR		0x00000001
#define IGB_GPIE_MSIX_MODE	0x00000010
#define IGB_GPIE_EIAME     	0x40000000


#define IGB_MRQC_ENABLE_MASK         0x00000007
#define IGB_MRQC_DEFQ_MASK           0x00000078


/* Receive Control */
#define IGB_RCTL_EN             0x00000002    /* enable */
#define IGB_RCTL_SZ_2048        0x00000000    /* rx buffer size 1024 */
#define IGB_RCTL_SZ_1024        0x00010000    /* rx buffer size 1024 */
#define IGB_RCTL_SZ_512         0x00020000    /* rx buffer size 512 */
#define IGB_RCTL_SZ_256         0x00030000    /* rx buffer size 256 */
#define IGB_RCTL_SZ_MASK        0x00030000    /* rx buffer size 256 */
#define IGB_RCTL_PSP            BIT(21)       /* pad small packets */
#define IGB_RCTL_SECRC          0x04000000    /* Strip Ethernet CRC */

/* Device Control */
#define IGB_CTRL_FD       0x00000001  /* Full duplex.0=half; 1=full */
#define IGB_CTRL_ASDE     0x00000020  /* Auto-speed detect enable */
#define IGB_CTRL_SPD_SEL  0x00000300  /* Speed Select Mask */
#define IGB_CTRL_SPD_1000 0x00000200  /* Force 1Gb */
#define IGB_CTRL_FRCSPD   0x00000800  /* Force Speed */
#define IGB_CTRL_FRCDPX   0x00001000  /* Force Duplex */

#define IGB_CTRL_RST      0x04000000  /* Global reset */
#define IGB_CTRL_RFCE     0x08000000  /* Receive Flow Control enable */
#define IGB_CTRL_TFCE     0x10000000  /* Transmit flow control enable */
#define IGB_CTRL_VME      0x40000000  /* IEEE VLAN mode enable */


/* Device Status */
#define IGB_STATUS_FD         0x00000001      /* Full duplex.0=half,1=full */
#define IGB_STATUS_LU         0x00000002      /* Link up.0=no,1=link */
#define IGB_STATUS_SPEED_1000 0x00000080      /* Speed 1000Mb/s */

/* Transmit Control */
#define IGB_TCTL_EN     0x00000002    /* enable tx */

/* Interrupt Cause Read */
#define IGB_ICR_TXDW          0x00000001 /* Transmit desc written back */
#define IGB_ICR_LSC           0x00000004 /* Link Status Change */
#define IGB_ICR_RXT0          0x00000080 /* rx timer intr (ring 0) */
#define IGB_ICR_VMMB          0x00000100 /* VM MB event */
/* If this bit asserted, the driver should claim the interrupt */
/* LAN connected device generates an interrupt */


/* This defines the bits that are set in the Interrupt Mask
 * Set/Read Register.  Each bit is documented below:
 *   o RXT0   = Receiver Timer Interrupt (ring 0)
 *   o TXDW   = Transmit Descriptor Written Back
 *   o RXDMT0 = Receive Descriptor Minimum Threshold hit (ring 0)
 *   o RXSEQ  = Receive Sequence Error
 *   o LSC    = Link Status Change
 */

/* Interrupt Mask Set */
#define IGB_IMS_TXDW      IGB_ICR_TXDW      /* Transmit desc written back */
#define IGB_IMS_RXT0      IGB_ICR_RXT0      /* rx timer intr */

/* 802.1q VLAN Packet Size */

/* Receive Address */
/* Number of high/low register pairs in the RAR. The RAR (Receive Address
 * Registers) holds the directed and multicast addresses that we monitor.
 * Technically, we have 16 spots.  However, we reserve one of these spots
 * (RAR[15]) for our directed address used by controllers with
 * manageability enabled, allowing us room for 15 multicast addresses.
 */
#define IGB_RAH_AV  0x80000000        /* Receive descriptor valid */

/* PHY Control Register */
#define MII_CR_SPEED_SELECT_MSB 0x0040 /* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_FULL_DUPLEX      0x0100  /* FDX =1, half duplex =0 */
#define MII_CR_RESTART_AUTO_NEG 0x0200  /* Restart auto negotiation */
#define MII_CR_AUTO_NEG_EN      0x1000  /* Auto Neg Enable */
#define MII_CR_RESET            0x8000  /* 0 = normal, 1 = PHY reset */

/* PHY Status Register */
#define MII_SR_EXTENDED_CAPS     0x0001	/* Extended register capabilities */
#define MII_SR_LINK_STATUS       0x0004 /* Link Status 1 = link */
#define MII_SR_AUTONEG_CAPS      0x0008	/* Auto Neg Capable */
#define MII_SR_PREAMBLE_SUPPRESS 0x0040	/* Preamble may be suppressed */
#define MII_SR_EXTENDED_STATUS   0x0100	/* Ext. status info in Reg 0x0F */
#define MII_SR_10T_HD_CAPS       0x0800	/* 10T   Half Duplex Capable */
#define MII_SR_10T_FD_CAPS       0x1000	/* 10T   Full Duplex Capable */
#define MII_SR_100X_HD_CAPS      0x2000	/* 100X  Half Duplex Capable */
#define MII_SR_100X_FD_CAPS      0x4000	/* 100X  Full Duplex Capable */

/* PHY 1000 MII Register/Bit Definitions */
/* PHY Registers defined by IEEE */
#define PHY_STATUS       0x01 /* Status Register */
#define PHY_ID1          0x02 /* Phy Id Reg (word 1) */
#define PHY_ID2          0x03 /* Phy Id Reg (word 2) */
#define PHY_AUTONEG_ADV  0x04 /* Autoneg Advertisement */
#define PHY_LP_ABILITY   0x05 /* Link Partner Ability (Base Page) */
#define PHY_1000T_CTRL   0x09 /* 1000Base-T Control Reg */
#define PHY_1000T_STATUS 0x0A /* 1000Base-T Status Reg */

/* NVM Control */
#define IGB_EEC_PRES      0x00000100 /* NVM Present */

/* NVM Addressing bits based on type 0=small, 1=large */
#define IGB_EEC_AUTO_RD          0x00000200  /* NVM Auto Read done */
#define IGB_EEC_SIZE_EX_MASK     0x00007800  /* NVM Size */


/* Bit definitions for valid PHY IDs. */
/* I = Integrated
 * E = External
 */
#define I350_I_PHY_ID        0x015403B0

//* MDI Control */
#define IGB_MDIC_DATA_MASK 0x0000FFFF
#define IGB_MDIC_REG_MASK  0x001F0000
#define IGB_MDIC_PHY_MASK  0x03E00000
#define IGB_MDIC_OP_WRITE  0x04000000
#define IGB_MDIC_OP_READ   0x08000000
#define IGB_MDIC_READY     0x10000000
#define IGB_MDIC_ERROR     0x40000000


#endif
