/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC emulation
 * (PF Register index definitions)
 *
 * Copyright (c) 2021
 *
 * Author:
 *   Alex Olson, Star Lab
 *   (based on earlier 'igb' work by Kung Omang)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef HW_NET_IGB_PF_REG_H
#define HW_NET_IGB_PF_REG_H

#include "igb_common.h"
#include "igb_pf_reg_constants.h"

enum {

// General Register Descriptions
    defreg(CTRL),
    defreg(STATUS),
    defreg(CTRL_EXT),
    defreg(MDIC),
    defreg(LEDCTL),
    defreg(VET),

// Packet Buffers Control Register Descriptions
    defreg(RXPBS),

// EEPROM/Flash Register Descriptions
    defreg(EEMNGCTL),

// MNG-EEPROM CSR I/F
    defreg(EEC),
    defreg(EERD),

// Flow Control Register Descriptions
    defreg(FCAL),
    defreg(FCAH),
    defreg(FCT),
    defreg(FCTTV),
    defreg(FCRTL),
    defreg(FCRTH),

// PCIe Register Descriptions
    defreg(GCR),

// Semaphore registers
    defreg(FWSM),
    defreg(SWSM),
    defreg(SW_FW_SYNC),

// Interrupt Register Descriptions
    defreg(EIAC),
    defreg(EIAM),
    defreg(EICR),
    defreg(EICS),
    defreg(EIMC),
    defreg(EIMS),
    defreg(IAM),
    defreg(ICR),
    defreg(ICS),
    defreg(IMC),
    defreg(IMS),

    defreg_8(IVAR),
    defreg(IVAR_MISC),
    defreg_25(EITR),

    defreg(GPIE),

// MSI-X Table Register Descriptions

// Receive Register Descriptions
    defreg(RCTL),
    defreg(RXCSUM),
    defreg(RFCTL),
    defreg(MRQC),
    defreg(RLPML),


    defreg_24(RAL),
    defreg_24(RAH),
    defreg_16(RQDPC),
    defreg_10(RSSRK),
    defreg_32(RETA),

    defreg_16(RDBAL),
    defreg_16(RDBAH),
    defreg_16(RDLEN),
    defreg_16(RDH),
    defreg_16(RDT),
    defreg_16(RXDCTL),
    defreg_16(RXCTL),
    defreg_16(SRRCTL),



// Filtering Register Descriptions
    defreg_8(FTQF),
    defreg_8(ETQF),

// Transmit Register Descriptions
    defreg(TCTL),
    defreg_16(TDBAL),
    defreg_16(TDBAH),
    defreg_16(TDLEN),
    defreg_16(TDH),
    defreg_16(TDT),
    defreg_16(TXDCTL),
    defreg_16(TXCTL),

    defreg(DTXCTL),

// DCA Register Descriptions

// Virtualization Register Descriptions
    defreg(DTXSWC),
    defreg(VT_CTL),
    defreg(RPLOLR),
    defreg(MBVFICR),
    defreg(MBVFIMR),
    defreg(VFRE),
    defreg(VFTE),
    defreg(QDE),
    defreg(VFLRE),
    defreg_8(VMOLR),
    defreg_8(VMVIR),
    defreg_32(VLVF),
    defreg_8(PFMAILBOX),

// Tx Bandwidth Allocation to VM Register Description

// Timer Register Descriptions

// Time Sync Register Descriptions
    defreg(RXSTMPH),
    defreg(RXSTMPL),
    defreg(SYSTIMH),
    defreg(SYSTIML),
    defreg(TIMINCA),
    defreg(TSYNCRXCTL),
    defreg(TSYNCRXCFG),
    defreg(TSYNCTXCTL),
    defreg(TXSTMPH),
    defreg(TXSTMPL),

// PCS Register Descriptions

// Statistics Register Descriptions

// Wake Up Control Register Descriptions
    defreg(WUC),
    defreg(WUFC),

// Management Register Descriptions
    defreg(MANC),

// MACSec Register Descriptions

// MACSec RX Key - LSECRXKEY (0xB350 + 16*n [n=0

// MACSec Tx Port Statistics

// MACSec Rx Port Statistic

// MACSec Rx SC Statistic Register Descriptions

// MACSec Rx SA Statistic Register Descriptions

// IPsec Registers Description

// Diagnostic Registers Description

// PCIe
    defreg(IOVCTL),
///////////////////////////////////////////////
};

#endif
