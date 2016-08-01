/*
 * Copyright (c) 2016 Dell Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * THIS CODE IS PROVIDED ON AN  *AS IS* BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 *  LIMITATION ANY IMPLIED WARRANTIES OR CONDITIONS OF TITLE, FITNESS
 * FOR A PARTICULAR PURPOSE, MERCHANTABLITY OR NON-INFRINGEMENT.
 *
 * See the Apache Version 2.0 License for specific language governing
 * permissions and limitations under the License.
 */

/*
 * filename: sdi_qsfp_reg.h
 */


/*******************************************************************
* @file   sdi_qsfp_reg.h
* @brief  Defines qsfp register offsets and flag values
*******************************************************************/

#ifndef __SDI_QSFP_REG_H_
#define __SDI_QSFP_REG_H_

/**
 * QSFP Register definitions. Spec for QSFP is available
 * http://force10.dell.com/npf/Platform%20Dependent%20Hardware/eleceng/compeng/Optics%20Documents/Standards/SFF/SFF-8436.PDF
 */
typedef enum {
    QSFP_STATUS_INDICATOR_OFFSET    = 2,
    QSFP_CHANNEL_LOS_INDICATOR      = 3,
    QSFP_CHANNEL_TXFAULT_INDICATOR  = 4,
    QSFP_TEMP_INTERRUPT_OFFSET      = 6,
    QSFP_VOLT_INTERRUPT_OFFSET      = 7,
    QSFP_RX12_POWER_INTERRUPT_OFFSET= 9,
    QSFP_RX34_POWER_INTERRUPT_OFFSET= 10,
    QSFP_TX12_BIAS_INTERRUPT_OFFSET = 11,
    QSFP_TX34_BIAS_INTERRUPT_OFFSET = 12,
    QSFP_TEMPERATURE_OFFSET         = 22,
    QSFP_VOLTAGE_OFFSET             = 26,
    QSFP_RX1_POWER_OFFSET           = 34,
    QSFP_RX2_POWER_OFFSET           = 36,
    QSFP_RX3_POWER_OFFSET           = 38,
    QSFP_RX4_POWER_OFFSET           = 40,
    QSFP_TX1_POWER_BIAS_OFFSET      = 42,
    QSFP_TX2_POWER_BIAS_OFFSET      = 44,
    QSFP_TX3_POWER_BIAS_OFFSET      = 46,
    QSFP_TX4_POWER_BIAS_OFFSET      = 48,
    QSFP_TX_CONTROL_OFFSET          = 86,
    QSFP_PAGE_SELECT_BYTE_OFFSET    = 127,
    QSFP_IDENTIFIER_OFFSET          = 128,
    QSFP_EXT_IDENTIFIER_OFFSET      = 129,
    QSFP_CONNECTOR_OFFSET           = 130,
    QSFP_COMPLIANCE_CODE_OFFSET     = 131,
    QSFP_ENCODING_TYPE_OFFSET       = 139,
    QSFP_NM_BITRATE_OFFSET          = 140,
    QSFP_LENGTH_SMF_KM_OFFSET       = 142,
    QSFP_LENGTH_OM3_OFFSET          = 143,
    QSFP_LENGTH_OM2_OFFSET          = 144,
    QSFP_LENGTH_OM1_OFFSET          = 145,
    QSFP_LENGTH_CABLE_ASSEMBLY_OFFSET = 146,
    QSFP_DEVICE_TECH_OFFSET         = 147,
    QSFP_VENDOR_NAME_OFFSET         = 148,
    QSFP_VENDOR_OUI_OFFSET          = 165,
    QSFP_VENDOR_PN_OFFSET           = 168,
    QSFP_VENDOR_REVISION_OFFSET     = 184,
    QSFP_WAVELENGTH_OFFSET          = 186,
    QSFP_WAVELENGTH_TOLERANCE_OFFSET= 188,
    QSFP_MAX_CASE_TEMP_OFFSET       = 190,
    QSFP_CC_BASE_OFFSET             = 191,
    QSFP_OPTIONS1_OFFSET            = 192,
    QSFP_OPTIONS2_OFFSET            = 193,
    QSFP_OPTIONS3_OFFSET            = 194,
    QSFP_OPTIONS4_OFFSET            = 195,
    QSFP_VENDOR_SN_OFFSET           = 196,
    QSFP_VENDOR_DATE_OFFSET         = 212,
    QSFP_DIAG_MON_TYPE_OFFSET       = 220,
    QSFP_ENHANCED_OPTIONS_OFFSET    = 221,
    QSFP_CC_EXT_OFFSET              = 223,
    QSFP_DELL_PRODUCT_ID_OFFSET     = 240,
    QSFP_DELL_PRODUCT_ID_OFFSET_SEC = 248,
} qsfp_reg_offset_t;

/* Table 46 — Module and Channel Thresholds (Page 03) */
typedef enum {
    QSFP_TEMP_HIGH_ALARM_THRESHOLD_OFFSET         = 128,
    QSFP_TEMP_LOW_ALARM_THRESHOLD_OFFSET          = 130,
    QSFP_TEMP_HIGH_WARNING_THRESHOLD_OFFSET       = 132,
    QSFP_TEMP_LOW_WARNING_THRESHOLD_OFFSET        = 134,
    QSFP_VOLT_HIGH_ALARM_THRESHOLD_OFFSET         = 144,
    QSFP_VOLT_LOW_ALARM_THRESHOLD_OFFSET          = 146,
    QSFP_VOLT_HIGH_WARNING_THRESHOLD_OFFSET       = 148,
    QSFP_VOLT_LOW_WARNING_THRESHOLD_OFFSET        = 150,
    QSFP_RX_PWR_HIGH_ALARM_THRESHOLD_OFFSET       = 176,
    QSFP_RX_PWR_LOW_ALARM_THRESHOLD_OFFSET        = 178,
    QSFP_RX_PWR_HIGH_WARNING_THRESHOLD_OFFSET     = 180,
    QSFP_RX_PWR_LOW_WARNING_THRESHOLD_OFFSET      = 182,
    QSFP_TX_BIAS_HIGH_ALARM_THRESHOLD_OFFSET      = 184,
    QSFP_TX_BIAS_LOW_ALARM_THRESHOLD_OFFSET       = 186,
    QSFP_TX_BIAS_HIGH_WARNING_THRESHOLD_OFFSET    = 188,
    QSFP_TX_BIAS_LOW_WARNING_THRESHOLD_OFFSET     = 190,
} qsfp_page3_reg_offset_t;


/* Table 18 - Status Indicators (Page A0) */
#define QSFP_FLAT_MEM_BIT_OFFSET    2

/* Table 19 — Channel status Interrupt Flags (Page A0) */
#define QSFP_TX_LOS_BIT_OFFSET  0x10
#define QSFP_RX_LOS_BIT_OFFSET  0x01

/* Table 20 - Module Monitor Interrupt Flags (Page A0) */
#define QSFP_TEMP_HIGH_ALARM_FLAG    (1 << 7) /* 0x80 */
#define QSFP_TEMP_LOW_ALARM_FLAG     (1 << 6) /* 0x40 */
#define QSFP_TEMP_HIGH_WARNING_FLAG  (1 << 5) /* 0x20 */
#define QSFP_TEMP_LOW_WARNING_FLAG   (1 << 4) /* 0x10 */

#define QSFP_VOLT_HIGH_ALARM_FLAG    (1 << 7) /* 0x80 */
#define QSFP_VOLT_LOW_ALARM_FLAG     (1 << 6) /* 0x40 */
#define QSFP_VOLT_HIGH_WARNING_FLAG  (1 << 5) /* 0x20 */
#define QSFP_VOLT_LOW_WARNING_FLAG   (1 << 4) /* 0x10 */

/* Table 21 — Channel Monitor Interrupt Flags (Page A0)*/
/* offset 9 and 10 */
/* QSFP_RX13_XXXX -> RX13 represents channel 1 and channel 3 of QSFP */
/* QSFP_RX24_XXXX -> RX24 represents channel 2 and channel 4 of QSFP */
#define QSFP_RX13_POWER_HIGH_ALARM_FLAG      (1 << 7) /* 0x80 */
#define QSFP_RX13_POWER_LOW_ALARM_FLAG       (1 << 6) /* 0x40 */
#define QSFP_RX13_POWER_HIGH_WARNING_FLAG    (1 << 5) /* 0x20 */
#define QSFP_RX13_POWER_LOW_WARNING_FLAG     (1 << 4) /* 0x10 */
#define QSFP_RX24_POWER_HIGH_ALARM_FLAG      (1 << 3) /* 0x08 */
#define QSFP_RX24_POWER_LOW_ALARM_FLAG       (1 << 2) /* 0x04 */
#define QSFP_RX24_POWER_HIGH_WARNING_FLAG    (1 << 1) /* 0x02 */
#define QSFP_RX24_POWER_LOW_WARNING_FLAG     (1 << 0) /* 0x01 */

/* offset 11 and 12 */
/* QSFP_TX13_XXXX -> TX13 represents channel 1 and channel 3 of QSFP */
/* QSFP_TX24_XXXX -> TX24 represents channel 2 and channel 4 of QSFP */
#define QSFP_TX13_BIAS_HIGH_ALARM_FLAG      (1 << 7) /* 0x80 */
#define QSFP_TX13_BIAS_LOW_ALARM_FLAG       (1 << 6) /* 0x40 */
#define QSFP_TX13_BIAS_HIGH_WARNING_FLAG    (1 << 5) /* 0x20 */
#define QSFP_TX13_BIAS_LOW_WARNING_FLAG     (1 << 4) /* 0x10 */
#define QSFP_TX24_BIAS_HIGH_ALARM_FLAG      (1 << 3) /* 0x08 */
#define QSFP_TX24_BIAS_LOW_ALARM_FLAG       (1 << 2) /* 0x04 */
#define QSFP_TX24_BIAS_HIGH_WARNING_FLAG    (1 << 1) /* 0x02 */
#define QSFP_TX24_BIAS_LOW_WARNING_FLAG     (1 << 0) /* 0x01 */

/* Table 39 — Option Values (Address 195) (Page 00) */
#define QSFP_TX_DISABLE_BIT_OFFSET  4
#define QSFP_RATE_SELECT_BIT_OFFSET 5

#define QSFP_WAVELENGTH_DIVIDER             20
#define QSFP_WAVELENGTH_TOLERANCE_DIVIDER   200

#define SDI_QSFP_MAGIC_KEY_SIZE            2
#define SDI_QSFP_DELL_PRODUCT_ID_MAGIC0    0x0F
#define SDI_QSFP_DELL_PRODUCT_ID_MAGIC1    0x10

#endif
