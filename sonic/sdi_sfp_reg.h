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
 * filename: sdi_sfp_reg.h
 */


/*******************************************************************
* @file   sdi_sfp_reg.h
* @brief  Defines sfp register offsets and flag values
*******************************************************************/

#ifndef _SDI_SFP_REG_H_
#define _SDI_SFP_REG_H_

/**
 * SFP Register definitions. Spec for SFP is available
 * http://force10.dell.com/npf/Platform%20Dependent%20Hardware/eleceng/compeng/Optics%20Documents/Standards/SFF/SFF-8472.PDF
 */


/**
 * @enum sfp_reg_offset_t
 * SFP Register definitions (Table 3.1)
 */
typedef enum {
    SFP_IDENTIFIER_OFFSET           = 0,
    SFP_EXT_IDENTIFIER_OFFSET       = 1,
    SFP_CONNECTOR_OFFSET            = 2,
    SFP_COMPLIANCE_CODE_OFFSET      = 3,
    SFP_ENCODING_TYPE_OFFSET        = 11,
    SFP_NM_BITRATE_OFFSET           = 12,
    SFP_LENGTH_SMF_KM_OFFSET        = 14,
    SFP_LENGTH_SMF_OFFSET           = 15,
    SFP_LENGTH_OM2_OFFSET           = 16,
    SFP_LENGTH_OM1_OFFSET           = 17,
    SFP_LENGTH_CABLE_ASSEMBLY_OFFSET= 18,
    SFP_LENGTH_OM3_OFFSET           = 19,
    SFP_VENDOR_NAME_OFFSET          = 20,
    SFP_VENDOR_OUI_OFFSET           = 37,
    SFP_VENDOR_PN_OFFSET            = 40,
    SFP_VENDOR_REVISION_OFFSET      = 56,
    SFP_WAVELENGTH_OFFSET           = 60,
    SFP_CC_BASE_OFFSET              = 63,
    SFP_OPTIONS_OFFSET              = 64,
    SFP_MAX_BITRATE_OFFSET          = 66,
    SFP_MIN_BITRATE_OFFSET          = 67,
    SFP_VENDOR_SN_OFFSET            = 68,
    SFP_VENDOR_DATE_OFFSET          = 84,
    SFP_DIAG_MON_TYPE_OFFSET        = 92,
    SFP_ENHANCED_OPTIONS_OFFSET     = 93,
    SFP_CC_EXT_OFFSET               = 95,
    SFP_DELL_PRODUCT_ID_OFFSET         = 96,
} sfp_reg_offset_t;


/**
 * @def Attribute used to define the i2c address of Diagnostics. All diagnostics
 * are OPTIONAL and one should check SFP_DIAG_MON_TYPE_OFFSET at address 0xa0 to
 * know which are supported
 */
#define SFP_DIAG_MNTR_I2C_ADDR 0x51

/**
 * @enum sfp_diag_mntr_reg_offset_t
 * Alarm and Warning Thresholds
 */
typedef enum {
    SFP_TEMP_HIGH_ALARM_THRESHOLD_OFFSET    = 0,
    SFP_TEMP_LOW_ALARM_THRESHOLD_OFFSET     = 2,
    SFP_TEMP_HIGH_WARNING_THRESHOLD_OFFSET  = 4,
    SFP_TEMP_LOW_WARNING_THRESHOLD_OFFSET   = 6,
    SFP_VOLT_HIGH_ALARM_THRESHOLD_OFFSET    = 8,
    SFP_VOLT_LOW_ALARM_THRESHOLD_OFFSET     = 10,
    SFP_VOLT_HIGH_WARNING_THRESHOLD_OFFSET  = 12,
    SFP_VOLT_LOW_WARNING_THRESHOLD_OFFSET   = 14,
    SFP_BIAS_HIGH_ALARM_THRESHOLD_OFFSET    = 16,
    SFP_BIAS_LOW_ALARM_THRESHOLD_OFFSET     = 18,
    SFP_BIAS_HIGH_WARNING_THRESHOLD_OFFSET  = 20,
    SFP_BIAS_LOW_WARNING_THRESHOLD_OFFSET   = 22,
    SFP_TX_PWR_HIGH_ALARM_THRESHOLD_OFFSET  = 24,
    SFP_TX_PWR_LOW_ALARM_THRESHOLD_OFFSET   = 26,
    SFP_TX_PWR_HIGH_WARNING_THRESHOLD_OFFSET= 28,
    SFP_TX_PWR_LOW_WARNING_THRESHOLD_OFFSET = 30,
    SFP_RX_PWR_HIGH_ALARM_THRESHOLD_OFFSET  = 32,
    SFP_RX_PWR_LOW_ALARM_THRESHOLD_OFFSET   = 34,
    SFP_RX_PWR_HIGH_WARNING_THRESHOLD_OFFSET= 36,
    SFP_RX_PWR_LOW_WARNING_THRESHOLD_OFFSET = 38,
    SFP_CALIB_RX_POWER_CONST_START_OFFSET   = 56,
    SFP_CALIB_TX_BIAS_SLOPE_OFFSET          = 76,
    SFP_CALIB_TX_BIAS_CONST_OFFSET          = 78,
    SFP_CALIB_TX_POWER_SLOPE_OFFSET         = 80,
    SFP_CALIB_TX_POWER_CONST_OFFSET         = 82,
    SFP_CALIB_TEMP_SLOPE_OFFSET             = 84,
    SFP_CALIB_TEMP_CONST_OFFSET             = 86,
    SFP_CALIB_VOLT_SLOPE_OFFSET             = 88,
    SFP_CALIB_VOLT_CONST_OFFSET             = 90,
    SFP_TEMPERATURE_OFFSET                  = 96,
    SFP_VOLTAGE_OFFSET                      = 98,
    SFP_TX_BIAS_CURRENT_OFFSET              = 100,
    SFP_TX_OUTPUT_POWER_OFFSET              = 102,
    SFP_RX_INPUT_POWER_OFFSET               = 104,
    SFP_ALARM_STATUS_1_OFFSET       = 112,
    SFP_ALARM_STATUS_2_OFFSET       = 113,
    SFP_WARNING_STATUS_1_OFFSET     = 116,
    SFP_WARNING_STATUS_2_OFFSET     = 117
} sfp_diag_mntr_reg_offset_t;

/**
 * @enum sfp_diag_mntr_bit_mask_t
 * SFP Diagnostic Monitoring Type bit definitions
 */
typedef enum {
    SFP_ADDR_CHANGE_REQUIRED_BIT_OFFSET  = 2,
    SFP_RCVD_PWR_TYPE_BIT_OFFSET    = 3,
    SFP_CALIB_TYPE_EXTERNAL_BIT_OFFSET  = 4,
    SFP_CALIB_TYPE_INTERNAL_BIT_OFFSET  = 5,
    SFP_DDM_SUPPORT_BIT_OFFSET      = 6,
} sfp_diag_mntr_bit_mask_t;

/**
 * @enum sfp_enhanced_options_bit_mask_t
 * SFP Enhanced options bit definitions
 */
typedef enum {
    SFP_SOFT_RATE_SELECT_CTRL_BIT_OFFSET    = 1,
    SFP_APPLICATION_SELECT_CTRL_BIT_OFFSET  = 2,
    SFP_SOFT_RATE_SELECT_MNTR_BIT_OFFSET    = 3,
    SFP_SOFT_RX_LOS_MNTR_BIT_OFFSET         = 4,
    SFP_SOFT_TX_FAULT_MNTR_BIT_OFFSET       = 5,
    SFP_SOFT_TX_FAULT_CTRL_BIT_OFFSET       = 6,
    SFP_ALARM_SUPPORT_BIT_OFFSET            = 7
} sfp_enhanced_options_bit_mask_t;

/* Diagnostic Monitoring Type Address A0h, Byte 92 */
#define SFP_DIAG_MNTR_BIT_OFFSET    (6)

/* Enhanced Options [Address A0h, Byte 93] */
#define SFP_RATE_SELECT_BIT_OFFSET  (1)

/* Sec 8.6 -  Alarm and Warning Flags (Page A2) */
/* offset 112 */
#define SFP_TEMP_HIGH_ALARM_FLAG    (1 << 7) /* 0x80 */
#define SFP_TEMP_LOW_ALARM_FLAG     (1 << 6) /* 0x40 */
#define SFP_VOLT_HIGH_ALARM_FLAG    (1 << 5) /* 0x20 */
#define SFP_VOLT_LOW_ALARM_FLAG     (1 << 4) /* 0x10 */
#define SFP_TX_BIAS_HIGH_ALARM_FLAG (1 << 3) /* 0x08 */
#define SFP_TX_BIAS_LOW_ALARM_FLAG  (1 << 2) /* 0x04 */
#define SFP_TX_PWR_HIGH_ALARM_FLAG  (1 << 1) /* 0x02 */
#define SFP_TX_PWR_LOW_ALARM_FLAG   (1)      /* 0x01 */
/* offset 113 */
#define SFP_RX_PWR_HIGH_ALARM_FLAG  (1 << 7) /* 0x80 */
#define SFP_RX_PWR_LOW_ALARM_FLAG   (1 << 6) /* 0x40 */
/* offset 116 */
#define SFP_TEMP_HIGH_WARNING_FLAG  (1 << 7) /* 0x80 */
#define SFP_TEMP_LOW_WARNING_FLAG   (1 << 6) /* 0x40 */
#define SFP_VOLT_HIGH_WARNING_FLAG  (1 << 5) /* 0x20 */
#define SFP_VOLT_LOW_WARNING_FLAG   (1 << 4) /* 0x10 */
#define SFP_TX_BIAS_HIGH_WARNING_FLAG (1 << 3) /* 0x08 */
#define SFP_TX_BIAS_LOW_WARNING_FLAG  (1 << 2) /* 0x04 */
#define SFP_TX_PWR_HIGH_WARNING_FLAG  (1 << 1) /* 0x02 */
#define SFP_TX_PWR_LOW_WARNING_FLAG   (1)      /* 0x01 */
/* offset 117 */
#define SFP_RX_PWR_HIGH_WARNING_FLAG  (1 << 7) /* 0x80 */
#define SFP_RX_PWR_LOW_WARNING_FLAG   (1 << 6) /* 0x40 */

/* Table 3.17 Option Values (Page A2) */
#define SFP_LOS_BIT_OFFSET  0x2
#define SFP_TX_DISABLE_OFFSET        0x40
#define SFP_TX_FAULT_OFFSET            0x4

#define SDI_SFP_MAGIC_KEY_SIZE            2
#define SDI_SFP_DELL_PRODUCT_ID_MAGIC0    0x0F
#define SDI_SFP_DELL_PRODUCT_ID_MAGIC1    0x10

/**
 * @def Attribute used to define the i2c address of Copper SFP PHY.
 */

#define SFP_PHY_I2C_ADDR  0x56

/** CuSFP Register Values **/
#define SFP_COPPER_CTRL_SS_MSB         (1 << 6) /* Speed select, MSb */
#define SFP_COPPER_CTRL_FD             (1 << 8) /* Full Duplex */
#define SFP_COPPER_CTRL_RAN            (1 << 9) /* Restart Autonegotiation */
#define SFP_COPPER_CTRL_IP             (1 << 10) /* Isolate Phy */
#define SFP_COPPER_CTRL_PD             (1 << 11) /* Power Down */
#define SFP_COPPER_CTRL_AE             (1 << 12) /* Autonegotiation enable */
#define SFP_COPPER_CTRL_SS_LSB         (1 << 13) /* Speed select, LSb */
#define SFP_COPPER_CTRL_LE             (1 << 14) /* Loopback enable */
#define SFP_COPPER_CTRL_RESET          (1 << 15) /* PHY reset */

/** CuSFP registers **/
#define SFP_COPPER_CTRL_REG            0x00
#define SFP_COPPER_STAT_REG            0x01
#define SFP_COPPER_ANA_REG             0x04
#define SFP_COPPER_GB_CTRL_REG         0x09
#define SFP_COPPER_EXT_STATUS_1_REG    0x11
#define SFP_COPPER_EXT_STATUS_2_REG    0x1B
/** MII link advertisement registers */
#define SFP_COPPER_ANA_ASF             (1 << 0)/* Advertise Selector Field */
#define SFP_COPPER_ANA_HD_10           (1 << 5)/* Half duplex 10Mb/s supported */
#define SFP_COPPER_ANA_FD_10           (1 << 6)/* Full duplex 10Mb/s supported */
#define SFP_COPPER_ANA_HD_100          (1 << 7)/* Half duplex 100Mb/s supported */
#define SFP_COPPER_ANA_FD_100          (1 << 8)/* Full duplex 100Mb/s supported */
#define SFP_COPPER_GB_CTRL_ADV_1000FD  (1 << 9) /* Advertise 1000Base-T FD */
#define SFP_COPPER_GB_CTRL_ADV_1000HD  (1 << 8) /* Advertise 1000Base-T HD */

#define SFP_ETH_10G_CODE_MASK          (0xF)  /* ethernet 10g code mask */
#define SFP_ETH_10G_CODE_BIT_SHIFT     (4)
#define SFP_INFINIBAND_CODE_MASK       (0xF)  /* SFP infiniband code mask */
#define SFP_ESCON_CODE_MASK            (0x3)  /* SFP ESCON Compliance Codes mask */
#define SFP_ESCON_CODE_BIT_SHIFT       (6)
#define SFP_SONET_CODE_MASK            (0x3F) /* SFP SONET code bit mask */
#define SFP_SONET_CODE_BIT_SHIFT       (8)
#define SFP_FC_DISTANCE_MASK           (0x1F) /* Fibre Channel Link Length mask */
#define SFP_FC_DISTANCE_BIT_SHIFT      (3)
#define SFP_FC_TECH_MASK1              (0x7)  /* Fibre Channel Technology mask */
#define SFP_FC_TECH_MASK2              (0xF)  /* Fibre Channel Technology mask */
#define SFP_FC_TECH_BIT_SHIFT          (4)
#define SFP_PLUS_CABLE_TECH_MASK       (0x3)  /* SFP+ Cable Technology mask */
#define SFP_PLUS_CABLE_TECH_BIT_SHIFT  (2)

#endif
