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
 * filename: sdi_sfp_eeprom.c
 */


/******************************************************************************
 * sdi_sfp_eeprom.c
 * Implements the SFP eeprom related APIs
 *****************************************************************************/
#include "sdi_sfp.h"
#include "sdi_sfp_reg.h"
#include "sdi_media.h"
#include "sdi_media_attr.h"
#include "sdi_media_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_device_common.h"
#include "sdi_pin_group_bus_framework.h"
#include "sdi_pin_group_bus_api.h"
#include "sdi_i2c_bus_api.h"
#include "std_error_codes.h"
#include "std_assert.h"
#include "std_time_tools.h"
#include "std_bit_ops.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Delay for accesing phy device
   When phy device is enabled it needs delay before access*/
#define SFP_PHY_DELAY (100 * 1000)
/* Magic number description not given in appnote from Marvell
   an-2036 app note from Mavell give below magic value to enable
   SGMII mode for phy device */
#define PHY_SGMII_MODE 0x9084

 /*SFP parameter sizes */
enum {
    SDI_SFP_BYTE_SIZE = 1,
    SDI_SFP_WORD_SIZE = 2,
    SDI_SFP_DOUBLE_WORD_SIZE = 4,
    SDI_SFP_QUAD_WORD_SIZE = 8,
};

/* sfp register information structure */
typedef struct sdi_sfp_reg_info {
    uint_t offset; /* register offset */
    uint_t size; /* register size */
} sdi_sfp_reg_info_t;

#define SDI_SFP_PADDING_CHAR 0

/* parameter register information strucutre. Parameters should be defined in the
 * same order of sdi_media_param_type_t */
static sdi_sfp_reg_info_t param_reg_info[] = {
    { SFP_WAVELENGTH_OFFSET, SDI_SFP_WORD_SIZE }, /* for SDI_MEDIA_WAVELENGTH */
    { 0, 0 }, /* for SDI_MEDIA_WAVELENGTH_TOLERANCE */
    { 0, 0 }, /* for SDI_MEDIA_MAX_CASE_TEMP */
    { SFP_CC_BASE_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_CC_BASE */
    { SFP_CC_EXT_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_CC_EXT */
    { SFP_CONNECTOR_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_CONNECTOR */
    { SFP_ENCODING_TYPE_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_ENCODING_TYPE */
    { SFP_NM_BITRATE_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_NM_BITRATE */
    { SFP_IDENTIFIER_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_IDENTIFIER */
    { SFP_EXT_IDENTIFIER_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_EXT_IDENTIFIER */
    { SFP_LENGTH_SMF_KM_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_SMF_KM */
    { SFP_LENGTH_OM1_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_OM1 */
    { SFP_LENGTH_OM2_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_OM2 */
    { SFP_LENGTH_OM3_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_OM3 */
    { SFP_LENGTH_CABLE_ASSEMBLY_OFFSET, SDI_SFP_BYTE_SIZE  }, /* for SDI_MEDIA_LENGTH_CABLE_ASSEMBLY */
    { SFP_LENGTH_SMF_OFFSET, SDI_SFP_BYTE_SIZE}, /* for SDI_MEDIA_LENGTH_SMF, not supported on SFP */
    { SFP_OPTIONS_OFFSET, SDI_SFP_WORD_SIZE }, /* for SDI_MEDIA_OPTIONS */
    { SFP_ENHANCED_OPTIONS_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_ENHANCED_OPTIONS */
    { SFP_DIAG_MON_TYPE_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_DIAG_MON_TYPE */
    { 0,0 }, /* for SDI_MEDIA_DEVICE_TECH */
    { SFP_MAX_BITRATE_OFFSET, SDI_SFP_BYTE_SIZE } , /* for SDI_MEDIA_MAX_BITRATE */
    { SFP_MIN_BITRATE_OFFSET, SDI_SFP_BYTE_SIZE }, /* for SDI_MEDIA_MIN_BITRATE */
};

/* vendor register information structure. Parameters in this structure should be
 * defined in the same order of sdi_media_vendor_info_type_t */
static sdi_sfp_reg_info_t vendor_reg_info[] = {
    { SFP_VENDOR_NAME_OFFSET, SDI_MEDIA_MAX_VENDOR_NAME_LEN }, /* for SDI_MEDIA_VENDOR_NAME */
    { SFP_VENDOR_OUI_OFFSET, SDI_MEDIA_MAX_VENDOR_OUI_LEN }, /* for SDI_MEDIA_VENDOR_OUI */
    { SFP_VENDOR_SN_OFFSET, SDI_MEDIA_MAX_VENDOR_SERIAL_NUMBER_LEN }, /* for SDI_MEDIA_VENDOR_SN */
    { SFP_VENDOR_DATE_OFFSET, SDI_MEDIA_MAX_VENDOR_DATE_LEN }, /* for SDI_MEDIA_VENDOR_DATE */
    { SFP_VENDOR_PN_OFFSET, SDI_MEDIA_MAX_VENDOR_PART_NUMBER_LEN }, /* for SDI_MEDIA_VENDOR_PN */
    { SFP_VENDOR_REVISION_OFFSET, SDI_MEDIA_MAX_VENDOR_REVISION_LEN } /* for SDI_MEDIA_VENDOR_REVISION */
};


/* threshold value register information structure. Parameters in this structure
 * should be defined in the same order of sdi_media_threshold_type_t */
static sdi_sfp_reg_info_t threshold_reg_info[] = {
    /* for SDI_MEDIA_TEMP_HIGH_ALARM_THRESHOLD */
    { SFP_TEMP_HIGH_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TEMP_LOW_ALARM_THRESHOLD */
    { SFP_TEMP_LOW_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TEMP_HIGH_WARNING_THRESHOLD */
    { SFP_TEMP_HIGH_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TEMP_LOW_WARNING_THRESHOLD */
    { SFP_TEMP_LOW_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_HIGH_ALARM_THRESHOLD */
    { SFP_VOLT_HIGH_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_LOW_ALARM_THRESHOLD */
    { SFP_VOLT_LOW_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_HIGH_WARNING_THRESHOLD */
    { SFP_VOLT_HIGH_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_LOW_WARNING_THRESHOLD */
    { SFP_VOLT_LOW_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_HIGH_ALARM_THRESHOLD */
    { SFP_RX_PWR_HIGH_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_LOW_ALARM_THRESHOLD */
    { SFP_RX_PWR_LOW_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_HIGH_WARNING_THRESHOLD */
    { SFP_RX_PWR_HIGH_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_LOW_WARNING_THRESHOLD */
    { SFP_RX_PWR_LOW_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_HIGH_ALARM_THRESHOLD */
    { SFP_BIAS_HIGH_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_LOW_ALARM_THRESHOLD */
    { SFP_BIAS_LOW_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_HIGH_WARNING_THRESHOLD */
    { SFP_BIAS_HIGH_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_LOW_WARNING_THRESHOLD */
    { SFP_BIAS_LOW_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_PWR_HIGH_ALARM_THRESHOLD */
    { SFP_TX_PWR_HIGH_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_PWR_LOW_ALARM_THRESHOLD */
    { SFP_TX_PWR_LOW_ALARM_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_PWR_HIGH_WARNING_THRESHOLD */
    { SFP_TX_PWR_HIGH_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_PWR_LOW_WARNING_THRESHOLD */
    { SFP_TX_PWR_LOW_WARNING_THRESHOLD_OFFSET, SDI_SFP_WORD_SIZE },
};

/* This function enables the particular device on a bus */
static inline t_std_error sdi_sfp_module_select(sdi_device_hdl_t sfp_device)
{
    t_std_error rc = STD_ERR_OK;
    sfp_device_t *sfp_priv_data = NULL;

    STD_ASSERT(sfp_device != NULL);
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    /* Check whether module selection required or not on this particular sfp. If
     * module selectin is not required just return STD_ERR_OK */
    if(sfp_priv_data->mod_sel_hdl != NULL) {
        rc = sdi_pin_group_acquire_bus(sfp_priv_data->mod_sel_hdl);
        if (rc != STD_ERR_OK){
            return rc;
        }

        rc = sdi_pin_group_write_level(sfp_priv_data->mod_sel_hdl,
                                       sfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            /* module selection failed, hence release the lock.*/
            sdi_pin_group_release_bus(sfp_priv_data->mod_sel_hdl);
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s rc : %d",
                    sfp_device->alias, rc);
        }
    }
    /* If module selection success, releasing the lock taken care by
     * sdi_sfp_module_deselect api */
    return rc;
}

/* This function disables the particular device on a bus */
static inline void sdi_sfp_module_deselect(sfp_device_t *sfp_priv_data)
{
    STD_ASSERT(sfp_priv_data != NULL);

    if(sfp_priv_data->mod_sel_hdl != NULL) {
        sdi_pin_group_release_bus(sfp_priv_data->mod_sel_hdl);
    }
}

/* This function checks whether Alarm/warning flags implemented for this module.
 * Make sure that module is already selected before calling this function */
static inline t_std_error sdi_is_alarm_flags_supported(sdi_device_hdl_t sfp_device,
                                                       bool *support_status)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(sfp_device != NULL);
    STD_ASSERT(support_status != NULL);

    *support_status =  false;

    rc = sdi_smbus_read_byte(sfp_device->bus_hdl, sfp_device->addr.i2c_addr,
                             SFP_ENHANCED_OPTIONS_OFFSET, &buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d reg : %d for %s rc : %d",
                              sfp_device->addr, SFP_ENHANCED_OPTIONS_OFFSET, sfp_device->alias, rc);
        return rc;
    }

    if( (STD_BIT_TEST(buf, SFP_ALARM_SUPPORT_BIT_OFFSET) != 0 ) ) {
       *support_status = true;
    } else {
       *support_status = false;
    }

   return rc;
}

/* This function checks whether diag monitoring is supported or not on this divice
 * Make sure that module is already selected before calling this function */
static inline t_std_error sdi_is_diag_monitoring_supported(sdi_device_hdl_t sfp_device,
                                                           bool *support_status)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(sfp_device != NULL);
    STD_ASSERT(support_status != NULL);

    *support_status =  false;

    rc = sdi_smbus_read_byte(sfp_device->bus_hdl, sfp_device->addr.i2c_addr,
                             SFP_DIAG_MON_TYPE_OFFSET, &buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Getting of diag monitoring value is failed for %s rc : %d",
                              sfp_device->alias, rc);
        return rc;
    }

    if( (STD_BIT_TEST(buf, SFP_DDM_SUPPORT_BIT_OFFSET) == 0) ) {
        *support_status = false;
    } else {
        *support_status = true;
    }

    return rc;
}

/* This function checks whether rate select is supported or not on this divice
 * Make sure that module is already selected before calling this function */
static inline t_std_error sdi_is_rate_select_supported(sdi_device_hdl_t sfp_device,
                                                       bool *support_status)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(sfp_device != NULL);
    STD_ASSERT(support_status != NULL);

    *support_status =  false;

    rc = sdi_smbus_read_byte(sfp_device->bus_hdl, sfp_device->addr.i2c_addr,
                             SFP_ENHANCED_OPTIONS_OFFSET, &buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Getting of diag monitoring value is failed for %s rc : %d",
                              sfp_device->alias, rc);
        return rc;
    }

    if( (STD_BIT_TEST(buf, SFP_RATE_SELECT_BIT_OFFSET) == 0) ) {
        *support_status = false;
    } else {
        *support_status = true;
    }

    return rc;
}

/* This function converts temperature raw data to human readable format based on
 * calibration types. SFP standard supports two types of calibration. One is
 * internal calibration and other is external calibration.
 *
 * 1. Internal calibration : Internally measured transceiver temperature
 * represented as a 16 bit signed twos complement value in increments of 1/256
 * degrees Celsius, yielding a total range of -128C to +128C. The temperature in
 * degrees Celsius is given by the signed twos complement value with LSB equal
 * to 1/256 C. For more details "Internal caibration" section in sff-8472 spec.
 *
 * 2. External calibration : Internally measured transceiver temperature, T is
 * given by the following quation
 * T(C) = T(slope)* Reg_value(16 bit signed twos complement value) + T(offset).
 * The result is in units of 1/256C, yielding a total range of –128C to +128C.
 * */
static inline float convert_sfp_temp(uint8_t *buf, sfp_calib_info_t *calib_info)
{
    int16_t temp = 0;
    float calib_temp = 0;
    float slope = 0;
    bool is_negative = false;

    temp = ( (buf[0] << BITS_PER_BYTE ) | (buf[1]) );

    if( temp < 0 ){
        /* Negative value */
        temp = ((~temp) + 1);
        is_negative = true;
    }

    if(calib_info->type == SFP_CALIB_TYPE_INTERNAL) {
        calib_temp = ((float)(temp & 0xFF) / 256.0) + ((temp & 0xFF00) >> 8);
    } else if(calib_info->type == SFP_CALIB_TYPE_EXTERNAL) {
        slope = ( (float)(calib_info->slope[1] / 256 ) + calib_info->slope[0] );
        calib_temp = ( ((slope * temp) + calib_info->offset) / 256 );
    }

    if(is_negative == true) {
        calib_temp = -calib_temp;
    }
    return calib_temp;
}

/* This function converts voltage raw data to human readable format based on
 * calibration types. SFP standard supports two types of calibration. One is
 * internal calibration and other is external calibration.
 *
 * 1. Internal calibration : Internally measured transceiver supply voltage,
 * represented as a 16 bit unsigned integer with the voltage defined as the full
 * 16 bit value (0 – 65535) with LSB equal to 100 uVolt, yielding a total range
 * of 0 to +6.55 Volts.
 *
 * 2. External calibration : Internally measured supply voltage. Module internal
 * supply voltage, V, is given in microvolts by the following equation:
 * V(uV) = VSLOPE* VAD (16 bit unsigned integer)+ VOFFSET.
 * The result is in units of 100uV, yielding a total range of 0 – 6.55V.
 */
static inline float convert_sfp_volt(uint8_t *buf, sfp_calib_info_t *calib_info)
{
    uint16_t volt = 0;
    float calib_volt = 0;
    float slope = 0;

    volt = ( (buf[0] << BITS_PER_BYTE ) | (buf[1]) );

    if(calib_info->type == SFP_CALIB_TYPE_INTERNAL) {
        /* LSB is equal to 100uV.converting to V */
        calib_volt = ((float)(volt) / (10000.0));
    } else if(calib_info->type == SFP_CALIB_TYPE_EXTERNAL) {
        slope = ( (float)(calib_info->slope[1] / 256 ) + calib_info->slope[0] );
        calib_volt = ( ((slope * volt) + calib_info->offset) / (10000.0) );
    }
    return calib_volt;
}

/* This function converts temperature raw data to human readable format based on
 * calibration types. SFP standard supports two types of calibration. One is
 * internal calibration and other is external calibration.
 *
 * 1. Internal calibration : Internally measured RX received optical power in
 * mW, represented as a 16 bit unsigned integer with the power defined as the
 * full 16 bit value (0 – 65535) with LSB equal to 0.1 uW, yielding a total
 * range of 0 to 6.5535 mW (~ -40 to +8.2 dBm).
 *
 * 2. External calibration : Internally measured received optical power(RX_PWR),
 * is given in uW by the following equation:
 *      Rx_PWR (uW) = Rx_PWR(4) * Rx_PWRAD4 (16 bit unsigned integer) +
 *                    Rx_PWR(3) * Rx_PWRAD3 (16 bit unsigned integer) +
 *                    Rx_PWR(2) * Rx_PWRAD2 (16 bit unsigned integer) +
 *                    Rx_PWR(1) * Rx_PWRAD (16 bit unsigned integer) +
 *                    Rx_PWR(0)
 * The result is in units of 0.1uW yielding a total range of 0 – 6.5mW.
 * */
static inline float convert_sfp_rx_power(uint8_t *buf, sfp_rx_power_calib_info_t *calib_info)
{
    uint16_t rx_power = 0;
    float calib_rx_power = 0;

    rx_power = ( (buf[0] << BITS_PER_BYTE ) | (buf[1]) );

    if(calib_info->type == SFP_CALIB_TYPE_INTERNAL) {
        calib_rx_power = ( ((float)rx_power) / 10000.0 );
    } else if(calib_info->type == SFP_CALIB_TYPE_EXTERNAL) {
        calib_rx_power = ( (((float)(*(calib_info->rx_power_const_4))) * (powf((float)rx_power, 4.0))) +
                           (((float)(*(calib_info->rx_power_const_3))) * (powf((float)rx_power, 3.0))) +
                           (((float)(*(calib_info->rx_power_const_2))) * (powf((float)rx_power, 2.0))) +
                           (((float)(*(calib_info->rx_power_const_1))) * ((float)rx_power)) +
                           ((float)(*calib_info->rx_power_const_0)) );
    }
    return calib_rx_power;
}

/* This function converts temperature raw data to human readable format based on
 * calibration types. SFP standard supports two types of calibration. One is
 * internal calibration and other is external calibration.
 *
 * 1. Internal calibration : Internally measured TX output power in mW.
 * Represented as a 16 bit unsigned integer with the power defined as the full
 * 16 bit value (0 – 65535) with LSB equal to 0.1 uW, yielding a total range of
 * 0 to 6.5535 mW (~ -40 to +8.2 dBm).
 *
 * 2. External calibration : Internally measured module transmitter coupled
 * output power, TX_PWR, is given in uW by the following equation:
 *      TX_PWR (uW) = TX_PWRSLOPE * TX_PWRAD  +  TX_PWROFFSET.
 * This result is in units of 0.1uW yielding a total range of 0 – 6.5mW.
 * */
static inline float convert_sfp_tx_power(uint8_t *buf, sfp_calib_info_t *calib_info)
{
    uint16_t tx_pwr = 0;
    float calib_tx_pwr = 0;
    float slope = 0;

    tx_pwr = ( (buf[0] << BITS_PER_BYTE ) | (buf[1]) );

    if(calib_info->type == SFP_CALIB_TYPE_INTERNAL) {
        /* LSB is 0.1uW. converting to mW */
        calib_tx_pwr = ((float)(tx_pwr) / (10000.0));
    } else if(calib_info->type == SFP_CALIB_TYPE_EXTERNAL) {
        slope = ( (float)(calib_info->slope[1] / 256 ) + calib_info->slope[0] );
        /* Result is in units of 0.1uW. converting to mW */
        calib_tx_pwr = ( ((slope * tx_pwr) + calib_info->offset) / (10000.0) );
    }
    return calib_tx_pwr;
}

/* This function converts temperature raw data to human readable format based on
 * calibration types. SFP standard supports two types of calibration. One is
 * internal calibration and other is external calibration.
 *
 * 1. Internal calibration : Internally measured TX bias current in uA is
 * represented as a 16 bit unsigned integer with the current defined as the full
 * 16 bit value (0 – 65535) with LSB equal to 2 uA, yielding a total range of 0
 * to 131 mA.
 *
 * 2. External calibration : Internally measured module laser bias current, I,
 * is given in microamps by the following equation:
 *      I (uA) = ISLOPE * IAD (16 bit unsigned integer) + IOFFSET.
 * This result is in units of 2 uA, yielding a total range of 0 to 131 mA.
 * */
static inline float convert_sfp_tx_bias_current(uint8_t *buf, sfp_calib_info_t *calib_info)
{
    uint16_t tx_bias = 0;
    float calib_tx_bias = 0;
    float slope = 0;

    tx_bias = ( (buf[0] << BITS_PER_BYTE ) | (buf[1]) );

    if(calib_info->type == SFP_CALIB_TYPE_INTERNAL) {
        /*LSB is equal to 2uA. Converting to mA */
        calib_tx_bias = ( ((float)(tx_bias * 2)) / (1000.0) );
    } else if(calib_info->type == SFP_CALIB_TYPE_EXTERNAL) {
        slope = ( (float)(calib_info->slope[1] / 256 ) + calib_info->slope[0] );
        calib_tx_bias = ( (slope * tx_bias) + calib_info->offset );
        /* Converting to mA as the calc gives units of 2uA */
        calib_tx_bias = ( ((float)(calib_tx_bias * 2)) / (1000.0) );
    }
    return calib_tx_bias;
}

/**
 * Get the required module status of the specific sfp
 * resource_hdl[in] - Handle of the resource
 * flags[in]        - flags for status that are of interest
 * status[out]    - returns the set of status flags which are asserted
 * return           - t_std_error
 */
t_std_error sdi_sfp_module_monitor_status_get (sdi_resource_hdl_t resource_hdl,
                                               uint_t flags, uint_t *status)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    bool alarm_support_status = false;
    uint8_t alarm_status_buf = 0;
    uint8_t warning_status_buf = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);
    *status = 0;

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        /* Check whether Alarm/warning flags implemented for this module */
        rc = sdi_is_alarm_flags_supported(sfp_device, &alarm_support_status);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("SFP alarm support status get is failed for %s, rc : %d",
                                  sfp_device->alias, rc);
            break;
        }

        if(!alarm_support_status) {
            rc = SDI_DEVICE_ERRCODE(EOPNOTSUPP);
            break;
        }

        if( ( (flags) & ( (SDI_MEDIA_STATUS_TEMP_HIGH_ALARM)  |
                          (SDI_MEDIA_STATUS_TEMP_LOW_ALARM)   |
                          (SDI_MEDIA_STATUS_VOLT_HIGH_ALARM)  |
                          (SDI_MEDIA_STATUS_VOLT_LOW_ALARM) ) ) != 0) {
            rc = sdi_smbus_read_byte(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                    SFP_ALARM_STATUS_1_OFFSET, &alarm_status_buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s, rc : %d",
                        sfp_device->addr, sfp_device->alias, rc);
                break;
            }
        }

        if( ( (flags) & ( (SDI_MEDIA_STATUS_TEMP_HIGH_WARNING)  |
                          (SDI_MEDIA_STATUS_TEMP_LOW_WARNING)   |
                          (SDI_MEDIA_STATUS_VOLT_HIGH_WARNING)  |
                          (SDI_MEDIA_STATUS_VOLT_LOW_WARNING) ) ) != 0) {
            rc = sdi_smbus_read_byte(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                    SFP_WARNING_STATUS_1_OFFSET,
                    &warning_status_buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                        sfp_device->addr, sfp_device->alias, rc);
                break;
            }
        }
    } while(0);

    sdi_sfp_module_deselect(sfp_priv_data);

    if( rc == STD_ERR_OK) {
        if((alarm_status_buf & SFP_TEMP_HIGH_ALARM_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_TEMP_HIGH_ALARM;
        } else if ((alarm_status_buf & SFP_TEMP_LOW_ALARM_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_TEMP_LOW_ALARM;
        }

        if((alarm_status_buf & SFP_VOLT_HIGH_ALARM_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_VOLT_HIGH_ALARM;
        } else if ((alarm_status_buf & SFP_VOLT_LOW_ALARM_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_VOLT_LOW_ALARM;
        }
        if((warning_status_buf & SFP_TEMP_HIGH_WARNING_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_TEMP_HIGH_WARNING;
        } else if ((warning_status_buf & SFP_TEMP_LOW_WARNING_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_TEMP_LOW_WARNING;
        }

        if((warning_status_buf & SFP_VOLT_HIGH_WARNING_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_VOLT_HIGH_WARNING;
        } else if ((warning_status_buf & SFP_VOLT_LOW_WARNING_FLAG) != 0) {
            *status |= SDI_MEDIA_STATUS_VOLT_LOW_WARNING;
        }
    }
    return rc;
}

/**
 * Get the required channel monitoring(rx_power, tx_bias alarm) status of the specific SFP
 * resource_hdl[in] - Handle of the resource
 * channel[in]      - channel number that is of interest
 * flags[in]        - flags for channel status
 * status[out]      - returns the set of status flags which are asserted
 * return           - t_std_error
 */
t_std_error sdi_sfp_channel_monitor_status_get (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                                uint_t flags, uint_t *status)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    bool alarm_support_status = false;
    uint8_t tx_alarm_status = 0;
    uint8_t rx_alarm_status = 0;
    uint8_t tx_warn_status = 0;
    uint8_t rx_warn_status = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);
    *status = 0;

    if(channel != SDI_SFP_CHANNEL_NUM) {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        /* Check whether Alarm/warning flags implemented for this module */
        rc = sdi_is_alarm_flags_supported(sfp_device, &alarm_support_status);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("SFP alarm support status get is failed for %s, rc : %d",
                                  sfp_device->alias, rc);
            break;
        }

        if(!alarm_support_status) {
            rc = SDI_DEVICE_ERRCODE(EOPNOTSUPP);
            break;
        }

        if( ( (flags) & ((SDI_MEDIA_TX_BIAS_HIGH_ALARM) | (SDI_MEDIA_TX_BIAS_LOW_ALARM) |
                         (SDI_MEDIA_TX_PWR_HIGH_ALARM) | (SDI_MEDIA_TX_PWR_LOW_ALARM)) ) != 0 ) {
            rc = sdi_smbus_read_byte(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                                     SFP_ALARM_STATUS_1_OFFSET, &tx_alarm_status, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                                      sfp_device->addr, sfp_device->alias, rc);
                break;
            }
        }

        if( ( (flags) & ((SDI_MEDIA_RX_PWR_HIGH_ALARM) | (SDI_MEDIA_RX_PWR_LOW_ALARM)) ) != 0 ) {
            rc = sdi_smbus_read_byte(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                                     SFP_ALARM_STATUS_2_OFFSET, &rx_alarm_status, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                                      sfp_device->addr, sfp_device->alias, rc);
                break;
            }
        }

        if( ( (flags) & ((SDI_MEDIA_TX_PWR_HIGH_WARNING) | (SDI_MEDIA_TX_PWR_LOW_WARNING) |
                        (SDI_MEDIA_TX_BIAS_HIGH_WARNING) | (SDI_MEDIA_TX_BIAS_LOW_WARNING))) != 0 ) {
            rc = sdi_smbus_read_byte(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                                     SFP_WARNING_STATUS_1_OFFSET, &tx_warn_status, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                                      sfp_device->addr, sfp_device->alias, rc);
                break;
            }
        }

        if( ((flags) & ((SDI_MEDIA_RX_PWR_HIGH_WARNING) | (SDI_MEDIA_RX_PWR_LOW_WARNING)) ) != 0 ) {
            rc = sdi_smbus_read_byte(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                                     SFP_WARNING_STATUS_2_OFFSET, &rx_warn_status, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                                      sfp_device->addr, sfp_device->alias, rc);
                break;
            }
        }
    }while(0);
    sdi_sfp_module_deselect(sfp_priv_data);

    if( rc == STD_ERR_OK) {
        if( ( (tx_alarm_status & SFP_TX_BIAS_HIGH_ALARM_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_BIAS_HIGH_ALARM;
        } else if ( ( (tx_alarm_status & SFP_TX_BIAS_LOW_ALARM_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_BIAS_LOW_ALARM;
        }

        if( ( (tx_alarm_status & SFP_TX_PWR_HIGH_ALARM_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_PWR_HIGH_ALARM;
        } else if ( ( (tx_alarm_status & SFP_TX_PWR_LOW_ALARM_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_PWR_LOW_ALARM;
        }

        if( ( (rx_alarm_status & SFP_RX_PWR_HIGH_ALARM_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_RX_PWR_HIGH_ALARM;
        } else if ( ( (tx_alarm_status & SFP_RX_PWR_LOW_ALARM_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_RX_PWR_LOW_ALARM;
        }

        if( ( (tx_warn_status & SFP_TX_BIAS_HIGH_WARNING_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_BIAS_HIGH_WARNING;
        } else if ( ( (tx_warn_status & SFP_TX_BIAS_LOW_WARNING_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_BIAS_LOW_WARNING;
        }

        if( ( (tx_warn_status & SFP_TX_PWR_HIGH_WARNING_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_PWR_HIGH_WARNING;
        } else if ( ( (tx_warn_status & SFP_TX_PWR_LOW_WARNING_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_TX_PWR_LOW_WARNING;
        }

        if( ( (rx_warn_status & SFP_RX_PWR_HIGH_WARNING_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_RX_PWR_HIGH_WARNING;
        } else if ( ( (tx_warn_status & SFP_RX_PWR_LOW_WARNING_FLAG) != 0 ) ) {
            *status |= SDI_MEDIA_RX_PWR_LOW_WARNING;
        }
    }
    return rc;

}

/**
 * Get the required channel status of the specific SFP
 * resource_hdl[in] - Handle of the resource
 * channel[in]      - channel number that is of interest
 * flags[in]        - flags for channel status
 * status[out]    - returns the set of status flags which are asserted
 * return           - t_std_error
 * TODO : As of now this API supports to get the status of required flags from
 * system CPLD. Need to add support to get the same from optic eeprom
 */
t_std_error sdi_sfp_channel_status_get (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                        uint_t flags, uint_t *status)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t value = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);
    *status = 0;

    if(channel != SDI_SFP_CHANNEL_NUM) {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    if( ((flags) & (SDI_MEDIA_STATUS_TXDISABLE)) != 0 ) {
        rc = sdi_pin_group_acquire_bus(sfp_priv_data->mod_tx_control_hdl);
        if(rc != STD_ERR_OK) {
            return rc;
        }
        rc = sdi_pin_group_read_level(sfp_priv_data->mod_tx_control_hdl, &value);
        sdi_pin_group_release_bus(sfp_priv_data->mod_tx_control_hdl);

        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("tx disable status get failed for %s rc : %d",
                                  sfp_device->alias, rc);
            return rc;
        }

        if( (STD_BIT_TEST(value, sfp_priv_data->mod_tx_control_bitmask) != 0) ) {
            *status |= SDI_MEDIA_STATUS_TXDISABLE;
        }
    }

    if( ((flags) & (SDI_MEDIA_STATUS_TXFAULT)) != 0 ) {
        value = 0;
        rc = sdi_pin_group_acquire_bus(sfp_priv_data->mod_sfp_tx_fault_hdl);
        if(rc != STD_ERR_OK) {
            return rc;
        }
        rc = sdi_pin_group_read_level(sfp_priv_data->mod_sfp_tx_fault_hdl, &value);
        sdi_pin_group_release_bus(sfp_priv_data->mod_sfp_tx_fault_hdl);

        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("tx disable status get failed for %s rc : %d",
                                  sfp_device->alias, rc);
            return rc;
        }

        if ( (STD_BIT_TEST(value, sfp_priv_data->mod_sfp_tx_fault_bitmask) != 0) )  {
            *status |= SDI_MEDIA_STATUS_TXFAULT;
        }
    }

    if( ((flags) & (SDI_MEDIA_STATUS_RXLOSS)) != 0 ) {
        value = 0;

        rc = sdi_pin_group_acquire_bus(sfp_priv_data->mod_sfp_rx_los_hdl);
        if(rc != STD_ERR_OK) {
            return rc;
        }
        rc = sdi_pin_group_read_level(sfp_priv_data->mod_sfp_rx_los_hdl, &value);
        sdi_pin_group_release_bus(sfp_priv_data->mod_sfp_rx_los_hdl);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("rx los status get failed for %s rc : %d",
                                  sfp_device->alias, rc);
            return rc;
        }

        if ( (STD_BIT_TEST(value, sfp_priv_data->mod_sfp_rx_los_bitmask) != 0) ) {
            *status |= SDI_MEDIA_STATUS_RXLOSS;
        }
    }

    if( ( (flags) & (SDI_MEDIA_STATUS_TXLOSS) ) != 0 ) {
        /* tx_los is not supported on SFP */
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    return rc;
}

/**
 * Disable/Enable the transmitter of the specific SFP
 * resource_hdl[in] - handle of the resource
 * channel[in]      - channel number that is of interest. This variable should
 * be 0 for SFP.
 * enable[in]       - "false" to disable and "true" to enable
 * return           - t_std_error
 * TODO : As of now this API supports to control the tx disable from system
 * CPLD. Need to add support to set the same in optice eeprom.
 */
t_std_error sdi_sfp_tx_control (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                bool enable)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t value = 0;

    STD_ASSERT(resource_hdl != NULL);

    if(channel != SDI_SFP_CHANNEL_NUM) {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(sfp_priv_data->mod_tx_control_hdl);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        rc = sdi_pin_group_read_level(sfp_priv_data->mod_tx_control_hdl, &value);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("tx control status get failed for %s rc : %d",
                    sfp_device->alias, rc);
            break;
        }

        if(enable) {
            STD_BIT_CLEAR(value, sfp_priv_data->mod_tx_control_bitmask);
        } else {
            STD_BIT_SET(value, sfp_priv_data->mod_tx_control_bitmask);
        }

        rc = sdi_pin_group_write_level(sfp_priv_data->mod_tx_control_hdl, value);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("tx control status set failed for %s rc : %d",
                    sfp_device->alias, rc);
        }
    } while(0);
    sdi_pin_group_release_bus(sfp_priv_data->mod_tx_control_hdl);
    return rc;
}

/**
 * Gets the transmitter status on the specific channel of a SFP
 * resource_hdl[in] - handle of the resource
 * channel[in]      - channel number that is of interest
 * status[out]      - "true" if transmitter enabled, else "false"
 * return           - t_std_error
 */
t_std_error sdi_sfp_tx_control_status_get(sdi_resource_hdl_t resource_hdl,
                                          uint_t channel, bool *status)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t value = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    if(channel != SDI_SFP_CHANNEL_NUM) {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(sfp_priv_data->mod_tx_control_hdl);
    if (rc != STD_ERR_OK) {
        return rc;
    }

    rc = sdi_pin_group_read_level(sfp_priv_data->mod_tx_control_hdl, &value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("tx control status get failed for %s rc : %d",
                              sfp_device->alias, rc);
    }
    sdi_pin_group_release_bus(sfp_priv_data->mod_tx_control_hdl);

    if(rc == STD_ERR_OK) {
        if( (STD_BIT_TEST(value, sfp_priv_data->mod_tx_control_bitmask) != 0) ) {
            *status = false;
        } else {
            *status = true;
        }
    }
    return rc;
}

/**
 * Returns the maximum speed that can be supported by SFP module
 * resource_hdl[in] - handle of the media resource
 * speed[out]     - speed of the media will be filled in this
 * return           - standard t_std_error
 */
t_std_error  sdi_sfp_speed_get(sdi_resource_hdl_t resource_hdl,
                               sdi_media_speed_t *speed)
{
    *speed = SDI_MEDIA_SPEED_10G;
    return STD_ERR_OK;
}

/**
 * Check whether the specified SFP is dell qualified
 * resource_hdl[in] - handle of the resource
 * status[out]    - true if optics is dell qualified else false
 * return           - t_std_error
 */
t_std_error sdi_sfp_is_dell_qualified (sdi_resource_hdl_t resource_hdl,
                                       bool *status)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t magic_key[SDI_SFP_MAGIC_KEY_SIZE] = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    rc = sdi_smbus_read_word(sfp_device->bus_hdl, sfp_device->addr.i2c_addr,
                             SFP_DELL_PRODUCT_ID_OFFSET, (uint16_t *)magic_key, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK){
        SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                              sfp_device->addr, sfp_device->alias, rc);
    }

    sdi_sfp_module_deselect(sfp_priv_data);

    if(rc == STD_ERR_OK) {
        if ( (magic_key[0] == SDI_SFP_DELL_PRODUCT_ID_MAGIC0) &&
             (magic_key[1] == SDI_SFP_DELL_PRODUCT_ID_MAGIC1) )
        {
            *status = true;
        } else {
            *status = false;
        }
    }

    return rc;
}

/**
 * Reads the requested parameter value from eeprom
 * resource_hdl[in] - handle of the media resource
 * param[in]        - parametr type that is of interest(e.g wavelength, maximum
 *                    case temperature etc)
 * value[out]     - parameter value which is read from eeprom
 * return           - standard t_std_error
 */
t_std_error sdi_sfp_parameter_get(sdi_resource_hdl_t resource_hdl,
                                  sdi_media_param_type_t param, uint_t *value)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t byte_buf = 0;
    uint8_t word_buf[2] = { 0 } ;
    uint_t offset = 0;
    uint_t size = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    offset = param_reg_info[param].offset;
    size =  param_reg_info[param].size;

    if( (offset == 0) && (size == 0) ) {
        SDI_DEVICE_ERRMSG_LOG("parameter type : %u is unsupported on %s", param,
                              sfp_device->alias);
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        switch (size)
        {
            case SDI_SFP_BYTE_SIZE:
                rc = sdi_smbus_read_byte(sfp_device->bus_hdl, sfp_device->addr.i2c_addr, offset,
                                         &byte_buf, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d reg : %d for %s rc : %d",
                            sfp_device->addr, offset, sfp_device->alias, rc);
                }
                break;

            case SDI_SFP_WORD_SIZE:
                rc = sdi_smbus_read_word(sfp_device->bus_hdl, sfp_device->addr.i2c_addr, offset,
                                         (uint16_t *)word_buf, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d reg : %d for %s rc : %d",
                            sfp_device->addr, offset, sfp_device->alias, rc);
                }
                break;

            default:
                rc = SDI_DEVICE_ERRCODE(EINVAL);
        }
    } while(0);

    sdi_sfp_module_deselect(sfp_priv_data);

    if(rc == STD_ERR_OK) {
        if(size == SDI_SFP_BYTE_SIZE) {
            *value = (uint_t)byte_buf;
        } else if(size == SDI_SFP_WORD_SIZE) {
            *value = ( (word_buf[0] << 8) | (word_buf[1]) );
        }
    }
    return rc;
}

/**
 * Read the requested vendor information of a specific media resource
 * resource_hdl[in]     - handle of the media resource
 * vendor_info_type[in] - vendor information that is of interest.
 * vendor_info[out]   - vendor information which is read from eeprom
 * buf_size[in]         - size of the input buffer(vendor_info)
 * return               - standard t_std_error
 */
t_std_error sdi_sfp_vendor_info_get(sdi_resource_hdl_t resource_hdl,
                                    sdi_media_vendor_info_type_t vendor_info_type,
                                    char *vendor_info, size_t size)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    size_t data_len = 0;
    uint_t offset = 0;
    uint8_t *buf_ptr = NULL;
    uint8_t data_buf[SDI_MAX_NAME_LEN] = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(vendor_info != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    offset = vendor_reg_info[vendor_info_type].offset;
    data_len = vendor_reg_info[vendor_info_type].size;

    /* Input buffer size should be greater than or equal to data len*/
    STD_ASSERT(size >= data_len);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    rc = sdi_smbus_read_multi_byte(sfp_device->bus_hdl, sfp_device->addr.i2c_addr,
                                   offset, data_buf, data_len, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d reg : %d for %s rc : %d",
                              sfp_device->addr, offset, sfp_device->alias, rc);
    }

    sdi_sfp_module_deselect(sfp_priv_data);

    if(rc == STD_ERR_OK) {
        if( (vendor_info_type == SDI_MEDIA_VENDOR_DATE) ||
            (vendor_info_type == SDI_MEDIA_VENDOR_OUI) ) {
            snprintf(vendor_info, data_len, "%s", data_buf);
        } else {
            /* vendor name, part number, serial number and revision fields contains
             * ASCII characters, left-aligned and padded on the right with ASCII
             * spaces (20h).*/
            for(buf_ptr = &data_buf[data_len]; *(buf_ptr - 1) == SDI_SFP_PADDING_CHAR; buf_ptr--);
            *buf_ptr = '\0';
            snprintf(vendor_info, data_len, "%s", data_buf);
        }
    }
    return rc;
}

/**
 * Read the transceiver compliance code information for a specific media resource
 * resource_hdl[in]         - handle of the media resource
 * transceiver_info[out]    - transceiver information which is read from eeprom
 * return                   - standard t_std_error
 */
t_std_error sdi_sfp_transceiver_code_get(sdi_resource_hdl_t resource_hdl,
                                         sdi_media_transceiver_descr_t *transceiver_info)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    uint8_t xvr_buff[SDI_SFP_QUAD_WORD_SIZE];
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(transceiver_info != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    memset(xvr_buff, 0, sizeof(xvr_buff));
    rc = sdi_smbus_read_multi_byte(sfp_device->bus_hdl, sfp_device->addr.i2c_addr,
                                   SFP_COMPLIANCE_CODE_OFFSET, xvr_buff,
                                   SDI_SFP_QUAD_WORD_SIZE, SDI_I2C_FLAG_NONE);

    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                              sfp_device->addr, sfp_device->alias, rc);
    }

    sdi_sfp_module_deselect(sfp_priv_data);

    transceiver_info->sfp_descr.sdi_sfp_eth_10g_code
        = (xvr_buff[0] >> SFP_ETH_10G_CODE_BIT_SHIFT) & SFP_ETH_10G_CODE_MASK;
    transceiver_info->sfp_descr.sdi_sfp_infiniband_code
        = xvr_buff[0] & SFP_INFINIBAND_CODE_MASK;
    transceiver_info->sfp_descr.sdi_sfp_escon_code
        = (xvr_buff[1] >> SFP_ESCON_CODE_BIT_SHIFT) & SFP_ESCON_CODE_MASK;
    transceiver_info->sfp_descr.sdi_sfp_sonet_code
        = xvr_buff[2] | ((xvr_buff[1] & SFP_SONET_CODE_MASK) << SFP_SONET_CODE_BIT_SHIFT);
    transceiver_info->sfp_descr.sdi_sfp_eth_1g_code = xvr_buff[3];
    transceiver_info->sfp_descr.sdi_sfp_fc_distance
        = (xvr_buff[4] >> SFP_FC_DISTANCE_BIT_SHIFT) & SFP_FC_DISTANCE_MASK;
    transceiver_info->sfp_descr.sdi_sfp_fc_technology
        = ((xvr_buff[5] >> SFP_FC_TECH_BIT_SHIFT) & SFP_FC_TECH_MASK2)
             | ((xvr_buff[4] & SFP_FC_TECH_MASK1) << SFP_FC_TECH_BIT_SHIFT);
    transceiver_info->sfp_descr.sdi_sfp_plus_cable_technology
        = (xvr_buff[5] >> SFP_PLUS_CABLE_TECH_BIT_SHIFT) & SFP_PLUS_CABLE_TECH_MASK;
    transceiver_info->sfp_descr.sdi_sfp_fc_media = xvr_buff[6];
    transceiver_info->sfp_descr.sdi_sfp_fc_speed = xvr_buff[7];

    return rc;
}

/**
 * Read the dell product information
 * resource_hdl[in] - Handle of the resource
 * info[out]        - dell product information
 * return           - standard t_std_error
 */
t_std_error sdi_sfp_dell_product_info_get(sdi_resource_hdl_t resource_hdl,
                                          sdi_media_dell_product_info_t *info)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(info != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    rc = sdi_smbus_read_multi_byte(sfp_device->bus_hdl, sfp_device->addr.i2c_addr,
                                   SFP_DELL_PRODUCT_ID_OFFSET, (uint8_t *)info, sizeof(sdi_media_dell_product_info_t),
                                   SDI_I2C_FLAG_NONE);

    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %d for %s rc : %d",
                              sfp_device->addr, sfp_device->alias, rc);
    }
    sdi_sfp_module_deselect(sfp_priv_data);
    return rc;
}

/**
 * Get the alarm and warning theresholds for the given optic
 * resource_hdl[in]     - Handle of the resource
 * threshold_type[in]   - type of threshold
 * value[out]           - threshold value
 * return               - standard t_std_error
 */
t_std_error sdi_sfp_threshold_get(sdi_resource_hdl_t resource_hdl,
                                  sdi_media_threshold_type_t threshold_type,
                                  float *value)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t diag_mon_value = 0;
    uint_t offset = 0;
    uint8_t threshold_buf[2] = { 0 };
    sfp_calib_info_t calib_info = { 0 };
    uint_t slope_offset = 0;
    uint_t const_offset = 0;
    sfp_rx_power_calib_info_t rx_pwr_calib_info = { 0 };
    uint8_t rx_power_data[EXT_CAL_RX_POWER_LEN] = { 0 };
    uint8_t *rx_power_data_ptr = NULL;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    /* Check whether diag monitoring is supported on this device or not */
    rc = sdi_sfp_parameter_get(resource_hdl, SDI_MEDIA_DIAG_MON_TYPE, &diag_mon_value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Getting of diag monitoring value is failed for %s rc : %d",
                              sfp_device->alias, rc);
        return rc;
    }

    if( (STD_BIT_TEST(diag_mon_value, SFP_DDM_SUPPORT_BIT_OFFSET) == 0) ) {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    /* Get the calibration type */
    if( (STD_BIT_TEST(diag_mon_value, SFP_CALIB_TYPE_EXTERNAL_BIT_OFFSET) != 0) ) {
        calib_info.type = SFP_CALIB_TYPE_EXTERNAL;
    } else  if( (STD_BIT_TEST(diag_mon_value, SFP_CALIB_TYPE_INTERNAL_BIT_OFFSET) != 0) ) {
        calib_info.type = SFP_CALIB_TYPE_INTERNAL;
    } else {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    offset = threshold_reg_info[threshold_type].offset;

    switch(threshold_type)
    {
        case SDI_MEDIA_TEMP_HIGH_ALARM_THRESHOLD:
        case SDI_MEDIA_TEMP_LOW_ALARM_THRESHOLD:
        case SDI_MEDIA_TEMP_HIGH_WARNING_THRESHOLD:
        case SDI_MEDIA_TEMP_LOW_WARNING_THRESHOLD:
            slope_offset = SFP_CALIB_TEMP_SLOPE_OFFSET;
            const_offset = SFP_CALIB_TEMP_CONST_OFFSET;
            break;

        case SDI_MEDIA_VOLT_HIGH_ALARM_THRESHOLD:
        case SDI_MEDIA_VOLT_LOW_ALARM_THRESHOLD:
        case SDI_MEDIA_VOLT_HIGH_WARNING_THRESHOLD:
        case SDI_MEDIA_VOLT_LOW_WARNING_THRESHOLD:
            slope_offset = SFP_CALIB_VOLT_SLOPE_OFFSET;
            const_offset = SFP_CALIB_VOLT_CONST_OFFSET;
            break;

        case SDI_MEDIA_TX_BIAS_HIGH_ALARM_THRESHOLD:
        case SDI_MEDIA_TX_BIAS_LOW_ALARM_THRESHOLD:
        case SDI_MEDIA_TX_BIAS_HIGH_WARNING_THRESHOLD:
        case SDI_MEDIA_TX_BIAS_LOW_WARNING_THRESHOLD:
            slope_offset = SFP_CALIB_TX_BIAS_SLOPE_OFFSET;
            const_offset = SFP_CALIB_TX_BIAS_CONST_OFFSET;
            break;

        case SDI_MEDIA_TX_PWR_HIGH_ALARM_THRESHOLD:
        case SDI_MEDIA_TX_PWR_LOW_ALARM_THRESHOLD:
        case SDI_MEDIA_TX_PWR_HIGH_WARNING_THRESHOLD:
        case SDI_MEDIA_TX_PWR_LOW_WARNING_THRESHOLD:
            slope_offset = SFP_CALIB_TX_POWER_SLOPE_OFFSET;
            const_offset = SFP_CALIB_TX_POWER_CONST_OFFSET;
            break;

        case SDI_MEDIA_RX_PWR_HIGH_ALARM_THRESHOLD:
        case SDI_MEDIA_RX_PWR_LOW_ALARM_THRESHOLD:
        case SDI_MEDIA_RX_PWR_HIGH_WARNING_THRESHOLD:
        case SDI_MEDIA_RX_PWR_LOW_WARNING_THRESHOLD:
            const_offset = SFP_CALIB_RX_POWER_CONST_START_OFFSET;
            break;

        default:
            return SDI_DEVICE_ERRCODE(EINVAL);
    }

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        rc = sdi_smbus_read_word(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                offset, (uint16_t *)threshold_buf, SDI_I2C_FLAG_NONE);
        if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("smbus read failed for threshold values with rc : %d", rc);
            break;
        }

        if(calib_info.type == SFP_CALIB_TYPE_EXTERNAL) {
            if(const_offset == SFP_CALIB_RX_POWER_CONST_START_OFFSET) {
                rc = sdi_smbus_read_multi_byte(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR,
                        const_offset, rx_power_data, EXT_CAL_RX_POWER_LEN, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("smbus read failed for rx power calibration constant with rc : %d",
                                          rc);
                    break;
                }
            } else {
                rc = sdi_smbus_read_word(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR, slope_offset,
                        (uint16_t *)&calib_info.slope, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("smbus read failed for temp slope constant rc : %d",rc);
                    break;

                }

                rc = sdi_smbus_read_word(sfp_device->bus_hdl, SFP_DIAG_MNTR_I2C_ADDR, const_offset,
                        &calib_info.offset, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("smbus read failed for temp offset constant rc : %d", rc);
                    break;
                }
            }
        }
    } while(0);

    sdi_sfp_module_deselect(sfp_priv_data);

    if(rc == STD_ERR_OK) {
        if( (threshold_type == SDI_MEDIA_TEMP_HIGH_ALARM_THRESHOLD) ||
            (threshold_type == SDI_MEDIA_TEMP_LOW_ALARM_THRESHOLD) ||
            (threshold_type == SDI_MEDIA_TEMP_HIGH_WARNING_THRESHOLD) ||
            (threshold_type == SDI_MEDIA_TEMP_LOW_WARNING_THRESHOLD) ) {
            *value = convert_sfp_temp(threshold_buf, &calib_info);
        } else if( (threshold_type == SDI_MEDIA_VOLT_HIGH_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_VOLT_LOW_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_VOLT_HIGH_WARNING_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_VOLT_LOW_WARNING_THRESHOLD) ) {
            *value = convert_sfp_volt(threshold_buf, &calib_info);
        } else if( (threshold_type == SDI_MEDIA_RX_PWR_HIGH_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_RX_PWR_LOW_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_RX_PWR_HIGH_WARNING_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_RX_PWR_LOW_WARNING_THRESHOLD) ) {
            rx_pwr_calib_info.type = calib_info.type;
            if(calib_info.type == SFP_CALIB_TYPE_EXTERNAL) {
                rx_power_data_ptr = rx_power_data;
                memcpy(rx_pwr_calib_info.rx_power_const_4, rx_power_data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
                rx_power_data_ptr = rx_power_data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
                memcpy(rx_pwr_calib_info.rx_power_const_3, rx_power_data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
                rx_power_data_ptr = rx_power_data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
                memcpy(rx_pwr_calib_info.rx_power_const_2, rx_power_data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
                rx_power_data_ptr = rx_power_data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
                memcpy(rx_pwr_calib_info.rx_power_const_1, rx_power_data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
                rx_power_data_ptr = rx_power_data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
                memcpy(rx_pwr_calib_info.rx_power_const_0, rx_power_data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
            }
            *value = convert_sfp_rx_power(threshold_buf, &rx_pwr_calib_info);
        } else if( (threshold_type == SDI_MEDIA_TX_BIAS_HIGH_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_TX_BIAS_LOW_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_TX_BIAS_HIGH_WARNING_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_TX_BIAS_LOW_WARNING_THRESHOLD) ) {
            *value = convert_sfp_tx_bias_current(threshold_buf, &calib_info);
        } else if( (threshold_type == SDI_MEDIA_TX_PWR_HIGH_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_TX_PWR_LOW_ALARM_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_TX_PWR_HIGH_WARNING_THRESHOLD) ||
                   (threshold_type == SDI_MEDIA_TX_PWR_LOW_WARNING_THRESHOLD) ) {
            *value = convert_sfp_tx_power(threshold_buf, &calib_info);
        }
    }
    return rc;
}

/**
 * Read the threshold values for module monitors like temperature and voltage
 * resource_hdl[in]     - Handle of the resource
 * threshold_type[in]   - type of threshold
 * value[out]           - threshold value
 * return               - standard t_std_error
 */
t_std_error sdi_sfp_module_monitor_threshold_get(sdi_resource_hdl_t resource_hdl,
                                                 uint_t threshold_type, uint_t *value)
{
    return STD_ERR_UNIMPLEMENTED;
}

/**
 * Read the threshold values for channel monitors like rx-ower and tx-bias
 * resource_hdl[in]     - Handle of the resource
 * threshold_type[in]   - type of threshold
 * value[out]           - threshold value
 * return               - standard t_std_error
 */
t_std_error sdi_sfp_channel_monitor_threshold_get(sdi_resource_hdl_t resource_hdl,
                                                  uint_t threshold_type, uint_t *value)
{
    return STD_ERR_UNIMPLEMENTED;
}

/**
 * Read the received output power and calibration constant for output rx_power
 *
 * bus_hdl[in] - Handle of the i2c bus
 * val_offset[in] - Register offset for rx output power
 * buf[out] - buffer for storing rx output power
 * rx_pwr_calib_info[out] - structure contains calibration related information
 * for rx power
 * vs_offset[in] - rx power calibration constant start offset
 *
 * return - standard t_std_error
 */
t_std_error sdi_sfp_rx_power_value_read(sdi_i2c_bus_hdl_t bus_hdl, uint_t val_offset,
                                        uint16_t *buf, sfp_rx_power_calib_info_t *rx_pwr_calib_info,
                                        uint_t vs_offset)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t data[EXT_CAL_RX_POWER_LEN] = { 0 };
    uint8_t *data_ptr = NULL;

    STD_ASSERT(bus_hdl != NULL);
    STD_ASSERT(rx_pwr_calib_info != NULL);
    STD_ASSERT(buf != NULL);

    rc = sdi_smbus_read_word(bus_hdl, SFP_DIAG_MNTR_I2C_ADDR, val_offset, buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("smbus read failed for temperature with rc : %d", rc);
        return rc;
    }

    if(rx_pwr_calib_info->type == SFP_CALIB_TYPE_EXTERNAL) {
        rc = sdi_smbus_read_multi_byte(bus_hdl, SFP_DIAG_MNTR_I2C_ADDR, vs_offset, data,
                                       EXT_CAL_RX_POWER_LEN, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("smbus read failed for rx power calibration constant with rc : %d", rc);
            return rc;
        }
        data_ptr = data;
        memcpy(rx_pwr_calib_info->rx_power_const_4, data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
        data_ptr = data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
        memcpy(rx_pwr_calib_info->rx_power_const_3, data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
        data_ptr = data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
        memcpy(rx_pwr_calib_info->rx_power_const_2, data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
        data_ptr = data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
        memcpy(rx_pwr_calib_info->rx_power_const_1, data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
        data_ptr = data_ptr + EXT_CAL_RX_POWER_DATA_LEN;
        memcpy(rx_pwr_calib_info->rx_power_const_0, data_ptr, EXT_CAL_RX_POWER_DATA_LEN);
    }
    return rc;
}

/**
 * Read the requested module monitor values(temperature/votage) from offset specified by
 * val_offset and calibration constants for external calibration types
 *
 * bus_hdl[in] - Handle of the i2c bus
 * val_offset[in] - Register offset for module monitors(temperature/voltage)
 * buf[out] - buffer for storing module monitor values (temperature/voltage)
 * calib_info[out] - structure contains calibration related information
 * vs_offset[in] - Values Slope Offset for external calibration type
 * vc_offset[in] - Values Constant offset for external calibration type
 *
 * return - standard t_std_error
 */
t_std_error sdi_sfp_module_monitor_value_read(sdi_i2c_bus_hdl_t bus_hdl, uint_t val_offset,
                                              uint16_t *buf, sfp_calib_info_t *calib_info,
                                              uint_t vs_offset, uint_t vc_offset)
{
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(bus_hdl != NULL);
    STD_ASSERT(calib_info != NULL);
    STD_ASSERT(buf != NULL);

    rc = sdi_smbus_read_word(bus_hdl, SFP_DIAG_MNTR_I2C_ADDR, val_offset, buf,
            SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK){
        SDI_DEVICE_ERRMSG_LOG("smbus read failed for temperature with rc : %d", rc);
        return rc;
    }

    if(calib_info->type == SFP_CALIB_TYPE_EXTERNAL) {
        rc = sdi_smbus_read_word(bus_hdl, SFP_DIAG_MNTR_I2C_ADDR, vs_offset,
                (uint16_t *)&calib_info->slope, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("smbus read failed for temp slope constant rc : %d",rc);
            return rc;
        }

        rc = sdi_smbus_read_word(bus_hdl, SFP_DIAG_MNTR_I2C_ADDR, vc_offset,
                &calib_info->offset, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("smbus read failed for temp offset constant rc : %d", rc);
            return rc;
        }
    }
    return rc;
}

/**
 * Debug api to retrieve module monitors assoicated with the specified SFP
 * resource_hdl[in] - Handle of the resource
 * monitor[in]      - monitor which needs to be retrieved
 * value[out]     - Value of the monitor
 * return           - t_std_error
 */
t_std_error sdi_sfp_module_monitor_get (sdi_resource_hdl_t resource_hdl,
                                        sdi_media_module_monitor_t monitor, float *value)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t buf[2] = { 0 };
    uint_t diag_mon_value = 0;
    sfp_calib_info_t calib_info = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);


    /* Check whether diag monitoring is supported on this device or not */
    rc = sdi_sfp_parameter_get(resource_hdl, SDI_MEDIA_DIAG_MON_TYPE, &diag_mon_value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Getting of diag monitoring value is failed for %s rc : %d",
                              sfp_device->alias, rc);
        return rc;
    }

    if( (STD_BIT_TEST(diag_mon_value, SFP_DDM_SUPPORT_BIT_OFFSET) == 0) ) {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    /* Get the calibration type */
    if( (STD_BIT_TEST(diag_mon_value, SFP_CALIB_TYPE_EXTERNAL_BIT_OFFSET) != 0) ) {
        calib_info.type = SFP_CALIB_TYPE_EXTERNAL;
    } else  if( (STD_BIT_TEST(diag_mon_value, SFP_CALIB_TYPE_INTERNAL_BIT_OFFSET) != 0) ) {
        calib_info.type = SFP_CALIB_TYPE_INTERNAL;
    } else {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        switch (monitor)
        {
            case SDI_MEDIA_TEMP:
                rc = sdi_sfp_module_monitor_value_read(sfp_device->bus_hdl, SFP_TEMPERATURE_OFFSET,
                                                       (uint16_t *)buf, &calib_info, SFP_CALIB_TEMP_SLOPE_OFFSET,
                                                       SFP_CALIB_TEMP_CONST_OFFSET);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("module monitor value read failed for temperature with rc : %d on %s",
                                          rc, sfp_device->alias);
                }
                break;

            case SDI_MEDIA_VOLT:
                rc = sdi_sfp_module_monitor_value_read(sfp_device->bus_hdl, SFP_VOLTAGE_OFFSET,
                                                       (uint16_t *)buf, &calib_info, SFP_CALIB_VOLT_SLOPE_OFFSET,
                                                       SFP_CALIB_VOLT_CONST_OFFSET);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("module monitor value read failed for voltage with rc : %d on %s",
                            rc, sfp_device->alias);
                }
                break;

            default:
                rc = SDI_DEVICE_ERRCODE(EINVAL);
        }
    } while(0);

    sdi_sfp_module_deselect(sfp_priv_data);

    if(rc == STD_ERR_OK) {
        if(monitor == SDI_MEDIA_TEMP) {
            *value = convert_sfp_temp(buf, &calib_info);
        } else if(monitor == SDI_MEDIA_VOLT) {
            *value = convert_sfp_volt(buf, &calib_info);
        }
    }
    return rc;
}

/**
 * Retrieve channel monitors assoicated with the specified SFP
 * resource_hdl[in] - Handle of the resource
 * channel[in]      - channel whose monitor has to be retreived
 * monitor[in]      - monitor which needs to be retrieved
 * value[out]     - Value of the monitor
 * return           - t_std_error
 */
t_std_error sdi_sfp_channel_monitor_get (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                         sdi_media_channel_monitor_t monitor, float *value)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t buf[2] = { 0 };
    uint_t diag_mon_value = 0;
    sfp_calib_info_t calib_info = { 0 };
    sfp_rx_power_calib_info_t rx_power_calib_info = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    if(channel != SDI_SFP_CHANNEL_NUM) {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    /* Check whether diag monitoring is supported on this device or not */
    rc = sdi_sfp_parameter_get(resource_hdl, SDI_MEDIA_DIAG_MON_TYPE, &diag_mon_value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Getting of diag monitoring value is failed for %s rc : %d",
                              sfp_device->alias, rc);
        return rc;
    }

    if( (STD_BIT_TEST(diag_mon_value, SFP_DDM_SUPPORT_BIT_OFFSET) == 0) ) {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    /* Get the calibration type */
    if( (STD_BIT_TEST(diag_mon_value, SFP_CALIB_TYPE_EXTERNAL_BIT_OFFSET) != 0) ) {
        calib_info.type = SFP_CALIB_TYPE_EXTERNAL;
    } else  if( (STD_BIT_TEST(diag_mon_value, SFP_CALIB_TYPE_INTERNAL_BIT_OFFSET) != 0) ) {
        calib_info.type = SFP_CALIB_TYPE_INTERNAL;
    } else {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        switch (monitor)
        {
            case SDI_MEDIA_INTERNAL_RX_POWER_MONITOR:
                rx_power_calib_info.type = calib_info.type;
                rc = sdi_sfp_rx_power_value_read(sfp_device->bus_hdl, SFP_RX_INPUT_POWER_OFFSET,
                                                 (uint16_t *)buf, &rx_power_calib_info,
                                                 SFP_CALIB_RX_POWER_CONST_START_OFFSET);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("smbus read failed for rx power with rc : %d", rc);
                }
                break;

            case SDI_MEDIA_INTERNAL_TX_BIAS_CURRENT:
                rc = sdi_sfp_module_monitor_value_read(sfp_device->bus_hdl, SFP_TX_BIAS_CURRENT_OFFSET,
                                                       (uint16_t *)buf, &calib_info, SFP_CALIB_TX_BIAS_SLOPE_OFFSET,
                                                       SFP_CALIB_TX_BIAS_CONST_OFFSET);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("channel monitor value read failed for tx bias current with rc : %d on %s",
                                           rc, sfp_device->alias);
                }
                break;

            case SDI_MEDIA_INTERNAL_TX_OUTPUT_POWER:
                rc = sdi_sfp_module_monitor_value_read(sfp_device->bus_hdl, SFP_TX_OUTPUT_POWER_OFFSET,
                                                       (uint16_t *)buf, &calib_info, SFP_CALIB_TX_POWER_SLOPE_OFFSET,
                                                       SFP_CALIB_TX_POWER_CONST_OFFSET);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("channel monitor value read failed for tx output power with rc : %d on %s",
                                          rc, sfp_device->alias);
                }
                break;

            default:
                rc = SDI_DEVICE_ERRCODE(EINVAL);
        }
    } while(0);
    sdi_sfp_module_deselect(sfp_priv_data);

    if( rc == STD_ERR_OK) {
        if(monitor == SDI_MEDIA_INTERNAL_RX_POWER_MONITOR) {
            *value = convert_sfp_rx_power(buf, &rx_power_calib_info);
        } else if(monitor == SDI_MEDIA_INTERNAL_TX_BIAS_CURRENT) {
            *value = convert_sfp_tx_bias_current(buf, &calib_info);
        }else if(monitor == SDI_MEDIA_INTERNAL_TX_OUTPUT_POWER) {
            *value = convert_sfp_tx_power(buf, &calib_info);
        }
    }
    return rc;
}

/**
 * Get the inforamtion of whether optional features supported or not on a given
 * module
 * resource_hdl[in] - Handle of the resource
 * feature_support[out] - feature support flags. Flag will be set to "true" if
 * feature is supported else "false"
 * return standard t_std_error
 */
t_std_error sdi_sfp_feature_support_status_get (sdi_resource_hdl_t resource_hdl,
                                                sdi_media_supported_feature_t *feature_support)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(feature_support != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        rc = sdi_is_alarm_flags_supported(sfp_device,
                                          &feature_support->sfp_features.alarm_support_status);
        if (rc != STD_ERR_OK){
            break;
        }

        rc = sdi_is_diag_monitoring_supported(sfp_device,
                                              &feature_support->sfp_features.diag_mntr_support_status);
        if (rc != STD_ERR_OK){
            break;
        }

        rc = sdi_is_rate_select_supported(sfp_device,
                                          &feature_support->sfp_features.rate_select_status);
        if (rc != STD_ERR_OK){
            break;
        }
    }while(0);

    sdi_sfp_module_deselect(sfp_priv_data);

    return rc;
}

/**
 * Raw read api for media eeprom
 * resource_hdl[in] - Handle of the resource
 * offset[in]       - offset from which to read
 * data[out]      - Data will be filled after read
 * data_len[in]     - length of the data to be read
 * return           - t_std_error
 */
t_std_error sdi_sfp_read (sdi_resource_hdl_t resource_hdl, uint_t offset,
                          uint8_t *data, size_t data_len)
{
    return STD_ERR_UNIMPLEMENTED;
}

/**
 * Raw write api for media eeprom
 * resource_hdl[in] - Handle of the resource
 * offset[in]       - offset from which to write
 * data[in]         - input buffer which contains the data to be written
 * data_len[in]     - length of the data to be written
 * return           - t_std_error
 */
t_std_error sdi_sfp_write (sdi_resource_hdl_t resource_hdl, uint_t offset,
                           uint8_t *data, size_t data_len)
{
    return STD_ERR_UNIMPLEMENTED;
}

/**
 * Raw write api for media copper sfp
 * resource_hdl[in] - Handle of the resource
 * offset[in]       - offset from which to write
 * data[in]         - input buffer which contains the data to be written
 * return           - t_std_error
 */

t_std_error sdi_sfp_phy_write(sdi_device_hdl_t sfp_device,int reg_offset,uint16_t regData)
{
    t_std_error rc = STD_ERR_OK;

    SDI_DEVICE_TRACEMSG_LOG("SMBUS Write  #offset %x and #data %x\n",reg_offset,regData);
    rc = sdi_smbus_write_word(sfp_device->bus_hdl,SFP_PHY_I2C_ADDR,reg_offset,
            regData,SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus write failed at addr : %x reg : %x for %s rc : %d",
                sfp_device->addr, reg_offset, sfp_device->alias, rc);
    }
    return rc;
}

/**
 * Raw read api for media eeprom
 * resource_hdl[in] - Handle of the resource
 * offset[in]       - offset from which to read
 * data[out]      - Data will be filled after read
 */

t_std_error sdi_sfp_phy_read(sdi_device_hdl_t sfp_device,int reg_offset,uint16_t *regData)
{
    t_std_error rc = STD_ERR_OK;

    rc = sdi_smbus_read_word(sfp_device->bus_hdl,SFP_PHY_I2C_ADDR,reg_offset,
            (uint16_t *)regData,SDI_I2C_FLAG_NONE);
    SDI_DEVICE_TRACEMSG_LOG("SMBUS read #offset %x and #data %x\n",reg_offset,*regData);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus read failed at addr : %x reg : %x for %s rc : %d",
                SFP_PHY_I2C_ADDR, reg_offset, sfp_device->alias, rc);
    }

    return rc;
}

/**
 * media PHY reset in SFP
 * resource_hdl[in] - handle of the resource
 * return           - t_std_error
 */

t_std_error sdi_media_phy_sw_reset(sdi_device_hdl_t sfp_device)
{
    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;

    sdi_sfp_phy_read(sfp_device,SFP_COPPER_CTRL_REG,&regData);
    regData |= SFP_COPPER_CTRL_RESET;

    rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_CTRL_REG,regData);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("sfp smbus Write failue failed at addr : %x reg : %x for %s rc : %d",
                sfp_device->addr, SFP_COPPER_CTRL_REG, sfp_device->alias, rc);
    }

    return rc;
}

/**
 * Disable/Enable Auto neg for media PHY in SFP
 * resource_hdl[in] - handle of the resource
 * enable[in]       - "false" to disable and "true" to enable
 * return           - t_std_error
 */
t_std_error sdi_sfp_phy_autoneg_set (sdi_resource_hdl_t resource_hdl,bool enable)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;

    STD_ASSERT(resource_hdl != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {

        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_CTRL_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        if (enable) {
            regData |= SFP_COPPER_CTRL_AE;
        }
        else {
            regData &= ~SFP_COPPER_CTRL_AE;
        }

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_CTRL_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

        /* if Power Down is set. software reset resets
         * the power down bit resulting in link up on the
         * partner.
         * */
        if (!(regData & SFP_COPPER_CTRL_PD))
            rc = sdi_media_phy_sw_reset(sfp_device);

        if (rc != STD_ERR_OK){
            break;
        }

    }while (0);
    sdi_sfp_module_deselect(sfp_priv_data);

    return rc;
}


/**
 * SFP Phy mode set (SGMII/GMII/MII ..)
 * resource_hdl[in] - handle of the resource
 * mode[in]         - mode of the interface
 * return           - t_std_error
 */

t_std_error sdi_sfp_phy_mode_set (sdi_resource_hdl_t resource_hdl,sdi_media_mode_t mode)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;

    STD_ASSERT(resource_hdl != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }
    /* 100 usec delay before perform a read */

    std_usleep(SFP_PHY_DELAY);

    do {
        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_EXT_STATUS_2_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        /* Details about this magic are not
         * given on the doc. Using hardcoded value. */
        if(mode == SDI_MEDIA_MODE_SGMII)
            regData = PHY_SGMII_MODE;

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_EXT_STATUS_2_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

        /* if Power Down is set. software reset resets
         * the power down bit resulting in link up on the
         * partner.
         * */
        if (!(regData & SFP_COPPER_CTRL_PD))
            rc = sdi_media_phy_sw_reset(sfp_device);

        if (rc != STD_ERR_OK){
            break;
        }

    } while (0);
    sdi_sfp_module_deselect(sfp_priv_data);

    return rc;
}

/**
 * SFP Phy speed set (1G/100M/10M)
 * resource_hdl[in] - handle of the resource
 * speed[in]         - speed of the interface
 * return           - t_std_error
 */

t_std_error sdi_sfp_phy_speed_set(sdi_resource_hdl_t resource_hdl,sdi_media_speed_t speed)
{

    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint16_t regData =0;
    uint_t  abilGB = 0, abil = 0;


    STD_ASSERT(resource_hdl != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_sfp_module_select(sfp_device);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do{
        if (speed == SDI_MEDIA_SPEED_1G)
            abilGB |= (SFP_COPPER_GB_CTRL_ADV_1000FD);

        if (speed == SDI_MEDIA_SPEED_100M)
            abil |= (SFP_COPPER_ANA_FD_100 | SFP_COPPER_ANA_FD_10);

        if (speed == SDI_MEDIA_SPEED_10M)
            abil |= (SFP_COPPER_ANA_FD_10);

        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_ANA_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        regData  &= ~(SFP_COPPER_ANA_HD_10 |
                SFP_COPPER_ANA_FD_10 |
                SFP_COPPER_ANA_HD_100 |
                SFP_COPPER_ANA_FD_100 );

        regData |= abil;

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_ANA_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

        rc = sdi_sfp_phy_read(sfp_device,SFP_COPPER_GB_CTRL_REG,&regData);
        if (rc != STD_ERR_OK){
            break;
        }

        regData &= ~(SFP_COPPER_GB_CTRL_ADV_1000FD |
                SFP_COPPER_GB_CTRL_ADV_1000HD);

        regData |= abilGB;

        rc = sdi_sfp_phy_write(sfp_device,SFP_COPPER_GB_CTRL_REG,regData);
        if (rc != STD_ERR_OK){
            break;
        }

    }while(0);

    sdi_sfp_module_deselect(sfp_priv_data);

    return rc;
}
