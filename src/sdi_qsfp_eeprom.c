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
 * filename: sdi_qsfp_eeprom.c
 */


/******************************************************************************
 * sdi_qsfp_eeprom.c
 * Implements the QSFP eeprom related APIs
 *****************************************************************************/
#include "sdi_resource_internal.h"
#include "sdi_device_common.h"
#include "sdi_pin_group_bus_framework.h"
#include "sdi_pin_group_bus_api.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_media.h"
#include "sdi_qsfp.h"
#include "sdi_qsfp_reg.h"
#include "sdi_media_internal.h"
#include "std_error_codes.h"
#include "std_assert.h"
#include "std_time_tools.h"
#include "std_bit_ops.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QSFP_TX_LOS_FLAG(x) ( QSFP_TX_LOS_BIT_OFFSET << (x) )
#define QSFP_RX_LOS_FLAG(x) ( QSFP_RX_LOS_BIT_OFFSET << (x) )

#define QSFP_TX_ENABLE_DELAY (400 * 1000)
#define QSFP_TX_DISABLE_DELAY (100 * 1000)

/* QSFP channel numbers */
enum {
    SDI_QSFP_CHANNEL_ONE = 0,
    SDI_QSFP_CHANNEL_TWO,
    SDI_QSFP_CHANNEL_THREE,
    SDI_QSFP_CHANNEL_FOUR,
};

 /*QSFP parameter sizes */
enum {
    SDI_QSFP_BYTE_SIZE = 1,
    SDI_QSFP_WORD_SIZE = 2,
    SDI_QSFP_DOUBLE_WORD_SIZE = 4,
    SDI_QSFP_QUAD_WORD_SIZE = 8,
};

/* QSFP pages */
enum {
    SDI_QSFP_PAGE_00 = 0,
    SDI_QSFP_PAGE_01,
    SDI_QSFP_PAGE_02,
    SDI_QSFP_PAGE_03
};

enum {
    SDI_QSFP_RX_PWR_OMA = 0,
    SDI_QSFP_RX_PWR_AVG
};

/* qsfp register information structure */
typedef struct sdi_qsfp_reg_info {
    uint_t offset; /* register offset */
    uint_t size; /* register size */
} sdi_qsfp_reg_info_t;

/* parameter register information strucutre. Parameters should be defined in the
 * same order of sdi_media_param_type_t */
static sdi_qsfp_reg_info_t param_reg_info[] = {
    { QSFP_WAVELENGTH_OFFSET, SDI_QSFP_WORD_SIZE }, /* for SDI_MEDIA_WAVELENGTH */
    { QSFP_WAVELENGTH_TOLERANCE_OFFSET, SDI_QSFP_WORD_SIZE }, /* for SDI_MEDIA_WAVELENGTH_TOLERANCE */
    { QSFP_MAX_CASE_TEMP_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_MAX_CASE_TEMP */
    { QSFP_CC_BASE_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_CC_BASE */
    { QSFP_CC_EXT_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_CC_EXT */
    { QSFP_CONNECTOR_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_CONNECTOR */
    { QSFP_ENCODING_TYPE_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_ENCODING_TYPE */
    { QSFP_NM_BITRATE_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_NM_BITRATE */
    { QSFP_IDENTIFIER_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_IDENTIFIER */
    { QSFP_EXT_IDENTIFIER_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_EXT_IDENTIFIER */
    { QSFP_LENGTH_SMF_KM_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_SMF_KM */
    { QSFP_LENGTH_OM1_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_OM1 */
    { QSFP_LENGTH_OM2_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_OM2 */
    { QSFP_LENGTH_OM3_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_LENGTH_OM3 */
    { QSFP_LENGTH_CABLE_ASSEMBLY_OFFSET, SDI_QSFP_BYTE_SIZE  }, /* for SDI_MEDIA_LENGTH_CABLE_ASSEMBLY */
    { 0, 0}, /* for SDI_MEDIA_LENGTH_SMF, not supported on QSFP */
    { QSFP_OPTIONS1_OFFSET, SDI_QSFP_DOUBLE_WORD_SIZE }, /* for SDI_MEDIA_OPTIONS */
    { QSFP_ENHANCED_OPTIONS_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_ENHANCED_OPTIONS */
    { QSFP_DIAG_MON_TYPE_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_DIAG_MON_TYPE */
    { QSFP_DEVICE_TECH_OFFSET, SDI_QSFP_BYTE_SIZE }, /* for SDI_MEDIA_DEVICE_TECH */
};

/* vendor register information structure. Parameters in this structure should be
 * defined in the same order of sdi_media_vendor_info_type_t */
static sdi_qsfp_reg_info_t vendor_reg_info[] = {
    { QSFP_VENDOR_NAME_OFFSET, SDI_MEDIA_MAX_VENDOR_NAME_LEN }, /* for SDI_MEDIA_VENDOR_NAME */
    { QSFP_VENDOR_OUI_OFFSET, SDI_MEDIA_MAX_VENDOR_OUI_LEN }, /* for SDI_MEDIA_VENDOR_OUI */
    { QSFP_VENDOR_SN_OFFSET, SDI_MEDIA_MAX_VENDOR_SERIAL_NUMBER_LEN }, /* for SDI_MEDIA_VENDOR_SN */
    { QSFP_VENDOR_DATE_OFFSET, SDI_MEDIA_MAX_VENDOR_DATE_LEN }, /* for SDI_MEDIA_VENDOR_DATE */
    { QSFP_VENDOR_PN_OFFSET, SDI_MEDIA_MAX_VENDOR_PART_NUMBER_LEN }, /* for SDI_MEDIA_VENDOR_PN */
    { QSFP_VENDOR_REVISION_OFFSET, SDI_MEDIA_MAX_VENDOR_REVISION_LEN } /* for SDI_MEDIA_VENDOR_REVISION */
};

/* threshold value register information structure. Parameters in this structure
 * should be defined in the same order of sdi_media_threshold_type_t */
static sdi_qsfp_reg_info_t threshold_reg_info[] = {
    /* for SDI_MEDIA_TEMP_HIGH_ALARM_THRESHOLD */
    { QSFP_TEMP_HIGH_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TEMP_LOW_ALARM_THRESHOLD */
    { QSFP_TEMP_LOW_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TEMP_HIGH_WARNING_THRESHOLD */
    { QSFP_TEMP_HIGH_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TEMP_LOW_WARNING_THRESHOLD */
    { QSFP_TEMP_LOW_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_HIGH_ALARM_THRESHOLD */
    { QSFP_VOLT_HIGH_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_LOW_ALARM_THRESHOLD */
    { QSFP_VOLT_LOW_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_HIGH_WARNING_THRESHOLD */
    { QSFP_VOLT_HIGH_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_VOLT_LOW_WARNING_THRESHOLD */
    { QSFP_VOLT_LOW_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_HIGH_ALARM_THRESHOLD */
    { QSFP_RX_PWR_HIGH_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_LOW_ALARM_THRESHOLD */
    { QSFP_RX_PWR_LOW_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_HIGH_WARNING_THRESHOLD */
    { QSFP_RX_PWR_HIGH_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_RX_PWR_LOW_WARNING_THRESHOLD */
    { QSFP_RX_PWR_LOW_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_HIGH_ALARM_THRESHOLD */
    { QSFP_TX_BIAS_HIGH_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_LOW_ALARM_THRESHOLD */
    { QSFP_TX_BIAS_LOW_ALARM_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_HIGH_WARNING_THRESHOLD */
    { QSFP_TX_BIAS_HIGH_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_BIAS_LOW_WARNING_THRESHOLD */
    { QSFP_TX_BIAS_LOW_WARNING_THRESHOLD_OFFSET, SDI_QSFP_WORD_SIZE },
    /* for SDI_MEDIA_TX_PWR_HIGH_ALARM_THRESHOLD */
    { 0, 0 },
    /* for SDI_MEDIA_TX_PWR_LOW_ALARM_THRESHOLD */
    { 0, 0 },
    /* for SDI_MEDIA_TX_PWR_HIGH_WARNING_THRESHOLD */
    { 0, 0 },
    /* for SDI_MEDIA_TX_PWR_LOW_WARNING_THRESHOLD */
    { 0, 0 },
};

/* This function validates the channel number */
static inline bool sdi_qsfp_validate_channel (uint_t channel)
{
    return( ( (channel >= SDI_QSFP_CHANNEL_ONE) && (channel <= SDI_QSFP_CHANNEL_FOUR) ) );
}

/* This function checks whether paging is supported or not on a QSFP. If paging
 * is supported then selects requested page. */
static inline t_std_error sdi_qsfp_page_select (sdi_device_hdl_t qsfp_device,
                                                uint_t page_num)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(qsfp_device != NULL);

    rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                             QSFP_STATUS_INDICATOR_OFFSET, &buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK){
        SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d ", qsfp_device->addr);
        return rc;
    }

    if( (STD_BIT_TEST(buf, QSFP_FLAT_MEM_BIT_OFFSET)) != 0 ) {
        return SDI_DEVICE_ERRCODE(ENOTSUP);
    }

    rc = sdi_smbus_write_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                              QSFP_PAGE_SELECT_BYTE_OFFSET, page_num, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("qsfp smbus write failed at addr : %d ", qsfp_device->addr);
    }
    return rc;
}

/* This function checks whether tx_disable implemented for this module */
static inline t_std_error sdi_is_tx_control_supported(sdi_device_hdl_t qsfp_device,
                                                      bool *support_status)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(qsfp_device != NULL);
    STD_ASSERT(support_status != NULL);

    *support_status = false;

    rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                             QSFP_OPTIONS4_OFFSET, &buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK) {
        return rc;
    }

    if( (STD_BIT_TEST(buf, QSFP_TX_DISABLE_BIT_OFFSET) != 0) ) {
        *support_status = true;
    } else {
        *support_status = false;
    }
    return rc;
}

/* This function checks whether paging is supported or not on a given module */
static inline t_std_error sdi_is_paging_supported(sdi_device_hdl_t qsfp_device,
                                                  bool *support_status)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(qsfp_device != NULL);
    STD_ASSERT(support_status != NULL);

    *support_status = false;

    rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                             QSFP_STATUS_INDICATOR_OFFSET, &buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK){
        SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d ", qsfp_device->addr);
        return rc;
    }

    if( (STD_BIT_TEST(buf, QSFP_FLAT_MEM_BIT_OFFSET)) != 0 ) {
        *support_status = false;
    } else {
        *support_status = true;
    }
    return rc;
}

/* This function checks whether rate select supported or not for a given module */
static inline t_std_error sdi_is_rate_select_supported(sdi_device_hdl_t qsfp_device,
                                                       bool *support_status)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(qsfp_device != NULL);
    STD_ASSERT(support_status != NULL);

    *support_status = false;

    rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                             QSFP_OPTIONS4_OFFSET, &buf, SDI_I2C_FLAG_NONE);
    if (rc != STD_ERR_OK){
        SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d rc : %d",
                              qsfp_device->addr, rc);
        return rc;
    }

    if ( (STD_BIT_TEST(buf, QSFP_RATE_SELECT_BIT_OFFSET) != 0) ) {
        *support_status = true;
    } else {
        *support_status = false;
    }

    return rc;
}

/* Internally measured Module temperature are represented as a
 * 16-bit signed twos complement value in increments of 1/256
 * degrees Celsius */
static inline float convert_qsfp_temp(uint8_t *buf)
{
    int16_t temp = 0;
    float calib_temp = 0;
    bool is_negative = false;

    temp = ( (buf[0] << 8) | (buf[1]) );

    if( temp < 0 ){
        /* Negative value */
        temp = ((~temp) + 1);
        is_negative = true;
    }

    calib_temp = ((float)(temp & 0xFF) / 256.0) + ((temp & 0xFF00) >> 8);

    if(is_negative == true) {
        calib_temp = -calib_temp;
    }

    return calib_temp;
}

/* Internally measured Module supply voltage are represented as a
 * 16-bit unsigned integer with the voltage defined as the full 16
 * bit value with LSB equal to 100 uVolt, yielding a total
 * measurement range of 0 to +6.55 Volts. */
static inline float convert_qsfp_volt(uint8_t *buf)
{
   return ((float)((buf[0] << 8) | (buf[1]))/10000.0);
}

/* Represented as a 16 bit unsigned integer with the power defined
 * as the full 16 bit value (0 – 65535) with LSB equal to 0.1 uW, yielding a
 * total measurement range of 0 to 6.5535 mW (~-40 to +8.2 dBm). */
static inline float convert_qsfp_rx_power(uint8_t *buf)
{
        return ((float)((buf[0] << 8) | (buf[1]))/10000.0);
}

/* Measured TX bias current is in mA and are represented as a 16-bit unsigned
 * integer with the current defined as the full 16 bit value (0 – 65535) with
 * LSB equal to 2 uA, yielding  a total measurement range of 0 to 131 mA.*/
static inline float convert_qsfp_tx_bias(uint8_t *buf)
{
    return ((float)(((buf[0] << 8) | (buf[1])) * 2)/1000.0);
}

/**
 * Get the required module status of the specific qsfp
 * resource_hdl[in] - Handle of the resource
 * flags[in]        - flags for status that are of interest
 * status[out]    - returns the set of status flags which are asserted
 * return           - t_std_error
 */
t_std_error sdi_qsfp_module_monitor_status_get (sdi_resource_hdl_t resource_hdl,
                                                uint_t flags, uint_t *status)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t temp_status_buf = 0;
    uint8_t volt_status_buf = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        if( (flags) & ( (SDI_MEDIA_STATUS_TEMP_HIGH_ALARM)  |
                    (SDI_MEDIA_STATUS_TEMP_LOW_ALARM)   |
                    (SDI_MEDIA_STATUS_TEMP_HIGH_WARNING)|
                    (SDI_MEDIA_STATUS_TEMP_LOW_WARNING) ) ) {
            rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                    QSFP_TEMP_INTERRUPT_OFFSET, &temp_status_buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                        "rc : %d", qsfp_device->addr, rc);
                break;
            }
        }

        if( (flags) & ( (SDI_MEDIA_STATUS_VOLT_HIGH_ALARM)  |
                    (SDI_MEDIA_STATUS_VOLT_LOW_ALARM)   |
                    (SDI_MEDIA_STATUS_VOLT_HIGH_WARNING)|
                    (SDI_MEDIA_STATUS_VOLT_LOW_WARNING) ) ) {
            rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                    QSFP_VOLT_INTERRUPT_OFFSET, &volt_status_buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                        "rc : %d", qsfp_device->addr, rc);
            }
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if(rc == STD_ERR_OK) {
        if (temp_status_buf & QSFP_TEMP_HIGH_ALARM_FLAG){
            *status |= SDI_MEDIA_STATUS_TEMP_HIGH_ALARM;
        } else if (temp_status_buf & QSFP_TEMP_LOW_ALARM_FLAG){
            *status |= SDI_MEDIA_STATUS_TEMP_LOW_ALARM;
        }

        if (temp_status_buf & QSFP_TEMP_HIGH_WARNING_FLAG){
            *status |= SDI_MEDIA_STATUS_TEMP_HIGH_WARNING;
        } else if (temp_status_buf & QSFP_TEMP_LOW_WARNING_FLAG){
            *status |= SDI_MEDIA_STATUS_TEMP_LOW_WARNING;
        }

        if (volt_status_buf & QSFP_VOLT_HIGH_ALARM_FLAG){
            *status |= SDI_MEDIA_STATUS_VOLT_HIGH_ALARM;
        } else if (volt_status_buf & QSFP_VOLT_LOW_ALARM_FLAG){
            *status |= SDI_MEDIA_STATUS_VOLT_LOW_ALARM;
        }

        if (volt_status_buf & QSFP_VOLT_HIGH_WARNING_FLAG){
            *status |= SDI_MEDIA_STATUS_VOLT_HIGH_WARNING;
        } else if (volt_status_buf & QSFP_VOLT_LOW_WARNING_FLAG){
            *status |= SDI_MEDIA_STATUS_VOLT_LOW_WARNING;
        }
    }

    return rc;
}

/**
 * Get the required channel monitoring(rx_power, tx_bias alarm) status of the specific QSFP
 * resource_hdl[in] - Handle of the resource
 * channel[in]      - channel number that is of interest
 * flags[in]        - flags for channel status
 * status[out]      - returns the set of status flags which are asserted
 * return           - t_std_error
 */
t_std_error sdi_qsfp_channel_monitor_status_get (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                                 uint_t flags, uint_t *status)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t offset = 0;
    uint8_t rx_pwr_buf = 0;
    uint8_t tx_bias_buf = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    if (sdi_qsfp_validate_channel(channel) != true){
        return SDI_DEVICE_ERR_PARAM;
    }

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        if( ((flags) & ((SDI_MEDIA_RX_PWR_HIGH_ALARM) | (SDI_MEDIA_RX_PWR_LOW_ALARM) |
                        (SDI_MEDIA_RX_PWR_HIGH_WARNING) | (SDI_MEDIA_RX_PWR_LOW_WARNING))) != 0 )
        {
            if( (channel == SDI_QSFP_CHANNEL_ONE) || (channel == SDI_QSFP_CHANNEL_TWO) ) {
                offset = QSFP_RX12_POWER_INTERRUPT_OFFSET;
            } else {
                offset = QSFP_RX34_POWER_INTERRUPT_OFFSET;
            }
            rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                                     offset, &rx_pwr_buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                        "rc : %d", qsfp_device->addr, rc);
                break;
            }
        }

        if( ((flags) & ((SDI_MEDIA_TX_BIAS_HIGH_ALARM) | (SDI_MEDIA_TX_BIAS_LOW_ALARM) |
                        (SDI_MEDIA_TX_BIAS_HIGH_WARNING) | (SDI_MEDIA_TX_BIAS_LOW_WARNING))) != 0)
        {
            if( (channel == SDI_QSFP_CHANNEL_ONE) || (channel == SDI_QSFP_CHANNEL_TWO) ) {
                offset = QSFP_TX12_BIAS_INTERRUPT_OFFSET;
            } else {
                offset = QSFP_TX34_BIAS_INTERRUPT_OFFSET;
            }
            rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                                     offset, &tx_bias_buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                        "rc : %d", qsfp_device->addr, rc);
                break;
            }
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if(rc == STD_ERR_OK) {
        if( ((rx_pwr_buf) & ((QSFP_RX13_POWER_HIGH_ALARM_FLAG) |
                             (QSFP_RX24_POWER_HIGH_ALARM_FLAG))) != 0 ) {
            *status |= SDI_MEDIA_RX_PWR_HIGH_ALARM;
        } else if ( ((rx_pwr_buf) & ((QSFP_RX13_POWER_LOW_ALARM_FLAG) |
                                     (QSFP_RX24_POWER_LOW_ALARM_FLAG))) != 0 ) {
            *status |= SDI_MEDIA_RX_PWR_LOW_ALARM;
        }

        if( ((rx_pwr_buf) & ((QSFP_RX13_POWER_HIGH_WARNING_FLAG) |
                             (QSFP_RX24_POWER_HIGH_WARNING_FLAG))) != 0 ) {
            *status |= SDI_MEDIA_RX_PWR_HIGH_WARNING;
        } else if ( ((rx_pwr_buf) & ((QSFP_RX13_POWER_LOW_WARNING_FLAG) |
                                     (QSFP_RX24_POWER_LOW_WARNING_FLAG))) != 0 ) {
            *status |= SDI_MEDIA_RX_PWR_LOW_WARNING;
        }

        if( ((tx_bias_buf) & ((QSFP_TX13_BIAS_HIGH_ALARM_FLAG) |
                              (QSFP_TX24_BIAS_HIGH_ALARM_FLAG))) != 0 ) {
            *status |= SDI_MEDIA_TX_BIAS_HIGH_ALARM;
        } else if( ((tx_bias_buf) & ((QSFP_TX13_BIAS_LOW_ALARM_FLAG) |
                                     (QSFP_TX24_BIAS_LOW_ALARM_FLAG))) != 0 ) {
            *status |= SDI_MEDIA_TX_BIAS_LOW_ALARM;
        }

        if( ((tx_bias_buf) & ((QSFP_TX13_BIAS_HIGH_WARNING_FLAG) |
                              (QSFP_TX24_BIAS_HIGH_WARNING_FLAG))) != 0) {
            *status |= SDI_MEDIA_TX_BIAS_HIGH_WARNING;
        } else if( ((tx_bias_buf) & ((QSFP_TX13_BIAS_LOW_WARNING_FLAG) |
                                     (QSFP_TX24_BIAS_LOW_WARNING_FLAG))) != 0 ) {
            *status |= SDI_MEDIA_TX_BIAS_LOW_WARNING;
        }
    }
    return rc;
}

/**
 * Get the required channel status of the specific QSFP
 * resource_hdl[in] - Handle of the resource
 * channel[in]      - channel number that is of interest
 * flags[in]        - flags for channel status
 * status[out]    - returns the set of status flags which are asserted
 * return           - t_std_error
 */
t_std_error sdi_qsfp_channel_status_get (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                         uint_t flags, uint_t *status)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    if (sdi_qsfp_validate_channel(channel) != true){
        return SDI_DEVICE_ERR_PARAM;
    }

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        if( ((flags) & (SDI_MEDIA_STATUS_TXDISABLE)) ) {
            rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                    QSFP_TX_CONTROL_OFFSET, &buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                        "rc : %d", qsfp_device->addr, rc);
                break;
            }

            if ( (STD_BIT_TEST(buf, channel)) != 0 ) {
                *status |= SDI_MEDIA_STATUS_TXDISABLE;
            }
        }

        if( ((flags) & (SDI_MEDIA_STATUS_TXFAULT)) ) {
            rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                    QSFP_CHANNEL_TXFAULT_INDICATOR, &buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                        "rc : %d", qsfp_device->addr, rc);
                break;
            }

            if ( (STD_BIT_TEST(buf, channel)) != 0 ) {
                *status |= SDI_MEDIA_STATUS_TXFAULT;
            }
        }

        if( ( (flags) & ((SDI_MEDIA_STATUS_TXLOSS)|(SDI_MEDIA_STATUS_RXLOSS)) ) ) {
            rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                    QSFP_CHANNEL_LOS_INDICATOR, &buf, SDI_I2C_FLAG_NONE);
            if (rc != STD_ERR_OK){
                SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                        "rc : %d", qsfp_device->addr, rc);
                break;
            }

            if( (buf & QSFP_TX_LOS_FLAG(channel)) ) {
                *status |= SDI_MEDIA_STATUS_TXLOSS;
            }

            if( (buf & QSFP_RX_LOS_FLAG(channel)) ) {
                *status |= SDI_MEDIA_STATUS_RXLOSS;
            }
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);
    return rc;
}

/**
 * Disable/Enable the transmitter of the specific QSFP
 * resource_hdl[in] - handle of the resource
 * channel[in]      - channel number that is of interest
 * enable[in]       - "false" to disable and "true" to enable
 * return           - t_std_error
 */
t_std_error sdi_qsfp_tx_control (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                 bool enable)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;
    bool support_status = false;
    uint_t delay = 0;

    STD_ASSERT(resource_hdl != NULL);

    if (sdi_qsfp_validate_channel(channel) != true){
        return SDI_DEVICE_ERR_PARAM;
    }

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        rc = sdi_is_tx_control_supported(qsfp_device, &support_status);
        if (rc != STD_ERR_OK){
            break;
        }

        if(support_status == false) {
           rc = SDI_DEVICE_ERRCODE(EOPNOTSUPP);
           break;
        }

        rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                QSFP_TX_CONTROL_OFFSET, &buf, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d rc : %d",
                    qsfp_device->addr, rc);
            break;
        }

        if (enable == true){
            STD_BIT_CLEAR(buf, channel);
            /*After enabling transmitter on a particular channel, we need to
             * wait 400ms */
            delay = QSFP_TX_ENABLE_DELAY;
        } else {
            STD_BIT_SET(buf, channel);
            /*After disabling transmitter on a particular channel, we need to
             * wait 100ms */
            delay = QSFP_TX_DISABLE_DELAY;
        }

        rc = sdi_smbus_write_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                QSFP_TX_CONTROL_OFFSET, buf, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus write failed at addr : %d rc : %d",
                    qsfp_device->addr, rc);
        }
        std_usleep(delay);
    } while(0);
    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);
    return rc;
}

/**
 * Gets the transmitter status on the specific channel of a QSFP
 * resource_hdl[in] - handle of the resource
 * channel[in]      - channel number that is of interest
 * status[out]      - "true" if transmitter enabled, else "false"
 * return           - t_std_error
 */
t_std_error sdi_qsfp_tx_control_status_get(sdi_resource_hdl_t resource_hdl,
                                           uint_t channel, bool *status)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    if (sdi_qsfp_validate_channel(channel) != true){
        return SDI_DEVICE_ERR_PARAM;
    }

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        rc = sdi_smbus_read_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                QSFP_TX_CONTROL_OFFSET, &buf, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d rc : %d",
                    qsfp_device->addr, rc);
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if(rc == STD_ERR_OK) {
        /* If bit is set, transmitter is disabled on the channel and if bit is not
         * set, transmitter is enabled on the channel*/
        if ( (STD_BIT_TEST(buf, channel) == 0) ) {
            *status = true;
        } else {
            *status = false;
        }
    }

    return rc;
}

/**
 * Get the maximum speed that can be supported by a specific media resource
 * resource_hdl[in] - handle of the media resource
 * speed[out]     - speed of the media will be filled in this
 * return           - standard t_std_error
 */
t_std_error  sdi_qsfp_speed_get(sdi_resource_hdl_t resource_hdl,
                                sdi_media_speed_t *speed)
{
    STD_ASSERT(speed != NULL);

    *speed = SDI_MEDIA_SPEED_40G;

    return STD_ERR_OK;
}

/**
 * Check whether the specified QSFP is dell qualified
 * resource_hdl[in] - handle of the resource
 * status[out]    - true if optics is dell qualified else false
 * return           - t_std_error
 */
t_std_error sdi_qsfp_is_dell_qualified (sdi_resource_hdl_t resource_hdl,
                                        bool *status)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t magic_key[SDI_QSFP_MAGIC_KEY_SIZE] = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        rc = sdi_smbus_read_word(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                QSFP_DELL_PRODUCT_ID_OFFSET, (uint16_t *)magic_key, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d",
                    qsfp_device->addr);
            break;
        }
    } while(0);
    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if(rc == STD_ERR_OK) {
        if ( (magic_key[0] == SDI_QSFP_DELL_PRODUCT_ID_MAGIC0) &&
             (magic_key[1] == SDI_QSFP_DELL_PRODUCT_ID_MAGIC1) )
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
t_std_error sdi_qsfp_parameter_get(sdi_resource_hdl_t resource_hdl,
                                   sdi_media_param_type_t param, uint_t *value)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t byte_buf = 0;
    uint8_t word_buf[2] = { 0 };
    uint8_t buf[4] = { 0 };
    uint_t offset = 0;
    uint_t size = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    if(param == SDI_MEDIA_LENGTH_SMF) {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        offset = param_reg_info[param].offset;
        size =  param_reg_info[param].size;

        switch (size)
        {
            case SDI_QSFP_BYTE_SIZE:
                rc = sdi_smbus_read_byte(qsfp_device->bus_hdl,
                        qsfp_device->addr.i2c_addr, offset,
                        &byte_buf, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d reg : %d"
                            "rc : %d", qsfp_device->addr, offset, rc);
                }
                break;

            case SDI_QSFP_WORD_SIZE:
                rc = sdi_smbus_read_word(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                        offset, (uint16_t *)word_buf, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d reg : %d"
                            "rc : %d", qsfp_device->addr, offset, rc);
                }
                break;

            case SDI_QSFP_DOUBLE_WORD_SIZE:
                rc = sdi_smbus_read_multi_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                        offset, buf, SDI_QSFP_DOUBLE_WORD_SIZE, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK) {
                    SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d reg : %d"
                            "rc : %d", qsfp_device->addr, offset, rc);
                }
                break;

            default:
                rc = SDI_DEVICE_ERRCODE(EINVAL);
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if(rc == STD_ERR_OK) {
        if(size == SDI_QSFP_BYTE_SIZE) {
            if(param == SDI_MEDIA_DIAG_MON_TYPE) {
                if(STD_BIT_TEST(byte_buf, 3) != 0) {
                    *value = SDI_MEDIA_RX_PWR_AVG;
                } else {
                    *value = SDI_MEDIA_RX_PWR_OMA;
                }
            } else {
                *value = (uint_t)byte_buf;
            }
        } else if(size == SDI_QSFP_WORD_SIZE) {
            *value = ( (word_buf[0] << 8) | (word_buf[1]) );
            if(param == SDI_MEDIA_WAVELENGTH) {
                /* wavelength=value/20 in nm */
                *value = ( (*value) / QSFP_WAVELENGTH_DIVIDER);
            } else if(param == SDI_MEDIA_WAVELENGTH_TOLERANCE) {
                /* Guaranteed range of laser wavelength(+/- value) from nominal
                 * wavelength. (wavelength Tol.=value/200 in nm) */
                *value = ( (*value) / QSFP_WAVELENGTH_TOLERANCE_DIVIDER);
            }
        } else if(size == SDI_QSFP_DOUBLE_WORD_SIZE) {
            *value = ( (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3] );
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
t_std_error sdi_qsfp_vendor_info_get(sdi_resource_hdl_t resource_hdl,
                                     sdi_media_vendor_info_type_t vendor_info_type,
                                     char *vendor_info, size_t size)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    size_t data_len = 0;
    uint_t offset = 0;
    uint8_t *buf_ptr = NULL;
    uint8_t data_buf[SDI_MAX_NAME_LEN] = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(vendor_info != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }
        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        offset = vendor_reg_info[vendor_info_type].offset;
        data_len = vendor_reg_info[vendor_info_type].size;

        /* Input buffer size should be greater than or equal to data len*/
        STD_ASSERT(size >= data_len);

        rc = sdi_smbus_read_multi_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                offset, data_buf, data_len, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d reg : %d"
                    "rc : %d", qsfp_device->addr, offset, rc);
            break;
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if( rc == STD_ERR_OK) {
        if( (vendor_info_type == SDI_MEDIA_VENDOR_DATE) ||
            (vendor_info_type == SDI_MEDIA_VENDOR_OUI) ) {
            snprintf(vendor_info, data_len, "%s", data_buf);
        } else {
            /* vendor name, part number, serial number and revision fields contains
             * ASCII characters, left-aligned and padded on the right with ASCII
             * spaces (20h).*/
            for(buf_ptr = &data_buf[data_len]; *(buf_ptr - 1) == 0x20; buf_ptr--);
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
t_std_error sdi_qsfp_transceiver_code_get(sdi_resource_hdl_t resource_hdl,
                                          sdi_media_transceiver_descr_t *transceiver_info)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t buf[SDI_QSFP_QUAD_WORD_SIZE] = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(transceiver_info != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s rc : %d",
                    qsfp_device->alias, rc);
            break;
        }
        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        rc = sdi_smbus_read_multi_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                QSFP_COMPLIANCE_CODE_OFFSET, buf, SDI_QSFP_QUAD_WORD_SIZE,
                SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d rc : %d",
                    qsfp_device->addr, rc);
            break;
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if (rc == STD_ERR_OK) {
        memcpy((char *)transceiver_info, (char *)buf, SDI_QSFP_QUAD_WORD_SIZE);
    }

    return rc;
}

/**
 * Read the dell product information
 * resource_hdl[in] - Handle of the resource
 * info[out]        - dell product information
 * return           - standard t_std_error
 */
t_std_error sdi_qsfp_dell_product_info_get(sdi_resource_hdl_t resource_hdl,
                                           sdi_media_dell_product_info_t *info)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(info != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        rc = sdi_smbus_read_multi_byte(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                QSFP_DELL_PRODUCT_ID_OFFSET, (uint8_t *)info,
                sizeof(sdi_media_dell_product_info_t), SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d rc : %d",
                    qsfp_device->addr, rc);
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);
    return rc;
}

/**
 * Get the alarm and warning thresholds for the given optic
 * resource_hdl[in] - Handle of the resource
 * threshold_type[in] - Type of the threshold
 * value[out] - Threshold value
 * return - standard t_std_error
 */
t_std_error sdi_qsfp_threshold_get (sdi_resource_hdl_t resource_hdl,
                                    sdi_media_threshold_type_t threshold_type,
                                    float *value)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t offset = 0;
    bool paging_support_flag = true;
    uint8_t threshold_buf[2] = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    offset = threshold_reg_info[threshold_type].offset;

    if( offset == 0 )  {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                                       qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                                   qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        /* Select the page-3 of qsfp eeprom where threshold values are located */
        rc = sdi_qsfp_page_select(qsfp_device, SDI_QSFP_PAGE_03);
        if(rc != STD_ERR_OK){
            if( rc == SDI_DEVICE_ERRCODE(ENOTSUP) ) {
                paging_support_flag = false;
            } else {
                SDI_DEVICE_ERRMSG_LOG("page 3 selection is failed for %s",
                                       qsfp_device->alias);
            }
            break;
        }

        rc = sdi_smbus_read_word(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                                 offset, (uint16_t *)threshold_buf, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d reg : %d"
                                  "rc : %d", qsfp_device->addr, offset, rc);
            break;
        }
    } while(0);

    if(paging_support_flag == true) {
        /* Select the page-0 of qsfp eeprom which is default page */
        rc = sdi_qsfp_page_select(qsfp_device, SDI_QSFP_PAGE_00);
        if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("page 0 selection is failed for %s",
                                   qsfp_device->alias);
        }
    }

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if( (threshold_type == SDI_MEDIA_TEMP_HIGH_ALARM_THRESHOLD) ||
        (threshold_type == SDI_MEDIA_TEMP_LOW_ALARM_THRESHOLD) ||
        (threshold_type == SDI_MEDIA_TEMP_HIGH_WARNING_THRESHOLD) ||
        (threshold_type == SDI_MEDIA_TEMP_LOW_WARNING_THRESHOLD) ) {
        *value = convert_qsfp_temp(threshold_buf);
    } else if( (threshold_type == SDI_MEDIA_VOLT_HIGH_ALARM_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_VOLT_LOW_ALARM_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_VOLT_HIGH_WARNING_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_VOLT_LOW_WARNING_THRESHOLD) ) {
        *value = convert_qsfp_volt(threshold_buf);
    } else if( (threshold_type == SDI_MEDIA_RX_PWR_HIGH_ALARM_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_RX_PWR_LOW_ALARM_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_RX_PWR_HIGH_WARNING_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_RX_PWR_LOW_WARNING_THRESHOLD) ) {
        *value = convert_qsfp_rx_power(threshold_buf);
    } else if( (threshold_type == SDI_MEDIA_TX_BIAS_HIGH_ALARM_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_TX_BIAS_LOW_ALARM_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_TX_BIAS_HIGH_WARNING_THRESHOLD) ||
               (threshold_type == SDI_MEDIA_TX_BIAS_LOW_WARNING_THRESHOLD) ) {
        *value = convert_qsfp_tx_bias(threshold_buf);
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
t_std_error sdi_qsfp_module_monitor_threshold_get(sdi_resource_hdl_t resource_hdl,
                                                  uint_t threshold_type, uint_t *value)
{
    /* TODO:Depricated API and should me removed upper layers adopted new API */
    return STD_ERR_OK;
}

/**
 * Read the threshold values for channel monitors like rx-ower and tx-bias
 * resource_hdl[in]     - Handle of the resource
 * threshold_type[in]   - type of threshold
 * value[out]           - threshold value
 * return               - standard t_std_error
 */
t_std_error sdi_qsfp_channel_monitor_threshold_get(sdi_resource_hdl_t resource_hdl,
                                                   uint_t threshold_type, uint_t *value)
{
    /* TODO:Depricated API and should me removed upper layers adopted new API */
    return STD_ERR_OK;
}

/**
 * Debug api to retrieve module monitors assoicated with the specified QSFP
 * resource_hdl[in] - Handle of the resource
 * monitor[in]      - monitor which needs to be retrieved
 * value[out]     - Value of the monitor
 * return           - t_std_error
 */
t_std_error sdi_qsfp_module_monitor_get (sdi_resource_hdl_t resource_hdl,
                                         sdi_media_module_monitor_t monitor, float *value)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t temp_buf[2] = { 0 };
    uint8_t volt_buf[2] = { 0 };

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        switch (monitor)
        {
            case SDI_MEDIA_TEMP:
                rc = sdi_smbus_read_word(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                        QSFP_TEMPERATURE_OFFSET, (uint16_t *)temp_buf,
                        SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d ",
                            qsfp_device->addr);
                }
                break;

            case SDI_MEDIA_VOLT:
                rc = sdi_smbus_read_word(qsfp_device->bus_hdl,
                        qsfp_device->addr.i2c_addr, QSFP_VOLTAGE_OFFSET,
                        (uint16_t *)volt_buf, SDI_I2C_FLAG_NONE);
                if (rc != STD_ERR_OK){
                    SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d ",
                            qsfp_device->addr);
                }
                break;

            default:
                rc = SDI_DEVICE_ERRCODE(EINVAL);
        }
    } while(0);
    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if(rc == STD_ERR_OK) {
        if(monitor == SDI_MEDIA_TEMP) {
            *value = convert_qsfp_temp(temp_buf);
        } else if(monitor == SDI_MEDIA_VOLT) {
            *value = convert_qsfp_volt(volt_buf);
        }
    }

    return rc;
}

/**
 * Retrieve channel monitors assoicated with the specified QSFP
 * resource_hdl[in] - Handle of the resource
 * channel[in]      - channel whose monitor has to be retreived
 * monitor[in]      - monitor which needs to be retrieved
 * value[out]     - Value of the monitor
 * return           - t_std_error
 */
t_std_error sdi_qsfp_channel_monitor_get (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                          sdi_media_channel_monitor_t monitor, float *value)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint8_t buf[2] = { 0 };
    uint_t reg_offset = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(value != NULL);

    if (sdi_qsfp_validate_channel(channel) != true){
        return SDI_DEVICE_ERR_PARAM;
    }

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    switch (monitor)
    {
        case SDI_MEDIA_INTERNAL_RX_POWER_MONITOR:
            if (channel == SDI_QSFP_CHANNEL_ONE){
                reg_offset = QSFP_RX1_POWER_OFFSET;
            } else if (channel == SDI_QSFP_CHANNEL_TWO){
                reg_offset = QSFP_RX2_POWER_OFFSET;
            } else if (channel == SDI_QSFP_CHANNEL_THREE){
                reg_offset = QSFP_RX3_POWER_OFFSET;
            } else if (channel == SDI_QSFP_CHANNEL_FOUR){
                reg_offset = QSFP_RX4_POWER_OFFSET;
            } else {
                return SDI_DEVICE_ERRCODE(EINVAL);
            }
            break;

        case SDI_MEDIA_INTERNAL_TX_POWER_BIAS:
            if (channel == SDI_QSFP_CHANNEL_ONE){
                reg_offset = QSFP_TX1_POWER_BIAS_OFFSET;
            } else if (channel == SDI_QSFP_CHANNEL_TWO){
                reg_offset = QSFP_TX2_POWER_BIAS_OFFSET;
            } else if (channel == SDI_QSFP_CHANNEL_THREE){
                reg_offset = QSFP_TX3_POWER_BIAS_OFFSET;
            } else if (channel == SDI_QSFP_CHANNEL_FOUR){
                reg_offset = QSFP_TX4_POWER_BIAS_OFFSET;
            } else {
                return SDI_DEVICE_ERRCODE(EINVAL);
            }
            break;

        case SDI_MEDIA_INTERNAL_TX_OUTPUT_POWER:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);

        default:
            return SDI_DEVICE_ERRCODE(EINVAL);
    }

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s",
                    qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        rc = sdi_smbus_read_word(qsfp_device->bus_hdl, qsfp_device->addr.i2c_addr,
                reg_offset, (uint16_t *)buf, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("qsfp smbus read failed at addr : %d"
                    "reg : %d ", qsfp_device->addr, reg_offset);
            break;
        }
    } while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);

    if( rc == STD_ERR_OK) {
        if(monitor == SDI_MEDIA_INTERNAL_RX_POWER_MONITOR) {
            *value = convert_qsfp_rx_power(buf);
        } else if(monitor == SDI_MEDIA_INTERNAL_TX_POWER_BIAS) {
            *value = convert_qsfp_tx_bias(buf);
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
t_std_error sdi_qsfp_feature_support_status_get (sdi_resource_hdl_t resource_hdl,
                                                 sdi_media_supported_feature_t *feature_support)
{
    sdi_device_hdl_t qsfp_device = NULL;
    qsfp_device_t *qsfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(feature_support != NULL);

    qsfp_device = (sdi_device_hdl_t)resource_hdl;
    qsfp_priv_data = (qsfp_device_t *)qsfp_device->private_data;
    STD_ASSERT(qsfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(qsfp_priv_data->mod_sel_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    do {
        rc = sdi_pin_group_write_level(qsfp_priv_data->mod_sel_hdl,
                                       qsfp_priv_data->mod_sel_value);
        if (rc != STD_ERR_OK){
            SDI_DEVICE_ERRMSG_LOG("module selection is failed for %s", qsfp_device->alias);
            break;
        }

        std_usleep(MILLI_TO_MICRO(qsfp_priv_data->delay));

        rc = sdi_is_tx_control_supported(qsfp_device,
                                        &feature_support->qsfp_features.tx_control_support_status);
        if (rc != STD_ERR_OK){
            break;
        }

        rc = sdi_is_paging_supported(qsfp_device,
                                    &feature_support->qsfp_features.paging_support_status);
        if (rc != STD_ERR_OK){
            break;
        }

        rc = sdi_is_rate_select_supported(qsfp_device,
                                         &feature_support->qsfp_features.rate_select_status);
        if (rc != STD_ERR_OK){
            break;
        }
    }while(0);

    sdi_pin_group_release_bus(qsfp_priv_data->mod_sel_hdl);
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
t_std_error sdi_qsfp_read (sdi_resource_hdl_t resource_hdl, uint_t offset,
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
t_std_error sdi_qsfp_write (sdi_resource_hdl_t resource_hdl, uint_t offset,
                            uint8_t *data, size_t data_len)
{
    return STD_ERR_UNIMPLEMENTED;
}

