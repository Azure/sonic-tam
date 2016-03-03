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
 * filename: sdi_sfp.h
 */


/*******************************************************************
* @file   sdi_sfp.h
* @brief  Declares the sfp private data structures and driver functions
*******************************************************************/

#ifndef _SDI_SFP_H_
#define _SDI_SFP_H_

#include "sdi_media.h"
#include "sdi_resource_internal.h"
#include "sdi_pin_group.h"

#define SDI_SFP_CHANNEL_NUM 0

#define SDI_SFP_LED_HIGH_VALUE 1

/**
 * @struct sdi_media_led_t
 * SFP LED related data
 */
typedef struct port_led {
    sdi_pin_group_bus_hdl_t bus_hdl; /*<< port led pin group bus handle */
    uint_t bitmask; /*<< bit number for port led */
    uint_t led_1g_mode_value; /*<< 1G mode value to be written on bus */
    uint_t led_10g_mode_value; /*<< 10G mode value to be written on bus */
} sdi_media_led_t;

/**
 * @struct sfp_device_t
 * SFP device private data
 */
typedef struct sfp_device {
    /** sfp device module selection pin group bus handler */
    sdi_pin_group_bus_hdl_t mod_sel_hdl;
    /** value needs to be written on pin group bus for selecting module */
    uint_t mod_sel_value;
    /** sfp device module presence pin group bus handler */
    sdi_pin_group_bus_hdl_t mod_pres_hdl;
    /** sfp devie presence bit mask */
    uint_t mod_pres_bitmask;
    /** sfp device module transmitter control pin bus handler */
    sdi_pin_group_bus_hdl_t mod_tx_control_hdl;
    /** sfp devie tx control bit mask */
    uint_t mod_tx_control_bitmask;
    /** sfp device module rx los pin bus handler */
    sdi_pin_group_bus_hdl_t mod_sfp_rx_los_hdl;
    /** sfp devie rx los bit mask */
    uint_t mod_sfp_rx_los_bitmask;
    /** sfp device module tx fault pin bus handler */
    sdi_pin_group_bus_hdl_t mod_sfp_tx_fault_hdl;
    /** sfp devie tx fault bit mask */
    uint_t  mod_sfp_tx_fault_bitmask;
    /** flag for checking whether LED can be controlled by CPU */
    bool port_led_control_flag;
    /** port led related data */
    sdi_media_led_t port_led;
} sfp_device_t;

/**
 * @enum sfp_calib_info_type_t
 * sfp calibration types which is used to interpret the data
 */
typedef enum {
    /** internal calibration */
    SFP_CALIB_TYPE_INTERNAL = 0,
    /** external calibration */
    SFP_CALIB_TYPE_EXTERNAL = 1,
} sfp_calib_info_type_t;

/**
 * @struct sfp_calib_info_t
 * Caliberation Information. The use of calibration information is expalined in
 * SFP standard(SFF-8472) under section "Diagnostics Overview"
 */
typedef struct {
    /** Type of calibration */
    sfp_calib_info_type_t type;
    /** External calibration constant offset, not applicable for internal
     * calibration */
    uint16_t offset;
    /** External calibration constant slope, not applicable for internal
     * calibration */
    uint8_t slope[2];
} sfp_calib_info_t;

/**
 * @def Attribute used to represent rx output power data length
 */
#define EXT_CAL_RX_POWER_DATA_LEN 4
/**
 * @def Attribute used to represent rx output power full length
 */
#define EXT_CAL_RX_POWER_LEN (EXT_CAL_RX_POWER_DATA_LEN * 5)

/**
 * @struct sfp_rx_power_calib_info_t
 * Calibration Information for Rx optical power. The use of calibration information
 * is explained in SFP standard(SFF-8472) under section "Diagnostics Overview"
 */
typedef struct {
    /** Type of calibration */
    sfp_calib_info_type_t type;
    /** Rx optical power constant - 4 */
    uint8_t rx_power_const_4[EXT_CAL_RX_POWER_DATA_LEN];
    /** Rx optical power constant - 3 */
    uint8_t rx_power_const_3[EXT_CAL_RX_POWER_DATA_LEN];
    /** Rx optical power constant - 2 */
    uint8_t rx_power_const_2[EXT_CAL_RX_POWER_DATA_LEN];
    /** Rx optical power constant - 1 */
    uint8_t rx_power_const_1[EXT_CAL_RX_POWER_DATA_LEN];
    /** Rx optical power constant - 0 */
    uint8_t rx_power_const_0[EXT_CAL_RX_POWER_DATA_LEN];
} sfp_rx_power_calib_info_t;

/**
 * @brief Get the required module alarm status of sfp
 * @param[in] resource_hdl - handle of the sfp resource
 * @param[in] flags - flags for status that are of interest.
 * @param[out] status - returns the set of status flags which are asserted.
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_module_monitor_status_get (sdi_resource_hdl_t resource_hdl,
                                                uint_t flags, uint_t *status);

/**
 * @brief Get the required channel alarm status of sfp
 * @param resource_hdl[in] - handle of the sfp resource
 * @param channel[in] - channel number
 * @param flags[in] - flags for channel monitoring status
 * @param status[out] - returns the set of status flags which are asserted
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_channel_monitor_status_get (sdi_resource_hdl_t resource_hdl,
                                                uint_t channel, uint_t flags, uint_t *status);

/**
 * @brief Get the required channel status of sfp
 * @param[in] resource_hdl - handle of the sfp resource
 * @param[in] channel - channel number
 * @param[in] flags - flags for channel status
 * @param[out] status - returns the set of status flags which are asserted
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_channel_status_get (sdi_resource_hdl_t resource_hdl,
                                        uint_t channel, uint_t flags, uint_t *status);

/**
 * @brief Disable/Enable the transmitter of sfp
 * @param[in] resource_hdl - handle of the resource
 * @param[in] channel - channel number
 * @param[in] enable - "false" to disable and "true" to enable
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_tx_control (sdi_resource_hdl_t resource_hdl,
                                uint_t channel, bool enable);

/**
 * @brief Get the transmitter status on a particular channel of sfp
 * @param[in] resource_hdl - handle of the resource
 * @param[in] channel - channel number
 * @param[out] status - "true" if transmitter enabled else "false"
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_tx_control_status_get(sdi_resource_hdl_t resource_hdl,
                                          uint_t channel, bool *status);

/**
 * @brief Read the speed of a sfp
 * @param[in] resource_hdl - handle of the resource
 * @param[out] speed - speed of sfp
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_speed_get(sdi_resource_hdl_t resource_hdl,
                              sdi_media_speed_t *speed);

/**
 * @brief Check whether the specified sfp is dell qualified.
 * @param[in] resource_hdl - handle of the sfp
 * @param[out] status - true if optics is dell qualified else false
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_is_dell_qualified (sdi_resource_hdl_t resource_hdl,
                                       bool *status);

/**
* @brief Reads the parameter value from eeprom
* @param[in] resource_hdl - handle of the sfp
* @param[in] param_type - parameter type.
* @param[out] value - value of the parameter read from eeprom.
* @return - standard @ref t_std_error
*/
t_std_error sdi_sfp_parameter_get(sdi_resource_hdl_t resource_hdl,
                                  sdi_media_param_type_t param, uint_t *value);

/**
 * @brief Reads the requested vendor information from eeprom
 * @param[in] resource_hdl - handle of the sfp
 * @param[in] vendor_info_type - vendor information that is of interest.
 * @param[out] vendor_info - vendor information which is read from eeprom.
 * @param[in] buf_size - size of the input buffer which is allocated by user(vendor_info)
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_vendor_info_get(sdi_resource_hdl_t resource_hdl,
                                    sdi_media_vendor_info_type_t vendor_info_type,
                                    char *vendor_info, size_t size);

/**
 * @brief Reads the transceiver compliance code
 * @param resource_hdl[in] - handle of the sfp
 * @param transceiver_info[out] - transceiver compliance code information
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_transceiver_code_get(sdi_resource_hdl_t resource_hdl,
                                         sdi_media_transceiver_descr_t *transceiver_info);

/**
 * @brief Get the dell product information
 * @param resource_hdl[in] - Handle of the resource
 * @param info[out] - Dell product Identification data
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_dell_product_info_get(sdi_resource_hdl_t resource_hdl,
                                          sdi_media_dell_product_info_t *info);

/**
 * @brief Get the alarm and warning thresholds for the given optic
 * @param resource_hdl[in] - Handle of the resource
 * @param threshold_type[in] - Threshold type. Refer @ref sdi_media_dell_product_info_t
 * for valid threshold types
 * @param value[out] - Threshold value
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_threshold_get(sdi_resource_hdl_t resource_hdl,
                                  sdi_media_threshold_type_t threshold_type,
                                  float *value);

/**
 * @brief Get the threshold values for module monitors like temperature and
 * voltage
 * @param resource_hdl[in] - Handle of the resource
 * @param threshold_type[in] - Should be one of the flag @ref sdi_media_status
 * @param value[out] - threshold value
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_module_monitor_threshold_get(sdi_resource_hdl_t resource_hdl,
                                                 uint_t threshold_type, uint_t *value);

/**
 * @brief Get the threshold values for channel monitors like rx power and tx
 * bias
 * @param resource_hdl[in] - Handle of the resource
 * @param threshold_type[in] - Should be one of the flag @ref sdi_media_channel_status
 * @param value[out] - threshold value
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_channel_monitor_threshold_get(sdi_resource_hdl_t resource_hdl,
                                                  uint_t threshold_type, uint_t *value);

/**
 * @brief retrieve module monitors assoicated with the specified media.
 * @param[in] resource_hdl - handle to sfp
 * @param[in] monitor - The monitor which needs to be retrieved.
 * @param[out] value - Value of the monitor
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_module_monitor_get (sdi_resource_hdl_t resource_hdl,
                                        sdi_media_module_monitor_t monitor, float *value);

/**
 * @brief retrieve channel monitors assoicated with the specified media.
 * @param[in] resource_hdl - handle to sfp
 * @param[in] channel - The channel number
 * @param[in] monitor - The monitor which needs to be retrieved.
 * @param[out] value - Value of the monitor
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_channel_monitor_get (sdi_resource_hdl_t resource_hdl, uint_t channel,
                                         sdi_media_channel_monitor_t monitor, float *value);

/**
 * @brief Get the optional feature support status for optics
 * @param resource_hdl[in] - handle to sfp
 * @param feature_support[out] - feature support flags. Flag will be set to
 * "true" if feature supported else "false"
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_feature_support_status_get (sdi_resource_hdl_t resource_hdl,
                                                sdi_media_supported_feature_t *feature_support);

/**
 * @brief Set the port LED based on the speed settings of the port
 * @param resource_hdl[in] - handle to sfp
 * @param channel[in] - Channel number. Should be 0 in sfp
 * @param speed[in] - LED mode setting is derived from speed
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_led_set (sdi_resource_hdl_t resource_hdl, uint_t channel,
                             sdi_media_speed_t speed);

/**
 * @brief raw read from sfp eeprom
 * @param[in] resource_hdl - handle to the sfp
 * @param[in] offset - offset from which to read
 * @param[out] data - buffer for read data
 * @param[in] data_len - length of the data to be read
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_read (sdi_resource_hdl_t resource_hdl, uint_t offset,
                          uint8_t *data, size_t data_len);

/**
 * @brief Debug api to write data in to sfp eeprom
 * @param[in] resource_hdl - handle to the sfp
 * @param[in] offset - offset from which to read
 * @param[in] data - input buffer which contains the data to be written
 * @param[in] data_len - length of the data to be written
 * @return - standard @ref t_std_error
 */
t_std_error sdi_sfp_write (sdi_resource_hdl_t resource_hdl, uint_t offset,
                           uint8_t *data, size_t data_len);

/**
 * @brief Api to enable/disable Autoneg on media PHY .
 * @param[in] resource_hdl - handle to media
 * @param[in] speed - speed of the interface. Should be of type @ref sdi_media_speed_t .
 * @return - standard @ref t_std_error
 *
 */
t_std_error sdi_sfp_phy_autoneg_set (sdi_resource_hdl_t resource_hdl,bool enable);

/**
 * @brief Api to set mode on media PHY .
 * @param[in] resource_hdl - handle to media
 * @param[in] mode - SGMII/MII/GMII, Should be of type Refer @ref sdi_media_mode_t.
 * @return - standard @ref t_std_error
 *
 */

t_std_error sdi_sfp_phy_mode_set (sdi_resource_hdl_t resource_hdl,sdi_media_mode_t mode);

/**
 * @brief Api to  set speed  on media PHY .
 * @param[in] resource_hdl - handle to media
 * @param[in] speed - speed of the interface. Should be of type @ref sdi_media_speed_t .
 * @return - standard @ref t_std_error
 *
 */

t_std_error sdi_sfp_phy_speed_set(sdi_resource_hdl_t resource_hdl,sdi_media_speed_t speed);

#endif
