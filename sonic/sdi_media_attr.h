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
 * filename: sdi_media_attr.h
 */


/**
 * @file sdi_media_attr.h
 * @brief defines list of attributes supported for media node configuration
 *
 * @defgroup sdi_config_media_attributes SDI Configuration Media Attributes
 * @ingroup sdi_config_attributes
 *
 * @{
 */

#ifndef _SDI_MEDIA_ATTR_H_
#define _SDI_MEDIA_ATTR_H_

/**
 * @def Attribute used to represent delay, where delay is not required
 */
#define SDI_MEDIA_NO_DELAY 0

/**
 * @def Attribute used for representing pin group bus for selecting mux
 */
#define SDI_MEDIA_MUX_SELECTION_BUS      "mux_sel_bus"
/**
 * @def Attribute used for selecting mux. mux_sel_value needs to be written
 * on a pin/pin group bus for selecting a mux
 */
#define SDI_MEDIA_MUX_SELECTION_VALUE   "mux_sel_value"
/**
 * @def Attribute used for representing pin group bus for selecting module
 */
#define SDI_MEDIA_MODULE_SELECTION_BUS      "mod_sel_bus"
/**
 * @def Attribute used for selecting module. mod_sel_value needs to be written
 * on a pin/pin group bus for selecting a module
 */
#define SDI_MEDIA_MODULE_SELECTION_VALUE   "mod_sel_value"
/**
 * @def Attribute used to represent that single module present on the bus. When
 * a single module present on the bus, then mod_sel_bus should initialize with
 * this attribute.
 */
#define SDI_MEDIA_MODULE_SEL_ALWAYS_ENABLED "module_always_enabled"
/**
 * @def Attribute used for representing pin group bus for module presence status
 */
#define SDI_MEDIA_MODULE_PRESENCE_BUS       "mod_pres_bus"
/**
 * @def Attribute used for representing bit number for getting module presence
 * status
 */
#define SDI_MEDIA_MODULE_PRESENCE_BITMASK   "mod_pres_bitmask"
/**
 * @def Attribute used for representing pin group bus for module reset
 */
#define SDI_MEDIA_MODULE_RESET_BUS          "mod_reset_bus"
/**
 * @def Attribute used for representing bit number for controlling module reset
 */
#define SDI_MEDIA_MODULE_RESET_BITMASK      "mod_reset_bitmask"
/**
 * @def Attribute used for representing pin group bus for setting low power mode
 */
#define SDI_MEDIA_MODULE_LPMODE_BUS          "mod_lpmode_bus"
/**
 * @def Attribute used for representing bit number for controlling low power
 * mode
 */
#define SDI_MEDIA_MODULE_LPMODE_BITMASK      "mod_lpmode_bitmask"
/**
 * @def Attribute used for representing delay after module selection
 */
#define SDI_MEDIA_MODULE_SELECTION_DELAY_IN_MILLI_SECONDS    "mod_sel_delay"
/**
 * @def Attribute used for representing pin group bus for controlling
 * transmitter
 */
#define SDI_MEDIA_MODULE_TX_CONTROL_BUS         "mod_tx_control_bus"
/**
 * @def Attribute used for representing bit number for controlling transmitter
 */
#define SDI_MEDIA_MODULE_TX_CONTROL_BITMASK     "mod_tx_control_bitmask"
/**
 * @def Attribute used for representing pin group bus for rx los
 */
#define SDI_MEDIA_MODULE_RX_LOS_BUS             "mod_rx_los_bus"
/**
 * @def Attribute used for representing bit number for rx los
 */
#define SDI_MEDIA_MODULE_RX_LOS_BITMASK         "mod_rx_los_bitmask"
/**
 * @def Attribute used for representing pin group bus for tx fault
 */
#define SDI_MEDIA_MODULE_TX_FAULT_BUS           "mod_tx_fault_bus"
/**
 * @def Attribute used for representing bit number for tx fault
 */
#define SDI_MEDIA_MODULE_TX_FAULT_BITMASK       "mod_tx_fault_bitmask"
/* @def Attribute used for representing pin group bus for port led mode setting */
#define SDI_MEDIA_PORT_LED_BUS                  "port_led_bus"
/* @def Attribute used for representing bit number for port led  */
#define SDI_MEDIA_PORT_LED_BITMASK              "port_led_bit_mask"
/* @def Attribute used for representing 1G mode value for port led  */
#define SDI_MEDIA_PORT_LED_1G_MODE_VALUE        "port_led_1g_mode_value"
/* @def Attribute used for representing 10G mode value for port led  */
#define SDI_MEDIA_PORT_LED_10G_MODE_VALUE       "port_led_10g_mode_value"

/**
 * @}
 */

#endif   /* _SDI_MEDIA_ATTR_H_ */
