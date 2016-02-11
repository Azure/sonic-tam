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
 * filename: sdi_i2c_mux_attr.h
 */


#ifndef __SDI_I2C_MUX_ATTR_H__
#define __SDI_I2C_MUX_ATTR_H__

/**
 * @file sdi_i2c_mux_attr.h
 * @brief defines list of attributes supported for i2c mux node
 * configuration
 *
 * @defgroup sdi_config_i2c_mux_attributes SDI I2C Mux Configuration attributes
 * @ingroup sdi_config_attributes
 * @brief SDI Configuration attribute commonly used by i2c mux node
 *
 * @{
 */

/**
 * @def Attribute used for representing parent i2c bus of i2cmux
 */
#define SDI_DEV_ATTR_SDI_I2CMUX_PARENT_I2CBUS    "parent"
/**
 * @def Attribute used for representing pins used to multiplex i2cmux
 */
#define SDI_DEV_ATTR_SDI_I2CMUX_PINBUS           "mux_pin_group_bus"
/**
 * @def Attribute used for representing i2c mux channel
 */
#define SDI_DEV_ATTR_SDI_I2CMUX_CHANNEL          "channel"

/**
 * @}
 */

#endif   /* __SDI_I2C_MUX_ATTR_H__ */
