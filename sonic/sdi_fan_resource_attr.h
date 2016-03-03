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
 * filename: sdi_fan_resource_attr.h
 */


#ifndef __SDI_FAN_RESOURCE_ATTR_H__
#define __SDI_FAN_RESOURCE_ATTR_H__

/**
 * @file sdi_fan_resource_attr.h
 * @brief defines list of attributes commonly used for SDI_FAN_RESOURCE
 * configuration
 *
 * @defgroup sdi_config_fan_resource_attributes SDI configuration Fan
 * resources attributes
 * @ingroup sdi_config_attributes
 * @brief Configuration Attributes commonly used by Device nodes that export
 *    @ref SDI_FAN_RESOURCE
 *
 * @{
 */

/**
 * @def Node name used for representing the fan
 */
#define SDI_DEV_NODE_FAN  "fan"

/**
 * @def Attribute used for representing the default speed of a
 * fan resource
 */
#define SDI_DEV_ATTR_FAN_SPEED            "fan_speed"

/**
 * @def Attribute used for representing the tach pulse period of a
 * fan resource
 */
#define SDI_DEV_ATTR_FAN_NP            "no_of_tach_pulse"

/**
 * @def Attribute used for representing whether full speed should be
 * enabled on failure for a fan resource
 */
#define SDI_DEV_ATTR_FAN_EN_FS            "enable_full_speed"

/**
 * @def Attribute used for representing the poles of a fan resource
 */
#define SDI_DEV_ATTR_FAN_POLES            "poles"

/**
 * @}
 */

#endif   /* __SDI_FAN_RESOURCE_ATTR_H__ */
