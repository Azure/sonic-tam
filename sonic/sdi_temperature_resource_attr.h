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
 * filename: sdi_temperature_resource_attr.h
 */


/**
 * @file sdi_temperature_resource_attr.h
 * @brief defines list of attributes commonly used for SDI_TEMPERATURE_RESOURCE
 * configuration
 */

#ifndef __SDI_TEMPERATURE_RESOURCE_ATTR_H__
#define __SDI_TEMPERATURE_RESOURCE_ATTR_H__

/**
 * @defgroup sdi_config_temperature_resource_attributes SDI Configuration Temperature Resource Attributes
 * @ingroup sdi_config_attributes
 * @brief Configuration Attributes commonly used by Device nodes that export
 *    @ref SDI_TEMPERATURE_RESOURCE
 *
 * @{
 */

#include "sdi_common_attr.h"

/**
 * @def Attribute used for representing the default lower threshold of a
 * temperature resource
 */
#define SDI_DEV_ATTR_TEMP_LOW_THRESHOLD         "low_threshold"

/**
 * @def Attribute used for representing the default high threshold of a
 * temperature resource
 */
#define SDI_DEV_ATTR_TEMP_HIGH_THRESHOLD        "high_threshold"

/**
 * @def Attribute used for representing the default critical threshold of a
 * temperature resource
 */
#define SDI_DEV_ATTR_TEMP_CRITICAL_THRESHOLD    "critical_threshold"

/**
 * @def Node name used to represent resources of type SDI_THERMAL_RESOURCE
 */
#define SDI_DEV_NODE_TEMP_SENSOR                "temp_sensor"

/**
 * @}
 */

#endif   /* __SDI_TEMPERATURE_RESOURCE_ATTR_H__ */
