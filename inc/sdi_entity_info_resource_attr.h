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
 * filename: sdi_entity_info_resource_attr.h
 */


/**
 * @file sdi_entity_info_resource_attr.h
 * @brief defines list of attributes commonly used for SDI_ENTITY_INFO_RESOURCE
 * configuration
 *
 * @defgroup sdi_config_entity_info_resource_attributes Sdi Config Entity Info Resource Attributes.
 * @ingroup sdi_config_attributes
 * @brief Configuration Attributes commonly used by Device nodes that export
 * @ref SDI_ENTITY_INFO_RESOURCE
 *
 * @{
 */

#ifndef __SDI_ENTITY_INFO_RESOURCE_ATTR_H__
#define __SDI_ENTITY_INFO_RESOURCE_ATTR_H__

#include "sdi_common_attr.h"

/**
 * @def Attribute used for representing the size of the entity_info resource
 */
#define SDI_DEV_ATTR_ENTITY_SIZE              "size"

/**
 * @def Attribute used for representing the parser format of the entity_info resource
 */
#define SDI_DEV_ATTR_PARSER                   "parser"

/**
 * @def Attribute used for representing the maximum fan speed of the entity
 */
#define SDI_DEV_ATTR_MAX_SPEED                "max_fan_speed"

/**
 * @def Attribute used for representing the no.of fans of the entity
 */
#define SDI_DEV_ATTR_NO_OF_FANS               "no_of_fans"

/**
 * @}
 */

#endif   /* __SDI_ENTITY_INFO_RESOURCE_ATTR_H__ */
