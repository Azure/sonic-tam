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
 * filename: sdi_bus_attr.h
 */


/**
 * @file sdi_bus_attr.h
 * @brief defines list of attributes supported for every bus node
 * configuration
 *
 * @defgroup sdi_config_bus_attributes
 * @ingroup sdi_config_attributes
 * @brief SDI configuration attributes commonly used by sdi bus
 * @{
 */

#ifndef __SDI_BUS_ATTR_H__
#define __SDI_BUS_ATTR_H__

/**
 * @def Attribute used for representing the name of a bus
 */
#define SDI_DEV_ATTR_BUS_NAME        "bus_name"

/**
 * @def Attribute used for representing the bus device path name
 */
#define SDI_DEV_ATTR_DEV_NAME        "dev_name"
/**
 * @}
 */
#endif   /* __SDI_BUS_ATTR_H__ */
