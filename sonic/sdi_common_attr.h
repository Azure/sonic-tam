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
 * filename: sdi_common_attr.h
 */


/**
 * @file sdi_common_attr.h
 * @brief defines list of attributes supported for many nodes in configuration
 *
 * @defgroup sdi_config_common_attributes SDI Configuration Common Attributes
 * @ingroup sdi_config_attributes
 * @brief SDI Node configuration attributes commonly used by nodes
 * @{
 */

#ifndef __SDI_COMMON_ATTR_H__
#define __SDI_COMMON_ATTR_H__

/**
 * @def Attribute used for representing the instance of a node
 */
#define SDI_DEV_ATTR_INSTANCE        "instance"
/**
 * @def Attribute used for representing the alias name of a node
 */
#define SDI_DEV_ATTR_ALIAS            "alias"
/**
 * @def Attribute used for representing the address of a device
 */
#define SDI_DEV_ATTR_ADDRESS        "addr"
/**
   * @def Attribute used for representing name of the bus
   */
#define SDI_DEV_ATTR_NAME        "name"

/**
 * @}
 */

#endif   /* __SDI_COMMON_ATTR_H__ */
