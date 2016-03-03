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
 * filename: sdi_cpld_attr.h
 */


/******************************************************************************
 * Defines attributes used by cpld driver
 *****************************************************************************/

#ifndef __SDI_CPLD_ATTR_H__
#define __SDI_CPLD_ATTR_H__

/*
 * Attribute used for representing the bit offset of cpld pin bus
 * register address
 */
#define SDI_DEV_ATTR_CPLD_OFFSET        "offset"

/*
 * Attribute used for representing the register start address of cpld pin
 * group bus
 */
#define SDI_DEV_ATTR_CPLD_START_ADDR    "start_addr"

/*
 * Attribute used for representing the register end address of cpld pin
 * group bus
 */
#define SDI_DEV_ATTR_CPLD_END_ADDR      "end_addr"

/*
 * Attribute used for representing the bit start offset of cpld pin
 * group bus
 */
#define SDI_DEV_ATTR_CPLD_START_OFFSET  "start_offset"

/*
 * Attribute used for representing the bit end offset of cpld pin
 * group bus
 */
#define SDI_DEV_ATTR_CPLD_END_OFFSET    "end_offset"

/*
 * Attribute used for representing the bit end offset of cpld pin
 * group bus
 */
#define SDI_DEV_ATTR_CPLD_END_OFFSET    "end_offset"

/*
 * Attribute used for representing the width of cpld pin group bus
 */
#define SDI_DEV_ATTR_CPLD_WIDTH         "width"

/*
 * Default cpld register's width in bytes
 */
#define SDI_CPLD_DEFAULT_REGISTER_WIDTH        1

#endif /* __SDI_CPLD_ATTR_H__ */
