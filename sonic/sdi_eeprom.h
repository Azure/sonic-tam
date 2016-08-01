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
 * filename: sdi_eeprom.h
 */


/*******************************************************************
 * @file    sdi_eeprom.h
 * @brief   Declaration of eeprom driver functions.
 *          Also eeprom parser formats related info.
 *******************************************************************/

#ifndef _SDI_EEPROM_H_
#define _SDI_EEPROM_H_

#include "std_config_node.h"
#include "sdi_entity_info_internal.h"
#include "sdi_sys_common.h"
#include "sdi_driver_internal.h"

/** Common EEPROM macros */
#define SDI_BYTE_LENGTH                        8
#define SDI_EEPROM_START_OFFSET                0
#define SDI_STR_PSU                            "PSU"
#define SDI_PWR_RATING_SUPPORT                 1

/** EEPROM parser formats */
typedef enum {
     SDI_ONIE_SYS_EEPROM,             /**< ONIE SYSTEM EEPROM */
     SDI_ONIE_PSU_EEPROM,             /**< ONIE PSU EEPROM */
     SDI_ONIE_FAN_EEPROM,             /**< ONIE FAN EEPROM */
     SDI_DELL_LEGACY_PSU_EEPROM,      /**< DELL LEGACY PSU EEPROM */
     SDI_DELL_LEGACY_FAN_EEPROM,      /**< DELL LEGACY FAN EEPROM */
     SDI_DELTA_PSU_EEPROM,            /**< DELTA LEGACY PSU EEPROM */
}sdi_entity_parser_t ;

/**
 * @struct eeprom_device
 * ENTITY/EEPROM device private data
 */
typedef struct {
    /** Store the current entity info */
    uint_t entity_size;           /**< Size of the eeprom device */
    sdi_entity_parser_t format;   /**< Parser format */
    uint_t no_of_fans;            /**< No.of fan in the entity */
    uint_t max_fan_speed;         /**< Max Speed of the fan in the entity*/
    char alias[SDI_MAX_NAME_LEN]; /**< Device Alias */
}entity_info_device_t ;

#endif // _SDI_EEPROM_H_
