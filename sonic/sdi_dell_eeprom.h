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
 * filename: sdi_dell_eeprom.h
 */


/*******************************************************************
 * @file    sdi_dell_eeprom.h
 * @brief   Declaration of DELL EEPROM related info.
 *
 *******************************************************************/
#ifndef _SDI_DELL_EEPROM_H_
#define _SDI_DELL_EEPROM_H_

#include "std_config_node.h"
#include "sdi_entity_info_internal.h"

/* Delta PSU EEPROM Specific Fields */

#define SDI_DELTA_PSU_SERIAL_NUM "DELTA_PSU_SERIAL_NUM"
#define SDI_DELTA_PSU_PART_NUM   "DELTA_PSU_PART_NUM"
#define SDI_DELTA_PSU_LABEL_REV  "DELTA_PSU_LABEL_REV"
#define SDI_DELTA_PSU_FAN_AIRFLOW "DELTA_PSU_FAN_AIRFLOW"
#define SDI_DELTA_PSU_POWER_SRC   "DELTA_PSU_POWER_SRC"
#define SDI_DELTA_PSU_POWER_WATT  "DELTA_PSU_POWER_WATT"
#define SDI_DELTA_PSU_FAN_NUMBER  "DELTA_PSU_FAN_NUMBER"
#define SDI_DELTA_PSU_FAN_SPEED  "DELTA_PSU_FAN_SPEED"

#define SDI_DELTA_PSU_SERIAL_NUM_OFFSET  0xc4
#define SDI_DELTA_PSU_PART_NUM_OFFSET    0xd8
#define SDI_DELTA_PSU_LABEL_REV_OFFSET   0xde
#define SDI_DELTA_PSU_FAN_AIRFLOW_OFFSET 0xe1
#define SDI_DELTA_PSU_POWER_SRC_OFFSET   0xe8
#define SDI_DELTA_PSU_POWER_WATT_OFFSET  0xed
#define SDI_DELTA_PSU_FAN_NUMBER_OFFSET  0xf1
#define SDI_DELTA_PSU_FAN_SPEED_OFFSET   0xf2

#define SDI_DELTA_PSU_SERIAL_NUM_SIZE  20
#define SDI_DELTA_PSU_PART_NUM_SIZE     6
#define SDI_DELTA_PSU_LABEL_REV_SIZE    3
#define SDI_DELTA_PSU_FAN_AIRFLOW_SIZE  7
#define SDI_DELTA_PSU_POWER_SRC_SIZE    5
#define SDI_DELTA_PSU_POWER_WATT_SIZE   4
#define SDI_DELTA_PSU_FAN_NUMBER_SIZE   1
#define SDI_DELTA_PSU_FAN_SPEED_SIZE    5


/* FAN and PSU Related info */
#define SDI_DELL_LEGACY_PSU_TYPE                      "DELL_LEGACY_PSU_TYPE"
#define SDI_DELL_LEGACY_FAN_TYPE                      "DELL_LEGACY_FAN_TYPE"
#define SDI_DELL_LEGACY_FAN_COUNT                     "DELL_LEGACY_FAN_COUNT"
#define SDI_DELL_LEGACY_PSU_TYPE_OFFSET               134
#define SDI_DELL_LEGACY_FAN_TYPE_OFFSET               135
#define SDI_DELL_LEGACY_FAN_COUNT_OFFSET              134

#define SDI_DELL_LEGACY_FAN_TYPE_SIZE                 1
#define SDI_DELL_LEGACY_PSU_TYPE_SIZE                 1
#define SDI_DELL_LEGACY_FAN_COUNT_SIZE                1

/** Common EEPROM macros */
#define SDI_STR_DELTA_PSU_EEPROM                      "DELTA_PSU_EEPROM"
#define SDI_STR_DELL_LEGACY_PSU_EEPROM                "DELL_PSU_EEPROM"
#define SDI_STR_DELL_LEGACY_FAN_EEPROM                "DELL_FAN_EEPROM"
#define SDI_DELL_LEGACY_EEPROM_PPID                   "DELL_LEGACY_EEPROM_PPID"
#define SDI_DELL_LEGACY_EEPROM_MAX_LEN                256
#define SDI_DELL_LEGACY_EEPROM_PPID_SIZE              20
#define SDI_DELL_LEGACY_EEPROM_PPID_OFFSET            6
#define SDI_DELL_LEGACY_EEPROM_PART_NUM_SIZE          6
#define SDI_DELL_LEGACY_EEPROM_PART_NUM_OFFSET        2

/**
 * @enum sdi_dell_fan_air_flow_type_t
 * supported airflow types
 */
typedef enum{
    /**
     * Air flow direction is Normal = 1
     */
    SDI_DELL_LEGACY_FAN_AIR_FLOW_NORMAL = 1,
    /**
     * Air flow direction is Reverse = 2
     */
    SDI_DELL_LEGACY_FAN_AIR_FLOW_REVERSE
}sdi_dell_fan_air_flow_type_t;

/**
 * @enum sdi_dell_psu_type_t
 * supported PSU types
 */
typedef enum{
    SDI_DELL_LEGACY_PSU_AC_NORMAL,
    SDI_DELL_LEGACY_PSU_AC_REVERSE,
    SDI_DELL_LEGACY_PSU_DC_NORMAL,
    SDI_DELL_LEGACY_PSU_DC_REVERSE,
}sdi_dell_psu_type_t;

/**
 * Dell PSU EEPROM data get
 *
 * param[in] resource_hdl  - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_dell_legacy_psu_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info);

/**
 * Delta PSU EEPROM data get
 *
 * param[in] resource_hdl  - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */

t_std_error sdi_delta_psu_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info);

/**
 * FAN EEPROM data get
 *
 * param[in] resource_hdl  - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_dell_legacy_fan_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info);

#endif // _SDI_DELL_EEPROM_H_
