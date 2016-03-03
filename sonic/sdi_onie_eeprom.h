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
 * filename: sdi_onie_eeprom.h
 */


/*******************************************************************
 * @file    sdi_onie_eeprom.h
 * @brief   Declaration of ONIE TLV related info.
 *
 * More details about ONIE and TLV, refer the below link.
 * http://opencomputeproject.github.io/onie/docs/design-spec/switch_hw_requirements.html
 *******************************************************************/

#ifndef __SDI_ONIE_EEPROM_H_
#define __SDI_ONIE_EEPROM_H_

#include "std_config_node.h"
#include "sdi_entity_info_internal.h"

/** TLV Header related */
#define SDI_TL_FIELD_SIZE                      (sizeof(sdi_onie_tlv_field)-1)
#define SDI_TLV_FIELD_SIZE                     sizeof(sdi_onie_tlv_field)
#define SDI_TLV_HEADER_SIZE                    sizeof(sdi_onie_tlv_header)
#define SDI_ONIE_HDR_VERSION                   0x01
#define SDI_ONIE_HDR_ID_STRING                 "TlvInfo"
#define SDI_MAX_TLV_INFO_LEN                   2048
#define SDI_MAX_TLV_VALUE_LEN                  256

/** TLV Tag Value Size/Length */
#define SDI_TLV_CRC_SIZE                       6
#define SDI_ONIE_CRC_SIZE                      4
#define SDI_ONIE_NO_MACS_SIZE                  2
#define SDI_ONIE_EEPROM_PPID_SIZE              120
#define SDI_ONIE_DEVICE_VERSION_SIZE           1
#define SDI_ONIE_EEPROM_PPID_REV_SIZE          3
#define SDI_ONIE_EEPROM_MAC_ADDR_SIZE          6
#define SDI_ONIE_EEPROM_MFG_DATE_SIZE          19
#define SDI_ONIE_EEPROM_SERVICE_TAG_SIZE       7
#define SDI_ONIE_EEPROM_PART_NUMBER_SIZE       10
#define SDI_ONIE_EEPROM_COUNTRY_CODE_SIZE      2
#define SDI_ONIE_EEPROM_PRODUCT_NAME_SIZE      127
#define SDI_ONIE_EEPROM_SERIAL_NUMBER_SIZE     120
#define SDI_ONIE_EEPROM_PART_NUMBER_REV_SIZE   3
#define SDI_ONIE_EEPROM_MFG_TEST_RESULT_SIZE   2

/** Common EEPROM macros */
#define SDI_STR_ONIE_SYS_EEPROM                "ONIE_SYS_EEPROM"
#define SDI_STR_ONIE_PSU_EEPROM                "ONIE_PSU_EEPROM"
#define SDI_STR_ONIE_FAN_EEPROM                "ONIE_FAN_EEPROM"
#define SDI_VENDOR_EXTN_DELIMITER              "-"

/** Structure for representing the EEPROM fields in TLV format */
typedef struct
{
    uint8_t tag;             /**< TLV TAG */
    uint8_t length;          /**< Length of the TAG Value */
    uint8_t value[0];        /**< Value of the TAG */
} __attribute__((packed)) sdi_onie_tlv_field;

/** Structure for representing the EEPROM Headers in TLV format */
typedef struct
{
    char id_string[8];        /**< ID String */
    uint8_t hdr_version;     /**< TLV Header Version */
    uint16_t total_len;      /**< Total Number of bytes of data*/
} __attribute__((packed)) sdi_onie_tlv_header;

/** ONIE TLV Tags */
enum {

     /** Each TAG has its own value.
      * Careful before adding any TAG, also ensure TAGs are correctly
      * assigned its own value as per ONIE TAG. */
     SDI_ONIE_PRODUCT_NAME_TAG = 0x21,
     SDI_ONIE_PART_NO_TAG = 0x22,
     SDI_ONIE_SERIAL_NO_TAG = 0x23,
     SDI_ONIE_MAC_ADDR_TAG = 0x24,
     SDI_ONIE_MFG_DATE_TAG = 0x25,
     SDI_ONIE_DEVICE_VERSION_TAG = 0x26,
     SDI_ONIE_LABEL_VERSION_TAG = 0x27,
     SDI_ONIE_PLATFORM_NAME_TAG = 0x28,
     SDI_ONIE_VERSION_TAG = 0x29,
     SDI_ONIE_NO_MACS_TAG = 0x2a,
     SDI_ONIE_MFGR_TAG = 0x2b,
     SDI_ONIE_COUNTRY_CODE_TAG = 0x2c,
     SDI_ONIE_VENDOR_TAG = 0x2d,
     SDI_ONIE_DIAG_VERSION_TAG = 0x2e,
     SDI_ONIE_SVC_TAG_TAG = 0x2f,

     SDI_ONIE_VENDOR_EXTN_TAG = 0xfd,
     SDI_ONIE_CRC32_TAG = 0xfe,               /**< CRC32 is the final TAG */
}sdi_onie_eeprom_tag;

/**
 * System EEPROM data get
 *
 * param[in] resource_hdl    - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_onie_sys_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info);

/**
 * PSU EEPROM data get
 *
 * param[in] resource_hdl    - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_onie_psu_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info);

/**
 * FAN EEPROM data get
 *
 * param[in] resource_hdl    - resource handler
 * param[out] entity_info  - entity_info structure to fill
 *
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
t_std_error sdi_onie_fan_eeprom_data_get(void *resource_hdl,
                                         sdi_entity_info_t *entity_info);

#endif // __SDI_ONIE_EEPROM_H_
