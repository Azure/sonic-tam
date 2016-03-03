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
 * filename: genericideeprom.h
 */ 
     
/******************************************************************************
 * \file genericideeprom.h
 *
 * \brief  Components available from generic eeprom  Client Driver are defined here
 *    Dependancy : PAL device library depends on bus and event log library
 ******************************************************************************/
#ifndef __GENERIC_IDEEPROM
#define __GENERIC_IDEEPROM

#define EEPROM_CRC_LEN                   4
#define EEPROM_INVALID_TLV_00            0x00
#define EEPROM_INVALID_TLV_FF            0xFF
#define EEPROM_MAC_ADDRESS_STRING_LEN    17
#define EEPROM_MAC_ADDRESS_BINARY_LEN    6
#define EEPROM_DATE_STRING_LEN           19
#define EEPROM_DATE_BKSLASH_POS_MM       2
#define EEPROM_DATE_BKSLASH_POS_DD       5
#define EEPROM_DATE_SPACE_POS_YEAR       10
#define EEPROM_DATE_COLON_POS_HH         13
#define EEPROM_DATE_COLON_POS_MM         16
#define EEPROM_BYTE_MAX_VAL              0xFF
#define EEPROM_DVICE_MAX_VERSION         0xFF
#define EEPROM_MAC_SIZE_MAX              65535

#define EEPROM_4_OCTETS_TO_WORD(val)	 ((val[0] << 24) | (val[1] << 16) | (val[2] << 8) | val[3])
#define EEPROM_WORD_To_4_OCTETS(valw, valo) \
    {                                \
    valo[0] = (valw >> 24) & 0xFF;   \
    valo[1] = (valw >> 16) & 0xFF;   \
    valo[2] = (valw >>  8) & 0xFF;   \
    valo[3] = (valw >>  0) & 0xFF;   \
    }

#endif  /* __GENERIC_IDEEPROM */
