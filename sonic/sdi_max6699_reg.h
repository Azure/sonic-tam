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
 * filename: sdi_max6699_reg.h
 */


/******************************************************************************
 * Defines the max6699 temperature sensor register macros. This file is used by
 * the max6699 is used by max6699 temperature sensor driver, which supports
 * reading the temperature and reading/writing the high threshold limits for the
 * sensors.
 * The data sheet of the MAX 6699 is available in the below link
 * http://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX6699.html
 ******************************************************************************/

#ifndef _SDI_MAX6699_REG_H_
#define _SDI_MAX6699_REG_H_

/*
 * Naming for Register macros:
 * All the register values(Temperature, threshold are taken as integer,
 * hence the Low order byte which has the fractional part is ignored
 * ID   - Internal Diode
 * ED   - External Diode
 * HL   - High Limit
 * POR  - Power On Reset
 */

/* Temperature Data registers for sensors */
#define MAX6699_ID_DATA          0x07
#define MAX6699_ED_DATA_1        0x01
#define MAX6699_ED_DATA_2        0x02
#define MAX6699_ED_DATA_3        0x03
#define MAX6699_ED_DATA_4        0x04

/* The High threshold limit registers for sensors */
#define MAX6699_ID_HL        0x17
#define MAX6699_ED_HL_1      0x11
#define MAX6699_ED_HL_2      0x12
#define MAX6699_ED_HL_3      0x13
#define MAX6699_ED_HL_4      0x14

/* The Status register*/
#define MAX6699_STATUS_1_REG    0x44 /* Sensor High Temperature Alert status */
#define MAX6699_STATUS_2_REG    0x45 /* Sensor Over Temperature Status */
#define MAX6699_STATUS_3_REG    0x46 /* Sensor Diode Fault status */

/* Bit offset for High Temperature Alert and Diode fault status */
#define MAX6699_ID_HL_BIT       6
#define MAX6699_ED_HL_BIT_1     0
#define MAX6699_ED_HL_BIT_2     1
#define MAX6699_ED_HL_BIT_3     2
#define MAX6699_ED_HL_BIT_4     3

/* Max no.of sensors in the emc1428 chip */
#define MAX6699_MAX_SENSORS     5

/* Default POR(Power On Reset) high limit threshold values as per data sheet */
#define MAX6699_ID_HL_POR_VALUE     0x5A
#define MAX6699_ED_HL_POR_VALUE_1   0x6E
#define MAX6699_ED_HL_POR_VALUE_2   0x7F
#define MAX6699_ED_HL_POR_VALUE_3   0x64
#define MAX6699_ED_HL_POR_VALUE_4   0x64

#endif //_SDI_MAX6699_REG_H_

