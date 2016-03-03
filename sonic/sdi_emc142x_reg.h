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
 * filename: sdi_emc142x_reg.h
 */


/*****************************************************************
 * Defines the emc142x chip family register macros.This file
 * is used by the emc142x chip driver.This driver supports reading
 * the temperature and reading/writing the high,low and critical
 * threshold limits for the sensors.
 * This family of chip can have one or more sensors within it, this drivers
 * supports maximum of 8 sensors that the emc1428 chip has.
 * The data sheet of the EMC 1428 & 1422 are available in the below link
 * http://ww1.microchip.com/downloads/en/DeviceDoc/20005275A.pdf
 * http://ww1.microchip.com/downloads/en/DeviceDoc/1422.pdf
 *****************************************************************/

#ifndef __SDI_EMC142x_REG_H_
#define __SDI_EMC142x_REG_H_

/*
 * Naming for Register macros:
 * All the register values(Temperature, threshold are taken as integer,
 * hence the Low order byte which has the fractional part is ignored
 * ID - Internal Diode
 * ED - External Diode
 * HB - Higher order byte
 * HL - High Limit
 * LL - Low Limit
 */

/* Temperature Data registers for sensors */
#define EMC142x_ID_HB_DATA            0x00
#define EMC142x_ED_HB_DATA_1        0x01
#define EMC142x_ED_HB_DATA_2        0x23
#define EMC142x_ED_HB_DATA_3        0x2a
#define EMC142x_ED_HB_DATA_4        0x41
#define EMC142x_ED_HB_DATA_5        0x43
#define EMC142x_ED_HB_DATA_6        0x45
#define EMC142x_ED_HB_DATA_7        0x47

/* The High and Low threshold limit registers for sensors */
#define EMC142x_ID_HL           0x05
#define EMC142x_ED_HL_HB_1      0x07
#define EMC142x_ED_HL_HB_2      0x15
#define EMC142x_ED_HL_HB_3      0x2c
#define EMC142x_ED_HL_HB_4      0x50
#define EMC142x_ED_HL_HB_5      0x54
#define EMC142x_ED_HL_HB_6      0x58
#define EMC142x_ED_HL_HB_7      0x5c

#define EMC142x_ID_LL           0x06
#define EMC142x_ED_LL_HB_1      0x08
#define EMC142x_ED_LL_HB_2      0x16
#define EMC142x_ED_LL_HB_3      0x2d
#define EMC142x_ED_LL_HB_4      0x51
#define EMC142x_ED_LL_HB_5      0x55
#define EMC142x_ED_LL_HB_6      0x59
#define EMC142x_ED_LL_HB_7      0x5d

/* The Limit register for shutdown */
#define EMC142x_ID_THERM_LIMIT      0x20
#define EMC142x_ED_THERM_LIMIT_1    0x19
#define EMC142x_ED_THERM_LIMIT_2    0x1a
#define EMC142x_ED_THERM_LIMIT_3    0x30
#define EMC142x_ED_THERM_LIMIT_4    0x64
#define EMC142x_ED_THERM_LIMIT_5    0x65
#define EMC142x_ED_THERM_LIMIT_6    0x66
#define EMC142x_ED_THERM_LIMIT_7    0x67

/* The Status register*/
#define EMC142x_STATUS                0x02
#define EMC142x_FAULT_STATUS        0x1b
#define EMC142x_LOW_LIMIT_STATUS    0x36
#define EMC142x_HIGH_LIMIT_STATUS    0x35

/* Max no.of sensors in the emc1428 chip */
#define EMC142x_MAX_SENSORS     8

/* Mask values for various faults in the status register */
#define EMC142x_STATUS_FAULT_MASK         0x04
#define EMC142x_STATUS_LOW_LIMIT_MASK     0x08
#define EMC142x_STATUS_HIGH_LIMIT_MASK     0x10

/* Default power-on threshold values as per the data sheet */
#define EMC142x_DEFAULT_THIGH        0x55
#define EMC142x_DEFAULT_TLOW        0x00
#define EMC142x_DEFAULT_TCRITICAL    0x55

#endif //__SDI_EMC142x_REG_H_

