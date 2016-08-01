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
 * filename: sdi_emc2305.h
 */


/*******************************************************************************
 * @file   sdi_emc2305.h
 * @brief  Defines the emc2305 driver functions.
 * The EMC2305 chip can control upto five fans.It supports two methods for fan
 * control viz RPM mode and Direct mode. This driver supports only the Direct
 * mode. The device provides the register interface for configuration and status
 * control of the fans.
 * For more information about the EMC2305, refer the data sheet in below link.
 * http://ww1.microchip.com/downloads/en/DeviceDoc/2305.pdf
 ******************************************************************************/

#ifndef _SDI_EMC2305_H_
#define _SDI_EMC2305_H_

/** Max number of fans */
#define EMC2305_MAX_FANS                    5

enum {
     EMC2305_FAN_CONFIG_REG = 0x20,
     EMC2305_FAN_STATUS_REG = 0x24,
     EMC2305_FAN_STALL_STATUS_REG = 0x25,
     EMC2305_FAN_SPIN_STATUS_REG = 0x26,
     EMC2305_FAN_DRIVE_FAIL_STATUS_REG = 0x27,
     EMC2305_FAN_INTERRUPT_EN_REG = 0x29,
     EMC2305_FAN_PWM_OUTPUT_REG = 0x2b,
     EMC2305_FAN_PWM1_BASE_FREQ_REG = 0x2c,
     EMC2305_FAN_PWM2_BASE_FREQ_REG = 0x2d,
     EMC2305_FAN0_SETTING_REG = 0x30,
     EMC2305_FAN0_CFG1_REG = 0x32,
     EMC2305_FAN0_CFG2_REG = 0x33,
     EMC2305_FAN0_TACH_LTARG_REG = 0x3c,
     EMC2305_FAN0_TACH_TARG_REG = 0x3d,
     EMC2305_FAN0_TACH_REG = 0x3e,
     EMC2305_FAN0_TACH_LBYTE_REG = 0x3f,
     EMC2305_FAN1_SETTING_REG = 0x40,
     EMC2305_FAN1_CFG1_REG = 0x42,
     EMC2305_FAN1_CFG2_REG = 0x43,
     EMC2305_FAN1_TACH_LTARG_REG = 0x4c,
     EMC2305_FAN1_TACH_TARG_REG = 0x4d,
     EMC2305_FAN1_TACH_REG = 0x4e,
     EMC2305_FAN1_TACH_LBYTE_REG = 0x4f,
     EMC2305_FAN2_SETTING_REG = 0x50,
     EMC2305_FAN2_CFG1_REG = 0x52,
     EMC2305_FAN2_CFG2_REG = 0x53,
     EMC2305_FAN2_TACH_LTARG_REG = 0x5c,
     EMC2305_FAN2_TACH_TARG_REG = 0x5d,
     EMC2305_FAN2_TACH_REG = 0x5e,
     EMC2305_FAN2_TACH_LBYTE_REG = 0x5f,
     EMC2305_FAN3_SETTING_REG = 0x60,
     EMC2305_FAN3_CFG1_REG = 0x62,
     EMC2305_FAN3_CFG2_REG = 0x63,
     EMC2305_FAN3_TACH_LTARG_REG = 0x6c,
     EMC2305_FAN3_TACH_TARG_REG = 0x6d,
     EMC2305_FAN3_TACH_REG = 0x6e,
     EMC2305_FAN3_TACH_LBYTE_REG = 0x6f,
     EMC2305_FAN4_SETTING_REG = 0x70,
     EMC2305_FAN4_CFG1_REG = 0x72,
     EMC2305_FAN4_CFG2_REG = 0x73,
     EMC2305_FAN4_TACH_LTARG_REG = 0x7c,
     EMC2305_FAN4_TACH_TARG_REG = 0x7d,
     EMC2305_FAN4_TACH_REG = 0x7e,
     EMC2305_FAN4_TACH_LBYTE_REG = 0x7f,
} sdi_emc2305_registers;

typedef enum {
    EMC2305_FAN_CONTROL_DIRECT = 0x2b,
    EMC2305_FAN_CONTROL_RPM = 0xab,
} sdi_emc2305_fan_control;

/** Register values */
#define EMC2305_FAN_CONFIG_REG_VAL          0x40
#define EMC2305_FAN_CONFIG1_REG_VAL         0x2b
#define EMC2305_FAN_CONFIG2_REG_VAL         0x59
#define EMC2305_FAN_INTERRUPT_EN_VAL        0x1f
#define EMC2305_FAN_PWM_OUTPUT_CFG_VAL      0x1f
#define EMC2305_FAN_PWM1_BASE_FREQ_VAL      0x0f
#define EMC2305_FAN_PWM2_BASE_FREQ_VAL      0x3f

/** Other macros */
#define EMC2305_INDEX0                      0
#define EMC2305_INDEX1                      1
#define EMC2305_TACH_FREQ                   32768 /* 32.768KHz */
#define EMC2305_PERCENT_VAL                 100
#define EMC2305_FAN_POLE_MIN                0x1
#define EMC2305_FAN_POLE_MAX                0x4
#define EMC2305_RPM_CONST_VAL               60
#define EMC2305_FAN_POLE_MASK               0x18
#define EMC2305_FAN_RANGE_MASK              0x60
#define EMC2305_FAN_POLE1_EDGE              0x3
#define EMC2305_FAN_POLE2_EDGE              0x5
#define EMC2305_FAN_POLE3_EDGE              0x7
#define EMC2305_FAN_POLE4_EDGE              0x9
#define EMC2305_RANGE0_MIN_RPM              500
#define EMC2305_RANGE1_MIN_RPM              1000
#define EMC2305_RANGE2_MIN_RPM              2000
#define EMC2305_RANGE3_MIN_RPM              4000
#define EMC2305_TACH0_MULTIPLIER            1
#define EMC2305_TACH1_MULTIPLIER            2
#define EMC2305_TACH2_MULTIPLIER            4
#define EMC2305_TACH3_MULTIPLIER            8
#define EMC2305_FAN_SETTING_MAX_VAL         0xff
#define EMC2305_FAN_POLE_SHIFT_BITS         0x3
#define EMC2305_FAN_LTACH_SHIFT_BITS        0x3
#define EMC2305_FAN_RANGE_SHIFT_BITS        0x5
#define EMC2305_FAN_WATCHDOG_STATUS_MASK    0x80
#define EMC2305_FAN_RANGEX_DEFAULT          2

#endif //_SDI_EMC2305_H_
