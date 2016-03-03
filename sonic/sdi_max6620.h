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
 * filename: sdi_max6620.h
 */


/************************************************************************************
 * @file    sdi_max6620.h
 * @brief  Defines the max6620 driver functions.
 * The MAX6620 chip can control up to four fans.It supports two methods for
 * fan control viz RPM mode and DAC mode. This driver supports only the RPM mode as
 * the current APIs are in RPM mode. * The device provides the register interface
 * for configuration and status control of the fans.For more information about the
 * MAX6620, refer the data sheet in below link
 * http://datasheets.maximintegrated.com/en/ds/MAX6620.pdf
*************************************** *********************************************/

#ifndef __SDI_MAX6620_H_
#define __SDI_MAX6620_H_

/** Max number of fans */
#define  MAX6620_MAX_FANS  4

/** Max TACH Count */
#define MAX6620_MAX_TACH_COUNT 2047

/** Register address */
#define MAX6620_GLOBALCFG   0x0 /* rw */
#define MAX6620_FANFAULT    0x1 /* rw */

#define MAX6620_FAN1CFG     0x2 /* rw */
#define MAX6620_FAN1DYN     0x6 /* rw */

/**
 * Fan config registers are contiguous, so the below macro can be used to access the register
 * corresponding to the Fan
 * Macro is expanded as (Fan-1's Register Address+Fan.No)
 * These registers are read/write registers
 * Eg: MAX6620_FANCFG(0) -->0x2 and MAX6620_FANCFG(1) -->0x3
 */
#define MAX6620_FANCFG(x)    (MAX6620_FAN1CFG + x)

/**
 * Fan Dynamic registers are contiguous, so the below macro can be used to access the register
 * corresponding to the Fan
 * Macro is expanded as (Fan-1's Register Address+Fan.No)
 * These registers are read/write registers
 * Eg: MAX6620_FANDYN(0) -->0x6 and MAX6620_FANCFG(1) -->0x7
 */
#define MAX6620_FANDYN(x)    (MAX6620_FAN1DYN + x)


/**
 * Following registers are 2 byte I2C transactions,
 * MSB & LSB should be read/write without anyother I2C write in between them
 */
#define MAX6620_FAN1TACHCNT     0x10 /* ro */
#define MAX6620_FAN1DRVVOLT     0x18 /* ro */
#define MAX6620_FAN1TGTTACHCNT  0x20 /* rw */
#define MAX6620_FAN1TGTDRVVOLT  0x28 /* rw */

/**
 * Below macro can be used to access the register corresponding to the Fan.
 * Macro is expanded as (Fan-1's Register Address+(Fan.No*2))
 * These registers are read only registers
 * Eg: MAX6620_FANTACHCNT(0) -->0x10 and MAX6620_FANTACHCNT(1) -->0x12
 */
#define MAX6620_FANTACHCNT(x) (MAX6620_FAN1TACHCNT +((x)<<1))

/**
 * Below macro can be used to access the register corresponding to the Fan.
 * Macro is expanded as (Fan-1's Register Address+(Fan.No*2))
 * These registers are read only registers
 * Eg: MAX6620_FAN1DRVVOLT(0) -->0x18 and MAX6620_FAN1DRVVOLT(1) -->0x1A
 */
#define MAX6620_FANDRVVOLT(x) (MAX6620_FAN1DRVVOLT +((x)<<1))

/**
 * Below macro can be used to access the register corresponding to the Fan.
 * Macro is expanded as (Fan-1's Register Address+(Fan.No*2))
 * These registers are read/write registers
 * Eg: MAX6620_FANTGTTACHCNT(0) -->0x20 and MAX6620_FANTGTTACHCNT(1) -->0x22
 */
#define MAX6620_FANTGTTACHCNT(x) (MAX6620_FAN1TGTTACHCNT +((x)<<1))

/**
 * Below macro can be used to access the register corresponding to the Fan.
 * Macro is expanded as (Fan-1's Register Address + (Fan.No*2))
 * These registers are read/write  registers
 * Eg: MAX6620_FANTGTDRVVOLT(0) -->0x28 and MAX6620_FANTGTDRVVOLT(1) -->0x2A
 */
#define MAX6620_FANTGTDRVVOLT(x) (MAX6620_FAN1TGTDRVVOLT +((x)<<1))

#define MAX6620_FAN_CTRL_SPEED_RANGE_MASK 0xE0
#define MAX6620_FAN_CTRL_DEV0     0

/** Different register masks for MAX6620_GLOBALCFG register */
#define MAX6620_FAN_CTRL_I2C_WTCHDOG_STS_MASK  (1 << 0)
#define MAX6620_FAN_CTRL_I2C_WTCHDOG_TM_MASK   ((1<<1)) | (1<<2))
#define MAX6620_FAN_CTRL_OSLTR_SEL_MASK        (1<<3)
#define MAX6620_FAN_CTRL_FAN_FULL_SPD_MASK     (1<<4)
#define MAX6620_FAN_CTRL_I2C_TMTOUT_MASK       (1<<5)
#define MAX6620_FAN_CTRL_RES_POR_MASK          (1<<6)
#define MAX6620_FAN_CTRL_RUN_STATE_MASK        (1<<7)

/** Fan fault masks for MAX6620_FANFAULT registers */
#define MAX6620_FAN_CTRL_FAN1_FAULT_MSK_MASK    (1 << 0)
#define MAX6620_FAN_CTRL_FAN2_FAULT_MSK_MASK    (1 << 1)
#define MAX6620_FAN_CTRL_FAN3_FAULT_MSK_MASK    (1 << 2)
#define MAX6620_FAN_CTRL_FAN4_FAULT_MSK_MASK    (1 << 3)
#define MAX6620_FAN_CTRL_FAN1_FAULT_STS_MASK    (1 << 4)
#define MAX6620_FAN_CTRL_FAN2_FAULT_STS_MASK    (1 << 5)
#define MAX6620_FAN_CTRL_FAN3_FAULT_STS_MASK    (1 << 6)
#define MAX6620_FAN_CTRL_FAN4_FAULT_STS_MASK    (1 << 7)

/** Fan fault masks for MAX6620_FANCFG registers */
#define MAX6620_FAN_CTRL_FAN1_FAULT_MSK_MASK       (1 << 0)
#define MAX6620_FAN_CTRL_LOCKED_ROTOR_POL_MASK     (1 << 1)
#define MAX6620_FAN_CTRL_LOCKED_ROTOR_CNTEN_MASK   (1 << 2)
#define MAX6620_FAN_CTRL_TACH_INPUT_EN_MASK        (1 << 3)
#define MAX6620_FAN_CTRL_SPIN_UP_MASK              ((1 << 5) | (1 << 6) )
#define MAX6620_FAN_CTRL_SPIN_UP_05S_MASK          (1 << 5)
#define MAX6620_FAN_CTRL_SPIN_UP_1S_MASK           (1 << 6)
#define MAX6620_FAN_CTRL_SPIN_UP_2S_MASK           ((1 << 5)| (1 << 6))
#define MAX6620_FAN_CTRL_RPM_MODE_EN_MASK          (1 << 7)


/** Fan fault masks for MAX6620_FANDYN registers */
#define MAX6620_FAN_CTRL_FAN1_FAULT_MSK_MASK    (1 << 0)
#define MAX6620_FAN_CTRL_DAC_ROC_MASK           0x1C /*28  - Bits 2,3,4 set */
#define MAX6620_FAN_CTRL_SPEED_RANGE_MASK       0xE0 /*224 - Bits 5,6,7 set */

#define MAX6620_DEFAULT_TACH_COUNT_PERIOD    0x04
#define MAX6620_FREQUENCY_HZ                 8192
#define MAX6620_DEFAULT_TACH_PULSE_COUNT     0x02

/** Macro to calculate the tach count */
#define TACH_COUNT_VAL(msb,lsb) ((lsb << 3) | (msb >> 5))

#endif //__SDI_MAX6620_H_
