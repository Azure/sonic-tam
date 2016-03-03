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
 * filename: sdi_tmp75_reg.h
 */


/************************************************************************************
 * @file sdi_tmp75_reg.h
 * @brief  Defines the tmp75 chip register macros for the use of tmp75 driver.
 * The chip provides register interface for setting high and low temperature
 * threshold limits. The data sheet of tmp75 is available in the below link
 * http://www.ti.com/lit/ds/symlink/tmp75.pdf
 ************************************************************************************/

#ifndef __SDI_TMP75_REG_H_
#define __SDI_TMP75_REG_H_

/**
 * TMP75 Register definitions
 */
#define TMP75_TEMP_REG        0x00
#define TMP75_CONFIG_REG      0x01
#define TMP75_TLOW_REG        0x02
#define TMP75_THIGH_REG       0x03

/**
 *The default threshold values for the chip
 */
#define TMP75_DEFAULT_THIGH    80
#define TMP75_DEFAULT_TLOW    70

#endif //__SDI_TMP75_REG_H_
