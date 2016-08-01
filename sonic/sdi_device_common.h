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
 * filename: sdi_device_common.h
 */


/******************************************************************************
 * @file sdi_device_common.h
 * @brief   Common header file sdi-device-driver module
 *****************************************************************************/
#ifndef __SDI_DEVICE_COMMON
#define __SDI_DEVICE_COMMON
#include "std_error_codes.h"
#include "event_log.h"

/**
  * @def Attribute used to define the maximum key size
  */
#define SDI_DEVICE_MAX_KEY_SIZE 100
/**
 * @def Attribute used for representing the module type
 */
#define MODULE        "SDI_DEVICE_MODULE"
/**
 * @def Attribute used to log the errno for SDI module
 */
#define SDI_DEVICE_ERRNO_LOG()                 EV_LOG_ERRNO(ev_log_t_BOARD, ev_log_s_CRITICAL, MODULE, errno)
/**
 * @def Attribute used to log the errno to the error log
 */
#define SDI_DEVICE_TRACEMSG_LOG(format, args...) EV_LOG_TRACE(ev_log_t_BOARD, 3, MODULE, format, args)
/**
 * @def Attribute used to log an error event for SDI module
 */
#define SDI_DEVICE_ERRMSG_LOG(format, args...) EV_LOG_ERR(ev_log_t_BOARD, ev_log_s_CRITICAL, MODULE, format, args)
/**
 * @def Attribute used to map driver return codes to standard error numbers in fail cases
 */
#define SDI_DEVICE_ERRNO                   STD_ERR_MK(e_std_err_BOARD, e_std_err_code_FAIL, errno)
/**
 * @def Attribute used to map SDI sub-system return codes with standard error numbers in
 * cases of invalid permissions and functinality not supported
 */
#define SDI_DEVICE_ERRCODE(errcode)        STD_ERR_MK(e_std_err_BOARD, e_std_err_code_FAIL, errcode)
/**
 * @def Attribute used to log an error when a wrong parameter is passed
 */
#define SDI_DEVICE_ERR_PARAM               STD_ERR_MK(e_std_err_BOARD, e_std_err_code_PARAM, 0)
/**
 * @def Attribute used to define sysfs path
 */
#define SYSFS_PATH      "/sys"

#endif /* __SDI_DEVICE_COMMON */
