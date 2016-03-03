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
 * filename: sdi_pmbus_dev.h
 */


/****************************************************************************
 * @file    sdi_pmbus_dev.h
 * @brief  Declares data structure and common routines for PMbus device
 * As of now the temperature and Fan resources are supported. Any new
 * resource other than these two will be implemented whenever the requirement
 * arises.
 ****************************************************************************/

#ifndef __SDI_PMBUS_DEV_H_
#define __SDI_PMBUS_DEV_H_

#include "std_config_node.h"
#include "sdi_resource_internal.h"
#include "sdi_driver_internal.h"
#include "sdi_i2c.h"

/**
 * @enum sdi_pmbus_reg_t
 * Defines the register address for PMBUS devices
 */
typedef enum {
     /** Config Register for Fan1 and 2*/
    SDI_PMBUS_CMD_FAN_CONFIG_1_2 = 0x3A,
     /** Command Register for Fan1 */
    SDI_PMBUS_CMD_FAN_COMMAND_1 = 0x3B,
     /** Command Register for Fan2 */
    SDI_PMBUS_CMD_FAN_COMMAND_2 = 0x3C,
     /** Config Register for Fan3 and 4*/
    SDI_PMBUS_CMD_FAN_CONFIG_3_4 = 0x3D,
     /** Command Register for Fan3 */
    SDI_PMBUS_CMD_FAN_COMMAND_3 = 0x3E,
     /** Command Register for Fan4 */
    SDI_PMBUS_CMD_FAN_COMMAND_4 = 0x3F,
     /** Status Register for Fan1 and 2 */
    SDI_PMBUS_CMD_STATUS_FANS_1_2 = 0x81,
     /** Status Register for Fan3 and 4 */
    SDI_PMBUS_CMD_STATUS_FANS_3_4 = 0x82,
     /** Speed Register for Fan1 */
    SDI_PMBUS_CMD_READ_FAN_SPEED_1 = 0x90,
     /** Speed Register for Fan2 */
    SDI_PMBUS_CMD_READ_FAN_SPEED_2 = 0x91,
     /** Speed Register for Fan3 */
    SDI_PMBUS_CMD_READ_FAN_SPEED_3 = 0x92,
     /** Speed Register for Fan4 */
    SDI_PMBUS_CMD_READ_FAN_SPEED_4 = 0x93,
     /** Temperature Register Temperature-1*/
    SDI_PMBUS_CMD_READ_TEMPERATURE_1 = 0x8D,
     /** Temperature Register Temperature-2*/
    SDI_PMBUS_CMD_READ_TEMPERATURE_2 = 0x8E,
     /** Temperature Register Temperature-3*/
    SDI_PMBUS_CMD_READ_TEMPERATURE_3 = 0x8F
}sdi_pmbus_reg_t;

/**
 * @enum sdi_pmbus_resource_t
 * Defines the PMbus resources
 */
typedef enum {
    /** Resource is Temperature-1*/
    SDI_PMBUS_TEMPERATURE_1,
    /** Resource is Temperature-2*/
    SDI_PMBUS_TEMPERATURE_2,
    /** Resource is Temperature-3*/
    SDI_PMBUS_TEMPERATURE_3,
    /** Resource is Fan-1*/
    SDI_PMBUS_FAN_1,
    /** Resource is Fan-2*/
    SDI_PMBUS_FAN_2,
    /** Resource is Fan-3*/
    SDI_PMBUS_FAN_3,
    /** Resource is Fan-4*/
    SDI_PMBUS_FAN_4
}sdi_pmbus_resource_t;

/** Mask to find fault status of Fan-1 and Fan-3*/
#define SDI_FAN_1_3_FAULT_MASK        0x80
/** Mask to find fault status of Fan-2 and Fan-4*/
#define SDI_FAN_2_4_FAULT_MASK        0x40
/** Mask to find warning status of Fan-1 and Fan-3*/
#define SDI_FAN_1_3_WARNING_MASK        0x20
/** Mask to find warning status of Fan-2 and Fan-4*/
#define SDI_FAN_2_4_WARNING_MASK        0x10
/** Macro for the divider to convert RPM to duty cycle*/
#define SDI_FAN_RPM_TO_DUTY_CYCLE(x) (x/100)

/**
 * @enum pmbus_sensor_format_t
 * Different data formats supported by PMbus device
 */
typedef enum pmbus_sensor_format{
    /** Format is Linear*/
    SDI_PMBUS_LINEAR,
    /** Format is direct*/
    SDI_PMBUS_DIRECT
}pmbus_sensor_format_t;

/**
 * @struct sdi_pmbus_sensor_t
 * PMbus sensor definition
 */
typedef struct sdi_pmbus_sensor_ {
    sdi_pmbus_resource_t resource; /**<PMbus resource*/
    pmbus_sensor_format_t format;    /**<PMbus device data format*/
    char alias[SDI_MAX_NAME_LEN]; /**<Alias name of the sensor*/
}sdi_pmbus_sensor_t;

/**
 * @struct sdi_pmbus_dev_t
 * PMbus device definition
 */
typedef struct sdi_pmbus_dev_ {
    sdi_device_hdl_t dev; /**<device handle of the PMbus device */
    int max_sensors; /**<Maximum sensors supported */
    uint_t pec_req; /**<SMBUS PEC support requirement of the device*/
    sdi_pmbus_sensor_t *sdi_pmbus_sensors; /**<sensor definition of the device*/
    void *private_data;/**<Device private data*/
}sdi_pmbus_dev_t;

typedef struct sdi_pmbus_resource_hdl_ {
    int sensor_index;
    sdi_pmbus_dev_t * sdi_pmbus_dev_hdl;
}sdi_pmbus_resource_hdl_t;

/**
 * @brief  This is the register function for a PMBus device
 * @param[in] node - Config node for the device
 * @param[in] bus_handle - Parent bus handle of the device
 * @param[in] sdi_pmbus_device - device handle of a specific pmbus device
 * @param[in] driver_entry - driver entry structure for a specific pmbus device
 * @param[out] device_hdl - Device handle of the chip will be returned in this
 * @return - none
 */
void sdi_pmbus_dev_register(std_config_node_t node,void *bus_handle,
                            sdi_pmbus_dev_t *sdi_pmbus_device, sdi_driver_t *driver_entry,
                            sdi_device_hdl_t* device_hdl);

/*
 * @brief Does the chip level initialization for the PMbus devices.As of now only fan need a initialization.
 * @param[in] device_hdl - device handle of the specific pmbus device
 * @return: t_std_error
 */
t_std_error sdi_pmbus_dev_chip_init(void* device_hdl);

#endif //__SDI_PMBUS_DEV_H_
