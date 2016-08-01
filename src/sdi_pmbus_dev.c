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
 * filename: sdi_pmbus_dev.c
 */


/******************************************************************************
 * sdi_pmbus_dev.c
 * Implements routines to manipulate the PMBus devices.As of now the temperature
 * sensors and Fans are supported. Any new device other than these two will be
 * implemented whenever the requirement arises.
 * This file also exports each PMbus device sensors(Temperature sensor and Fan)
 * as a resource to the resource framework.
 *******************************************************************************/

#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_fan_internal.h"
#include "sdi_thermal_internal.h"
#include "sdi_pmbus_dev.h"
#include "sdi_fan_resource_attr.h"
#include "sdi_temperature_resource_attr.h"
#include "sdi_device_common.h"
#include "sdi_i2c_bus_api.h"
#include "std_assert.h"
#include "std_utils.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * Attribute used for representing the I2C flag of a device
 * @todo: Need to move this changes to i2c specfic files,
 * when i2c device structure is implemented.
 */
#define SDI_DEV_ATTR_I2C_FLAG    "i2c_flag"

/*
 * pmbus device private data
 */
typedef struct pmbus_dev_device_
{
    /* Default fan speed in RPM*/
    uint_t default_fan_speed;
    /* Maximum fan speed in RPM*/
    uint_t max_fan_speed;
} pmbus_dev_device_t;

/* Callback function to set the speed of the fan refered by resource */
static t_std_error sdi_pmbus_dev_fan_speed_set(void *resource_hdl, uint_t fan_speed);

/*
 * This is the function for resource init.
 * As of now only fan need a initialization.
 * [in] resource_hdl - resource handle of the specific resource
 * [in] max_speed - maximum speed of the Fan
 * Return - STD_ERR_OK for success or the respective error code from i2c API
 * in case of failure
 */
static t_std_error sdi_pmbus_dev_resource_init(void *resource_hdl, uint_t max_speed)
{
    sdi_pmbus_dev_t *pmbus_dev = NULL;
    pmbus_dev_device_t *pmbus_dev_data = NULL;
    t_std_error rc = STD_ERR_OK;

    pmbus_dev = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sdi_pmbus_dev_hdl;
    pmbus_dev_data =(pmbus_dev_device_t*)((sdi_device_hdl_t)(pmbus_dev->dev))->private_data;

    pmbus_dev_data->max_fan_speed = max_speed;

    rc = sdi_pmbus_dev_fan_speed_set(resource_hdl, pmbus_dev_data->default_fan_speed);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("PMBus dev fan speed set failed for sensor: %d \n",rc);
    }
    return rc;
}

/*
* The PMbus temperature sensors does not support the threshold settings and status getting
* So all the below APIs returns error code corresponding to operation not supported
*/
static t_std_error sdi_pmbus_dev_temp_threshold_get(void *resource_hdl, sdi_threshold_t type, int *temperature)
{
    return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
}

static t_std_error sdi_pmbus_dev_temp_threshold_set(void *resource_hdl, sdi_threshold_t type, int temperature)
{
    return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
}

static t_std_error sdi_pmbus_dev_temperature_status_get(void *resource_hdl, bool *status)
{
    return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
}

/*
 * Utility function to convert the linear data to the real value
 * PMbus data stored in Linear Data format will be as follows
 *
 *  <--------data high byte-------->|<---------data low byte-------->
 *  +---+---+-------+---+---+---+---+---+---+---+---+---+---+---+---+
 *  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 *  +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 *  <--------N---------> <--------------------Y--------------------->
 *  Where X = Y x 2^N ;
 *  X - is real world value
 *  Y - 11 bit 2's complement integer
 *  N - 5 bit 2's complement integer
 */
static inline int sdi_pmbus_linear_data_to_int(uint8_t user_data_msb, uint8_t user_data_lsb)
{
    int N = 0,Y = 0;

    N = ( user_data_msb >> 3 ) & 0x1F;
    N = ( N & (1<<4) ) ? (-((~N + 1) & 0x1F )) : N ;
    Y = user_data_msb & 0x7 ;
    Y = ( Y << 8 ) | user_data_lsb ;
    Y = ( Y & (1<<10) ) ? (-((~Y + 1) & 0x7FF)) : Y ;

    return ( Y * (1<<N));
}

/*
 * Callback function to retrieve the temperature of the sensor refered by resource
 * [in] resource_hdl - callback data for this function
 * [out] temperature - temperature will be returned in this
 * Return - STD_ERR_OK for success or the respective error code from i2c API
 * in case of failure
 */
static t_std_error sdi_pmbus_dev_temperature_get(void *resource_hdl, int *temperature)
{
    uint8_t buf[2] = {0};
    sdi_device_hdl_t chip = NULL;
    sdi_pmbus_dev_t *pmbus_dev = NULL;
    uint_t sensor_index = 0;
    uint_t pmbug_reg = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    sensor_index = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sensor_index;
    pmbus_dev = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sdi_pmbus_dev_hdl;
    chip = pmbus_dev->dev;

    switch(pmbus_dev->sdi_pmbus_sensors[sensor_index].resource)
    {
        case SDI_PMBUS_TEMPERATURE_1:
            pmbug_reg = SDI_PMBUS_CMD_READ_TEMPERATURE_1;
            break;
        case SDI_PMBUS_TEMPERATURE_2:
            pmbug_reg = SDI_PMBUS_CMD_READ_TEMPERATURE_2;
            break;
        case SDI_PMBUS_TEMPERATURE_3:
            pmbug_reg = SDI_PMBUS_CMD_READ_TEMPERATURE_3;
            break;
        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    rc = sdi_smbus_read_word(chip->bus_hdl, chip->addr.i2c_addr, pmbug_reg,
                             (uint16_t*)&buf, pmbus_dev->pec_req);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("pmbus device read failure at addr: %x reg: %x rc: %x\n",
                chip->addr.i2c_addr,pmbug_reg,rc);
        return rc;
    }

    switch(pmbus_dev->sdi_pmbus_sensors[sensor_index].format)
    {
        case SDI_PMBUS_LINEAR:
            *temperature = sdi_pmbus_linear_data_to_int(buf[1], buf[0]);
            break;
        case SDI_PMBUS_DIRECT:
            *temperature = (int)(*buf);
            break;
        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    return rc;
}

/*
 * Callback function to retrieve the speed of the fan refered by resource
 * [in] resource_hdl - callback data for this function
 * [out] fan_speed - Speed of the fan will be returned in this
 * Return - STD_ERR_OK for success or the respective error code from i2c API
 * in case of failure
 */
static t_std_error sdi_pmbus_dev_fan_speed_get(void *resource_hdl, uint_t *fan_speed)
{
    uint8_t buf[2] = {0};
    sdi_device_hdl_t chip = NULL;
    sdi_pmbus_dev_t *pmbus_dev = NULL;
    uint_t sensor_index = 0;
    uint_t pmbug_reg = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(fan_speed != NULL);

    sensor_index = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sensor_index;
    pmbus_dev = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sdi_pmbus_dev_hdl;
    chip = pmbus_dev->dev;

    switch(pmbus_dev->sdi_pmbus_sensors[sensor_index].resource)
    {
        case SDI_PMBUS_FAN_1:
            pmbug_reg = SDI_PMBUS_CMD_READ_FAN_SPEED_1;
            break;
        case SDI_PMBUS_FAN_2:
            pmbug_reg = SDI_PMBUS_CMD_READ_FAN_SPEED_2;
            break;
        case SDI_PMBUS_FAN_3:
            pmbug_reg = SDI_PMBUS_CMD_READ_FAN_SPEED_3;
            break;
        case SDI_PMBUS_FAN_4:
            pmbug_reg = SDI_PMBUS_CMD_READ_FAN_SPEED_4;
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    rc = sdi_smbus_read_word(chip->bus_hdl, chip->addr.i2c_addr, pmbug_reg,
                             (uint16_t*)&buf, pmbus_dev->pec_req);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("pmbus read failure at addr: %d reg: %d rc: %d\n",
                chip->addr.i2c_addr,pmbug_reg,rc);
        return rc;
    }

    switch(pmbus_dev->sdi_pmbus_sensors[sensor_index].format)
    {
        case SDI_PMBUS_LINEAR:
            *fan_speed = sdi_pmbus_linear_data_to_int(buf[1], buf[0]);
            break;
        case SDI_PMBUS_DIRECT:
            *fan_speed = (uint_t)(*buf);
            break;
        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }
    return rc;
}

/*
 * Callback function to set the speed of the fan refered by resource
 * Parameters:
 * [in] resource_hdl - callback data for this function
 * [in] fan_speed - Speed to be set
 * Return - STD_ERR_OK for success or the respective error code from i2c API
 * in case of failure
 */
static t_std_error sdi_pmbus_dev_fan_speed_set(void *resource_hdl, uint_t fan_speed)
{
    uint8_t buf[2] = {0};
    sdi_device_hdl_t chip = NULL;
    pmbus_dev_device_t* pmbus_dev_data = NULL;
    sdi_pmbus_dev_t *pmbus_dev = NULL;
    uint_t sensor_index = 0;
    uint_t pmbug_reg = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    sensor_index = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sensor_index;
    pmbus_dev = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sdi_pmbus_dev_hdl;
    chip = pmbus_dev->dev;
    pmbus_dev_data =(pmbus_dev_device_t*)chip->private_data;

    switch(pmbus_dev->sdi_pmbus_sensors[sensor_index].resource)
    {
        case SDI_PMBUS_FAN_1:
            pmbug_reg = SDI_PMBUS_CMD_FAN_COMMAND_1;
            break;
        case SDI_PMBUS_FAN_2:
            pmbug_reg = SDI_PMBUS_CMD_FAN_COMMAND_2;
            break;
        case SDI_PMBUS_FAN_3:
            pmbug_reg = SDI_PMBUS_CMD_FAN_COMMAND_3;
            break;
        case SDI_PMBUS_FAN_4:
            pmbug_reg = SDI_PMBUS_CMD_FAN_COMMAND_4;
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    /* Duty cycle */
    if(pmbus_dev_data->max_fan_speed != 0)
    {
        /*Store the speed percentage to write in to the pmbus register*/
        buf[0] = (fan_speed / SDI_FAN_RPM_TO_DUTY_CYCLE(pmbus_dev_data->max_fan_speed));
    }

    rc = sdi_smbus_write_word(chip->bus_hdl, chip->addr.i2c_addr, pmbug_reg,
                              *((uint16_t*)buf), pmbus_dev->pec_req);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("pmbus write failure at addr: %d reg: %d rc: %d\n",
                chip->addr.i2c_addr,pmbug_reg,rc);
    }

    return rc;
}

/*
 * Callback function to retrieve the status of the PSU
 * Parameters:
 * [in] resource_hdl - callback data for this function
 * [in] status - status will be returned in this
 * Return - STD_ERR_OK for success or the respective error code from i2c API
 * in case of failure
 */
static t_std_error sdi_pmbus_dev_fan_status_get(void *resource_hdl, bool *status)
{
    uint8_t buf = 0;
    sdi_device_hdl_t chip = NULL;
    sdi_pmbus_dev_t *pmbus_dev = NULL;
    uint_t sensor_index = 0;
    uint_t pmbug_reg = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    sensor_index = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sensor_index;
    pmbus_dev = ((sdi_pmbus_resource_hdl_t*)resource_hdl)->sdi_pmbus_dev_hdl;
    chip = pmbus_dev->dev;

    switch(pmbus_dev->sdi_pmbus_sensors[sensor_index].resource)
    {
        case SDI_PMBUS_FAN_1:
        case SDI_PMBUS_FAN_2:
            pmbug_reg = SDI_PMBUS_CMD_STATUS_FANS_1_2;
            break;
        case SDI_PMBUS_FAN_3:
        case SDI_PMBUS_FAN_4:
            pmbug_reg = SDI_PMBUS_CMD_STATUS_FANS_3_4;
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    rc = sdi_smbus_read_byte(chip->bus_hdl, chip->addr.i2c_addr, pmbug_reg,
                             &buf, pmbus_dev->pec_req);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("pmbus read failure at addr: %d reg: %d rc: %d\n",
                chip->addr.i2c_addr,pmbug_reg,rc);
        return rc;
    }

    switch(pmbus_dev->sdi_pmbus_sensors[sensor_index].resource)
    {
        case SDI_PMBUS_FAN_1:
        case SDI_PMBUS_FAN_3:
            /* The fan fault bit mask is same for Fan-1& Fan-3 and registers
             * are different*/
            *status = (buf & SDI_FAN_1_3_FAULT_MASK) ? true : false;
            break;
        case SDI_PMBUS_FAN_2:
        case SDI_PMBUS_FAN_4:
            /* The fan fault bit mask is same for Fan-2& Fan-4 and registers
             * are different*/
            *status = (buf & SDI_FAN_2_4_FAULT_MASK) ? true : false;
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    return rc;
}

/*Callback functions for the temperature resource*/
temperature_sensor_t pmbus_dev_temp_sensor = {
        NULL,
        sdi_pmbus_dev_temperature_get,
        sdi_pmbus_dev_temp_threshold_get,
        sdi_pmbus_dev_temp_threshold_set,
        sdi_pmbus_dev_temperature_status_get

};

/*Callback functions for the fan resource*/
fan_ctrl_t pmbus_dev_fan = {
        sdi_pmbus_dev_resource_init,
        sdi_pmbus_dev_fan_speed_get,
        sdi_pmbus_dev_fan_speed_set,
        sdi_pmbus_dev_fan_status_get
};

/*
 * Creates the resource handle for a specific sensor.
 * Parametrs:
 * [in] dev_hdl - PMbus device handle
 * [in] sensor_index - sensor index for a particular sensor
 * Return sdi_pmbus_resource_hdl_t - resource handle for the specific sensor
 */
static sdi_pmbus_resource_hdl_t *sdi_pmbus_create_resource_hdl(sdi_pmbus_dev_t *dev_hdl,
                                                               uint_t sensor_index)
{
    sdi_pmbus_resource_hdl_t *pmbus_resource = NULL;

    STD_ASSERT(dev_hdl != NULL);

    pmbus_resource = calloc(sizeof(sdi_pmbus_resource_hdl_t),1);
    STD_ASSERT(pmbus_resource != NULL);

    pmbus_resource->sdi_pmbus_dev_hdl = dev_hdl;
    pmbus_resource->sensor_index = sensor_index;

    return pmbus_resource;
}

/*
 * Export each sensor in the PMBus device as a resource.
 * Parametrs:
 * [in] pmbus_dev - PMbus device handle
 * Return - None
 */
static void sdi_pmbus_resource_add(sdi_pmbus_dev_t  *pmbus_dev )
{
    uint_t sensor_index = 0;
    void *callback_fns = NULL;
    char alias[SDI_MAX_NAME_LEN] = {0};
    int resource_type = -1;

    for(sensor_index = 0;sensor_index < pmbus_dev->max_sensors;sensor_index++)
    {
        if(pmbus_dev->sdi_pmbus_sensors[sensor_index].resource >= SDI_PMBUS_TEMPERATURE_1 &&
           pmbus_dev->sdi_pmbus_sensors[sensor_index].resource <= SDI_PMBUS_TEMPERATURE_3)
        {
            resource_type = SDI_RESOURCE_TEMPERATURE;
            callback_fns = &pmbus_dev_temp_sensor;
        }
        else if(pmbus_dev->sdi_pmbus_sensors[sensor_index].resource >= SDI_PMBUS_FAN_1 &&
                pmbus_dev->sdi_pmbus_sensors[sensor_index].resource <= SDI_PMBUS_FAN_4)
        {
            resource_type = SDI_RESOURCE_FAN;
            callback_fns = &pmbus_dev_fan;
        }
        else
        {
            continue;
        }

         snprintf(alias,SDI_MAX_NAME_LEN,"psu-%d-%s",
                  pmbus_dev->dev->instance,
                  pmbus_dev->sdi_pmbus_sensors[sensor_index].alias);

        sdi_resource_add(resource_type, alias,
                         (void*)sdi_pmbus_create_resource_hdl(pmbus_dev,sensor_index),
                         callback_fns);
    }
}

/*
 * The config file format will be as below for pmbus devices
 * <<PMbus device name> instance="<instance>" addr="<Address of the device>"
 * fan_speed="<default fan speed>" alias="<Alias name for the particular device></>"
 */

 /*
  * This the call back function for the device registration
  * [in] node - Config node for the device
  * [in] bus_handle - Parent bus handle of the device
  * [in] sdi_pmbus_device - device handle of a specific pmbus device
  * [in] driver_entry - driver entry structure for a specific pmbus device
  * [out] device_hdl - Device handle of the chip will be returned in this
  * return - void
  */
void sdi_pmbus_dev_register(std_config_node_t node,void *bus_handle,
                                   sdi_pmbus_dev_t *sdi_pmbus_device,
                                   sdi_driver_t *driver_entry,sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    pmbus_dev_device_t *pmbus_dev_data = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(sdi_pmbus_device != NULL);
    STD_ASSERT(driver_entry != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_I2C_BUS);

    chip = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(chip != NULL);

    pmbus_dev_data = calloc(sizeof(pmbus_dev_device_t),1);
    STD_ASSERT(pmbus_dev_data != NULL);

    chip->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    chip->addr.i2c_addr = (i2c_addr_t) strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_I2C_FLAG);
    if (node_attr != NULL) {
        sdi_pmbus_device->pec_req = (uint_t) strtoul(node_attr, NULL, 16);
    } else {
        /* @todo: currently except s3000, other platforms uses PEC flag by
         * default for PSU fan operations. hence, making it as default case.
         * In case, when modify all the configs, can use SDI_I2C_FLAG_NONE for
         * default.
         */
        sdi_pmbus_device->pec_req = SDI_I2C_FLAG_PEC;
    }

    chip->callbacks = driver_entry;
    chip->private_data = (void*)pmbus_dev_data;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL)
    {
        snprintf(chip->alias,SDI_MAX_NAME_LEN,"psu-%d", chip->instance );
    }
    else
    {
        safestrncpy(chip->alias,node_attr,SDI_MAX_NAME_LEN);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_FAN_SPEED);
    STD_ASSERT(node_attr != NULL);
    pmbus_dev_data->default_fan_speed = (uint_t) strtoul(node_attr, NULL, 0);

    sdi_pmbus_device->dev = chip;

    sdi_pmbus_resource_add(sdi_pmbus_device);

    *device_hdl = chip;

}
