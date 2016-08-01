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
 * filename: sdi_max6699.c
 */


/************************************************************************************************
 * sdi_max6699.c
 * Brief:
 * Implements the driver for max6699 temperature sensor. The MAX6699 precision
 * multichannel temperature sensor monitors its own temperature and the
 * temperatures of up to four external diode-connected transistors. All
 * temperature channels have programmable alert thresholds.
 * It also provides a register interface for reading temperature, getting and
 * setting the temperature threshold.
 * Sensor ID = 0 is for internal diode and it starts from 1 for external sensor diodes.
 * This driver exports each max6699 connected sensors as a resource to the
 * resource framework.
 * The data sheet of the MAX 6699 is available in the below link
 * http://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX6699.html
 ***********************************************************************************************/
#include "sdi_max6699_reg.h"
#include "sdi_thermal_internal.h"
#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_temperature_resource_attr.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_device_common.h"
#include "std_assert.h"
#include "std_utils.h"
#include "std_bit_masks.h"
#include "std_bit_ops.h"
#include <stdlib.h>
#include <stdio.h>

/*
 * MAX6699 device private data
 */
typedef struct max6699_device
{
    /* The bit mask of the sensors used in the specific platform*/
    STD_BIT_ARRAY_CREATE(connected_sensors, MAX6699_MAX_SENSORS);
    /* Store the current sensor limits */
    int high_threshold[MAX6699_MAX_SENSORS];
    /* Store the default sensor limits */
    int default_high_threshold[MAX6699_MAX_SENSORS];
    char *alias[MAX6699_MAX_SENSORS];
} max6699_device_t;

typedef struct max6699_resource_hdl
{
    sdi_device_hdl_t max6699_dev_hdl;
    uint_t sensor_id;
}max6699_resource_hdl_t;

/* Temperature registers for the sensors */
static uint8_t temp_reg[MAX6699_MAX_SENSORS] = {
    MAX6699_ID_DATA,
    MAX6699_ED_DATA_1,
    MAX6699_ED_DATA_2,
    MAX6699_ED_DATA_3,
    MAX6699_ED_DATA_4
};

/* Temperature High Limit registers for the sensors */
static uint8_t high_limit_reg[MAX6699_MAX_SENSORS] = {
    MAX6699_ID_HL,
    MAX6699_ED_HL_1,
    MAX6699_ED_HL_2,
    MAX6699_ED_HL_3,
    MAX6699_ED_HL_4
};

/* Defult high limit values for sensors on power-on */
static uint8_t high_limit_reg_por_value[MAX6699_MAX_SENSORS] = {
    MAX6699_ID_HL_POR_VALUE,
    MAX6699_ED_HL_POR_VALUE_1,
    MAX6699_ED_HL_POR_VALUE_2,
    MAX6699_ED_HL_POR_VALUE_3,
    MAX6699_ED_HL_POR_VALUE_4
};

/* Bit offset for High Temperature Alert and Diode fault status */
static uint8_t status_bit_mask[MAX6699_MAX_SENSORS] = {
    MAX6699_ID_HL_BIT,
    MAX6699_ED_HL_BIT_1,
    MAX6699_ED_HL_BIT_2,
    MAX6699_ED_HL_BIT_3,
    MAX6699_ED_HL_BIT_4
};

/*
 * This is the register function for a max6699 driver.
 */
static t_std_error sdi_max6699_register(std_config_node_t node, void *bus_handle,
                                 sdi_device_hdl_t* device_hdl);

/*
 * Does the initialization of threshold limits for each connected sesnor.
 */
static t_std_error sdi_max6699_init(sdi_device_hdl_t device_hdl);

/**
 * Read the temperature of the diode
 * resource_hdl[in] - Handle of the resource
 * temperature[out] - Buffer to hold the temperature value
 * return - standard t_std_error
 */
static t_std_error sdi_max6699_temperature_get(void *resource_hdl, int *temperature)
{
    uint8_t  buf = 0;
    uint_t sensor_id = 0;
    sdi_device_hdl_t dev_hdl = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    sensor_id = ((max6699_resource_hdl_t*)resource_hdl)->sensor_id;

    dev_hdl = ((max6699_resource_hdl_t*)resource_hdl)->max6699_dev_hdl;
    STD_ASSERT(dev_hdl != NULL);

    /*All the temperature values are returned in decimal value, so only one byte read is used */
    rc = sdi_smbus_read_byte(dev_hdl->bus_hdl, dev_hdl->addr.i2c_addr,
                             temp_reg[sensor_id], &buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("max6699 read failure at addr: %d reg: %d rc: %d\n",
                              dev_hdl->addr.i2c_addr,temp_reg[sensor_id],rc);
        return rc;
    }

    *temperature = (int) buf;

    return rc;
}

/**
 * Used to retrieve the threshold values of the sensor. Max6699 will support
 * only high threshold value
 * resource_hdl[in] - Handle of the resource
 * type[in] - threshold type
 * temperature[out] - threshold value
 * return - STD_ERR_OK for success or EOPNOTSUPP for un-supported threshold types
 */
static t_std_error sdi_max6699_threshold_get(void *resource_hdl, sdi_threshold_t type,
                                             int *temperature)
{
    uint_t sensor_id = 0;
    sdi_device_hdl_t dev_hdl = NULL;
    max6699_device_t *max6699_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    sensor_id = ((max6699_resource_hdl_t*)resource_hdl)->sensor_id;

    dev_hdl = ((max6699_resource_hdl_t*)resource_hdl)->max6699_dev_hdl;
    STD_ASSERT(dev_hdl != NULL);

    max6699_data = (max6699_device_t*)dev_hdl->private_data;
    STD_ASSERT(max6699_data != NULL);

    switch(type)
    {
        case SDI_HIGH_THRESHOLD:
            (*temperature) = max6699_data->high_threshold[sensor_id];
            break;

        case SDI_LOW_THRESHOLD:
        case SDI_CRITICAL_THRESHOLD:
        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    return rc;
}

/**
 * Used to set the threshold values for temperature sensor
 * resource_hdl[in] - Handle of the resource
 * type[in] - threshold type
 * temperature[in] - threshold value that needs to be set
 * return - STD_ERR_OK for success, respective error code from i2c API in case
 * of failure and EOPNOTSUPP for un-supported threshold types
 */
static t_std_error sdi_max6699_threshold_set(void *resource_hdl, sdi_threshold_t type,
                                             int temperature)
{
    uint_t sensor_id = 0;
    sdi_device_hdl_t dev_hdl = NULL;
    max6699_device_t *max6699_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    sensor_id = ((max6699_resource_hdl_t*)resource_hdl)->sensor_id;

    dev_hdl = ((max6699_resource_hdl_t*)resource_hdl)->max6699_dev_hdl;
    STD_ASSERT(dev_hdl != NULL);

    max6699_data = (max6699_device_t*)dev_hdl->private_data;
    STD_ASSERT(max6699_data != NULL);

    if(temperature & (~(0xff)))
    {
        SDI_DEVICE_ERRMSG_LOG("The value %d exceeds the range,it should be in"
                              "the range of -128 to 127",temperature);
        return SDI_DEVICE_ERRCODE(EINVAL);
    }

    switch(type)
    {
        case SDI_HIGH_THRESHOLD:
            rc = sdi_smbus_write_byte(dev_hdl->bus_hdl, dev_hdl->addr.i2c_addr,
                                      high_limit_reg[sensor_id], temperature,
                                      SDI_I2C_FLAG_NONE);
            if(rc != STD_ERR_OK) {
                SDI_DEVICE_ERRMSG_LOG("max6699 write failure at addr: %d reg: %d rc: %d\n",
                                      dev_hdl->addr.i2c_addr,high_limit_reg[sensor_id],rc);
                return rc;
            }
            max6699_data->high_threshold[sensor_id] = temperature;
            break;

        case SDI_LOW_THRESHOLD:
        case SDI_CRITICAL_THRESHOLD:
        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    return rc;
}

/**
 * Retrieve the fault status of the temperature sensor/diode
 * resource_hdl[in] - Handle of the resource
 * status[out] - status of the diode/sensor, false if diode is faulty else true
 * return - STD_ERR_OK for success or the respective error code from i2c API in
 * case of failure
 */
static t_std_error sdi_max6699_status_get(void *resource_hdl, bool *status)
{
    uint_t sensor_id = 0;
    uint8_t buf = 0;
    sdi_device_hdl_t dev_hdl = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    sensor_id = ((max6699_resource_hdl_t*)resource_hdl)->sensor_id;

    dev_hdl = ((max6699_resource_hdl_t*)resource_hdl)->max6699_dev_hdl;
    STD_ASSERT(dev_hdl != NULL);

    /*Read the fault status register and check for the specific sensor fault*/
    rc = sdi_smbus_read_byte(dev_hdl->bus_hdl, dev_hdl->addr.i2c_addr,
                             MAX6699_STATUS_3_REG, &buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("max6699 read failure at addr: %d rc: %d",
                              dev_hdl->addr.i2c_addr,rc);
        return rc;
    }
    /* fault bit set to 1 if respective diode is fault */
    *status = ( (STD_BIT_TEST(buf, status_bit_mask[sensor_id])) ? false : true );

    return rc;
}

/**
 * Init function will set the defult high limit values for each diode
 * resource_hdl[in] - Handle of the resource
 * return - STD_ERR_OK on success else std error on error cases
 */
static t_std_error sdi_max6699_resource_init(void *resource_hdl)
{
    uint_t sensor_id = 0;
    sdi_device_hdl_t dev_hdl = NULL;
    max6699_device_t *max6699_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    sensor_id = ((max6699_resource_hdl_t*)resource_hdl)->sensor_id;

    dev_hdl = ((max6699_resource_hdl_t*)resource_hdl)->max6699_dev_hdl;
    STD_ASSERT(dev_hdl != NULL);

    max6699_data = (max6699_device_t*)dev_hdl->private_data;
    STD_ASSERT(max6699_data != NULL);

    rc = sdi_max6699_threshold_set(resource_hdl,
                                   SDI_HIGH_THRESHOLD,
                                   max6699_data->default_high_threshold[sensor_id]);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("Failed to set high threshold for %s\n",
                              max6699_data->alias[sensor_id]);
        return rc;
    }
    max6699_data->high_threshold[sensor_id] = max6699_data->default_high_threshold[sensor_id];

    return rc;
}

/* Call back handler for MAX6699 */
static temperature_sensor_t max6699_sensor={
        sdi_max6699_resource_init,
        sdi_max6699_temperature_get,
        sdi_max6699_threshold_get,
        sdi_max6699_threshold_set,
        sdi_max6699_status_get
};

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t * sdi_max6699_entry_callbacks(void)
{
    /*Export Driver table*/
    static const sdi_driver_t max6699_entry = {
        sdi_max6699_register,
        sdi_max6699_init
    };

    return &max6699_entry;
}

/**
 * Creates the resource handle for a specific sensor
 * dev_hdl[in] - max6699 device handle
 * sensor_id[in] - diode id
 * return max6699_resource_hdl_t(resource handle for a specified diode in MAX6699)
 */
static max6699_resource_hdl_t *sdi_max6699_create_resource_hdl(sdi_device_hdl_t dev_hdl,
                                                               uint_t sensor_id)
{
    max6699_resource_hdl_t *max6699_resource = NULL;

    max6699_resource = calloc(sizeof(max6699_resource_hdl_t),1);
    STD_ASSERT(max6699_resource != NULL);

    max6699_resource->max6699_dev_hdl = dev_hdl;
    max6699_resource->sensor_id = sensor_id;

    return max6699_resource;

}

/**
 * Update the database for a specific sensor/diode
 * cur_node[in] - temperature sensor node
 * dev[in]      - device handle of max6699 device
 * return void
 */
static void sdi_max6699_device_database_init(std_config_node_t cur_node,
                                             void *dev)
{
    uint_t sensor_id = 0;
    char *node_attr = NULL;
    sdi_device_hdl_t dev_hdl = NULL;
    max6699_device_t *max6699_data = NULL;

    STD_ASSERT(dev != NULL);

    dev_hdl = (sdi_device_hdl_t)dev;
    STD_ASSERT(dev_hdl != NULL);

    max6699_data = (max6699_device_t*)dev_hdl->private_data;
    STD_ASSERT(max6699_data != NULL);

    if (strncmp(std_config_name_get(cur_node), SDI_DEV_NODE_TEMP_SENSOR,
                strlen(SDI_DEV_NODE_TEMP_SENSOR)) != 0) {
        return;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_INSTANCE);
    if(node_attr != NULL) {
        sensor_id = (uint_t) strtoul(node_attr, NULL, 0);
    }
    STD_ASSERT(sensor_id < MAX6699_MAX_SENSORS);

    STD_BIT_ARRAY_SET((max6699_data->connected_sensors), sensor_id);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_TEMP_HIGH_THRESHOLD);
    if(node_attr != NULL) {
        max6699_data->default_high_threshold[sensor_id] = (int) strtol(node_attr, NULL, 0);
    } else {
        max6699_data->default_high_threshold[sensor_id] = high_limit_reg_por_value[sensor_id];
    }

    node_attr=std_config_attr_get(cur_node, SDI_DEV_ATTR_ALIAS );
    if(node_attr == NULL) {
        max6699_data->alias[sensor_id] = calloc(SDI_MAX_NAME_LEN,1);
        snprintf(max6699_data->alias[sensor_id],SDI_MAX_NAME_LEN,
                 "max6699-%d-%d",dev_hdl->instance,sensor_id);
    } else{
        max6699_data->alias[sensor_id] = calloc((strlen(node_attr)+1),1);
        safestrncpy(max6699_data->alias[sensor_id],node_attr,(strlen(node_attr)+1));
    }

    sdi_resource_add(SDI_RESOURCE_TEMPERATURE,max6699_data->alias[sensor_id],
                     sdi_max6699_create_resource_hdl(dev_hdl,sensor_id),&max6699_sensor);
}

/* The configuration file format for the MAX6699 device node is as follows
 *<max6699 driver="max6699" instance="<dev_hdl_instance>" addr="<address of the dev_hdl>">
 *<temp_sensor instance="<sensor_no>" alias="<sensor alias>" high_threshold="<high threshold value>"
 *</max6699>
 * Mandatory attributes    : instance and addr
 */

/**
 * Register function for MAX6699 temperature sensor
 * node[in]         - Device node from configuration file
 * bus_handle[in]   - Parent bus handle of the device
 * device_hdl[out]  - Device handle
 * return           - t_std_error, this function is kept as non void as the
 *                    driver table function pointe requires it to be non-void
 */
static t_std_error sdi_max6699_register(std_config_node_t node, void *bus_handle,
                                        sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t dev_hdl = NULL;
    max6699_device_t *max6699_data = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_I2C_BUS);

    dev_hdl = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(dev_hdl != NULL);

    max6699_data = calloc(sizeof(max6699_device_t),1);
    STD_ASSERT(max6699_data != NULL);

    dev_hdl->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->addr.i2c_addr = (i2c_addr_t)strtoul(node_attr,NULL,16);

    dev_hdl->callbacks = sdi_max6699_entry_callbacks();
    dev_hdl->private_data = (void*)max6699_data;

    std_config_for_each_node(node, sdi_max6699_device_database_init, dev_hdl);

    *device_hdl = dev_hdl;

    return STD_ERR_OK;
}

/*
 * Does the dev_hdl level initialization for the max6699 device
 * [in] device_hdl - device handle of the specific device
 * Return - STD_ERR_OK in case of success or the corresponding error code in case of failure
 */
static t_std_error sdi_max6699_init(sdi_device_hdl_t device_hdl)
{

    uint_t sensor_id = 0;
    max6699_device_t *max6699_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    max6699_data = (max6699_device_t*)device_hdl->private_data;
    STD_ASSERT(max6699_data != NULL);

    for (sensor_id = 0; sensor_id < MAX6699_MAX_SENSORS; sensor_id++)
    {
        if(!STD_BIT_ARRAY_TEST((max6699_data->connected_sensors),sensor_id))
        {
            continue;
        }

        rc = sdi_max6699_resource_init(sdi_max6699_create_resource_hdl(device_hdl,sensor_id));
        if(rc != STD_ERR_OK)
        {
            SDI_DEVICE_ERRMSG_LOG("sdi_max6699_init failed for dev_hdl-%d\n",
                                  device_hdl->instance);
            return rc;
        }
    }
    return rc;
}
