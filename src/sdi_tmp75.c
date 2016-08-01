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
 * filename: sdi_tmp75.c
 */


/******************************************************************************
 * Implements the driver for tmp75 temperature sensor chip.
 * This chip has only one sensor, so it exports only one resource to the
 * resource framework.The chip provides register interface for setting high
 * and low temperature threshold limits
 *
 ******************************************************************************/

#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_tmp75_reg.h"
#include "sdi_i2c_bus_api.h"
#include "std_assert.h"
#include "std_utils.h"
#include "sdi_device_common.h"
#include "sdi_thermal_internal.h"
#include "sdi_temperature_resource_attr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * TMP75 device private data
 */
typedef struct tmp75_device
{
    /* Sensor threshold limits */
    int low_threshold;
    int high_threshold;
    /* Default sensor limits */
    int default_low_threshold;
    int default_high_threshold;
} tmp75_device_t;

/*Register and chip init function declarations for the tmp75 driver*/
t_std_error sdi_tmp75_register(std_config_node_t node,void *bus_handle,sdi_device_hdl_t* device_hdl);
t_std_error sdi_tmp75_chip_init(sdi_device_hdl_t device_hdl);

/*
 * Retrieve temperature of the chip refered by resource.
 * This is a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [out] temperature - pointer to a buffer to ge the temperature of the chip.
 * Return - STD_ERR_OK for success and the respective error code from i2c api in case of failure
 */
static t_std_error sdi_tmp75_temperature_get(void *resource_hdl, int *temperature)
{
    uint8_t buf = 0;
    sdi_device_hdl_t chip = NULL;
    tmp75_device_t *tmp75_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp75_data = (tmp75_device_t*)chip->private_data;
    STD_ASSERT(tmp75_data != NULL);

    rc = sdi_smbus_read_byte(chip->bus_hdl,chip->addr.i2c_addr,TMP75_TEMP_REG,
                &buf,SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("tmp75 read failure at addr: %d reg: %d rc: %d\n",
                chip->addr.i2c_addr,TMP75_TEMP_REG,rc);
        return rc;
    }

    *temperature = (int)buf;

    return rc;
}

/*
 * Retrieve threshold values of the chip refered by resource.
 * This is a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] type - type of threshold (low or high)
 * [out] temperature - pointer to a buffer to get the temperature threshold value of the chip.
 * Return - STD_ERR_OK for success or EOPNOTSUPP in case of not supported threshold types
 */
static t_std_error sdi_tmp75_threshold_get(void *resource_hdl, sdi_threshold_t type, int *temperature)
{
    sdi_device_hdl_t chip = NULL;
    tmp75_device_t *tmp75_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp75_data = (tmp75_device_t*)chip->private_data;
    STD_ASSERT(tmp75_data != NULL);

    switch(type)
    {
        case SDI_LOW_THRESHOLD:
            *temperature = tmp75_data->low_threshold;
            break;

        case SDI_HIGH_THRESHOLD:
            *temperature = tmp75_data->high_threshold;
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }
    return rc;

}

/*
 * Set the threshold values for a chip refered by resource.
 * This is also a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] type - type of threshold (low or high)
 * [in] temperature - threshold value to be set for a given chip.
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_tmp75_threshold_set(void *resource_hdl, sdi_threshold_t type, int temperature)
{
    sdi_device_hdl_t chip = NULL;
    tmp75_device_t *tmp75_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp75_data = (tmp75_device_t*)chip->private_data;
    STD_ASSERT(tmp75_data != NULL);

    if(temperature & (~(0xff)))
    {
        return SDI_DEVICE_ERRCODE(EINVAL);
    }

    switch(type)
    {
        case SDI_LOW_THRESHOLD:
            tmp75_data->low_threshold = temperature;
            rc = sdi_smbus_write_byte(chip->bus_hdl,
                    (sdi_i2c_addr_t)chip->addr.i2c_addr,TMP75_TLOW_REG,
                        temperature,SDI_I2C_FLAG_NONE);
            if(rc != STD_ERR_OK)
            {
                SDI_DEVICE_ERRMSG_LOG("tmp75 write failure at addr: %d"
                    "reg: %d rc: %d\n",chip->addr.i2c_addr,TMP75_TLOW_REG,rc);
                return rc;
            }
            break;

        case SDI_HIGH_THRESHOLD:
            tmp75_data->high_threshold = temperature;
            rc = sdi_smbus_write_byte(chip->bus_hdl,
                    (sdi_i2c_addr_t)chip->addr.i2c_addr,TMP75_THIGH_REG,
                        temperature,SDI_I2C_FLAG_NONE);
            if(rc != STD_ERR_OK)
            {
                SDI_DEVICE_ERRMSG_LOG("tmp75 write failure at addr: %d"
                    "reg: %d rc: %d\n",chip->addr.i2c_addr,TMP75_THIGH_REG,rc);
                return rc;
            }
            break;
        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }
    return rc;
}

/*
 * Get the fault status of the chip refered by resource.
 * This is also a callback function for temperature sensor resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] status - pointer to a buffer to get the status of the chip
 * Return - STD_ERR_OK for success and the respective error code from i2c API in case of failure
 */
static t_std_error sdi_tmp75_status_get(void *resource_hdl, bool *status)
{
    sdi_device_hdl_t chip = NULL;
    tmp75_device_t *tmp75_data = NULL;
    t_std_error rc = STD_ERR_OK;
    int temperature = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    chip = (sdi_device_hdl_t)resource_hdl;

    tmp75_data = (tmp75_device_t*)chip->private_data;
    STD_ASSERT(tmp75_data != NULL);

    rc = sdi_tmp75_temperature_get(resource_hdl,&temperature);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_tmp75_temperature_get failure for %s\n",chip->alias);
        return rc;
    }

    *status = ((temperature >= tmp75_data->high_threshold) ||
            (temperature <= tmp75_data->low_threshold)) ? true : false;

    return rc;
}

temperature_sensor_t tmp75_sensor={
        NULL, /*As the init is done as part of chip init, resource init is not required*/
        sdi_tmp75_temperature_get,
        sdi_tmp75_threshold_get,
        sdi_tmp75_threshold_set,
        sdi_tmp75_status_get
};

/* Export the Driver table */
sdi_driver_t tmp75_entry={
        sdi_tmp75_register,
        sdi_tmp75_chip_init
};


/*
 * The config file format will be as below for tmp75 devices
 *
 * <tmp75 instance="<chip_instance>"
 * addr="<Address of the device>"
 * low_threshold="<low threshold value>" high_threshold="<high threshold value>"
 * alias="<Alias name for the particular devide>">
 * </tmp75>
 * Mandatory attributes    : instance and addr
 */
 /*
 * This the call back function for the device registration
 * [in] node - Config node for the device
 * [in] bus_handle - Parent bus handle of the device
 * [out] device_hdl - pointer to the device handle which will be updated by this function
 * Return - STD_ERR_OK, this function is kept as non void as the driver table fp requires it
 */
t_std_error sdi_tmp75_register(std_config_node_t node,void *bus_handle,sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    tmp75_device_t *tmp75_data = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_I2C_BUS);

    chip = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(chip != NULL);

    tmp75_data = calloc(sizeof(tmp75_device_t),1);
    STD_ASSERT(tmp75_data != NULL);

    chip->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    chip->addr.i2c_addr = (i2c_addr_t) strtoul(node_attr, NULL, 16);

    chip->callbacks = &tmp75_entry;
    chip->private_data = (void*)tmp75_data;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL)
    {
        snprintf(chip->alias,SDI_MAX_NAME_LEN,"tmp75-%d", chip->instance );
    }
    else
    {
        safestrncpy(chip->alias,node_attr,SDI_MAX_NAME_LEN);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_TEMP_LOW_THRESHOLD);
    if(node_attr != NULL)
    {
        tmp75_data->default_low_threshold = (int) strtol(node_attr, NULL, 0);
    }
    else
    {
        tmp75_data->default_low_threshold = TMP75_DEFAULT_TLOW;
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_TEMP_HIGH_THRESHOLD);
    if(node_attr != NULL)
    {
        tmp75_data->default_high_threshold = (int) strtol(node_attr, NULL, 0);
    }
    else
    {
        tmp75_data->default_high_threshold = TMP75_DEFAULT_THIGH;
    }

    sdi_resource_add(SDI_RESOURCE_TEMPERATURE,chip->alias,(void*)chip,
            &tmp75_sensor);

    *device_hdl = chip;

    return STD_ERR_OK;
}


t_std_error sdi_tmp75_chip_init(sdi_device_hdl_t device_hdl)
{
    tmp75_device_t *tmp75_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    tmp75_data = (tmp75_device_t*)device_hdl->private_data;
    STD_ASSERT(tmp75_data != NULL);

    rc = sdi_tmp75_threshold_set((void*)device_hdl,SDI_LOW_THRESHOLD,tmp75_data->default_low_threshold);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("Init failed for chip-%d",device_hdl->instance);
        return rc;
    }

    tmp75_data->low_threshold = tmp75_data->default_low_threshold;

    rc = sdi_tmp75_threshold_set((void*)device_hdl,SDI_HIGH_THRESHOLD,tmp75_data->default_high_threshold);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("Init failed for chip-%d",device_hdl->instance);
        return rc;
    }

    tmp75_data->high_threshold = tmp75_data->default_high_threshold;

    return rc;
}

