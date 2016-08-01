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
 * filename: sdi_emc142x.c
 */


/************************************************************************************************
 * sdi_emc142x.c
 * Brief:
 * Implements the driver for emc142x family of temperature sensor chips. The emc 142x
 * chips can have up to eight sensor diode per chip, so that it can monitor up to seven external
 * temperature channels and 1 internal sensor diode.It also provides a register interface for
 * reading temperature, getting and setting the temperature threshold.
 * Sensor ID = 0 is for internal diode and it starts from 1 for external sensor diodes.
 * This driver exports each emc142x connected sensors as a resource to the
 * resource framework.
 * The data sheet of the EMC 1428 and 1422 are available in the below link
 * http://ww1.microchip.com/downloads/en/DeviceDoc/20005275A.pdf
 * http://ww1.microchip.com/downloads/en/DeviceDoc/1422.pdf
 ***********************************************************************************************/

#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_temperature_resource_attr.h"
#include "sdi_emc142x_reg.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_device_common.h"
#include "sdi_thermal_internal.h"
#include "std_assert.h"
#include "std_utils.h"
#include "std_bit_masks.h"
#include <stdlib.h>
#include <stdio.h>

/*
 * EMC142x device private data
 */
typedef struct emc142x_device
{
    /* The bit mask of the sensors used in the specific platform*/
    STD_BIT_ARRAY_CREATE(connected_sensors, EMC142x_MAX_SENSORS);
    /* Store the current sensor limits */
    int low_threshold[EMC142x_MAX_SENSORS];
    int high_threshold[EMC142x_MAX_SENSORS];
    int critical_threshold[EMC142x_MAX_SENSORS];
    /* Store the default sensor limits */
    int default_low_threshold[EMC142x_MAX_SENSORS];
    int default_high_threshold[EMC142x_MAX_SENSORS];
    int default_critical_threshold[EMC142x_MAX_SENSORS];
    char *alias[EMC142x_MAX_SENSORS];
} emc142x_device_t;

typedef struct emc142x_resource_hdl
{
    sdi_device_hdl_t emc142x_dev_hdl;
    uint_t sensor_id;
}emc142x_resource_hdl_t;

/* Temperature registers for the sensors */
static uint8_t temp_reg[EMC142x_MAX_SENSORS] = {
    EMC142x_ID_HB_DATA,
    EMC142x_ED_HB_DATA_1,
    EMC142x_ED_HB_DATA_2,
    EMC142x_ED_HB_DATA_3,
    EMC142x_ED_HB_DATA_4,
    EMC142x_ED_HB_DATA_5,
    EMC142x_ED_HB_DATA_6,
    EMC142x_ED_HB_DATA_7
};

/* Temperature High Limit registers for the sensors */
static uint8_t high_limit_reg[EMC142x_MAX_SENSORS] = {
    EMC142x_ID_HL,
    EMC142x_ED_HL_HB_1,
    EMC142x_ED_HL_HB_2,
    EMC142x_ED_HL_HB_3,
    EMC142x_ED_HL_HB_4,
    EMC142x_ED_HL_HB_5,
    EMC142x_ED_HL_HB_6,
    EMC142x_ED_HL_HB_7
};

/* Temperature Low Limit registers for the sensors */
static uint8_t low_limit_reg[EMC142x_MAX_SENSORS] = {
    EMC142x_ID_LL,
    EMC142x_ED_LL_HB_1,
    EMC142x_ED_LL_HB_2,
    EMC142x_ED_LL_HB_3,
    EMC142x_ED_LL_HB_4,
    EMC142x_ED_LL_HB_5,
    EMC142x_ED_LL_HB_6,
    EMC142x_ED_LL_HB_7
};

/* Temperature registers for the sensors */
static uint8_t critical_limit_reg[EMC142x_MAX_SENSORS] = {
    EMC142x_ID_THERM_LIMIT,
    EMC142x_ED_THERM_LIMIT_1,
    EMC142x_ED_THERM_LIMIT_2,
    EMC142x_ED_THERM_LIMIT_3,
    EMC142x_ED_THERM_LIMIT_4,
    EMC142x_ED_THERM_LIMIT_5,
    EMC142x_ED_THERM_LIMIT_6,
    EMC142x_ED_THERM_LIMIT_7
};

/*
 * This is the register function for a emc142x driver.
 */
t_std_error sdi_emc142x_register(std_config_node_t node,
                                 void *bus_handle,
                                 sdi_device_hdl_t* device_hdl);

/*
 * Does the initialization of threshold limits for each connected sesnor.
 */
t_std_error sdi_emc142x_chip_init(sdi_device_hdl_t device_hdl);

/*
 * Callback function to retrieve the temperature of the sensor/diode refered by resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [out] temperature - pointer to a buffer to get the temperature of the chip.
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_emc142x_temperature_get(void *resource_hdl, int *temperature)
{
    uint8_t  buf = 0;
    uint_t sensor_id = 0;
    sdi_device_hdl_t chip = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    sensor_id = ((emc142x_resource_hdl_t*)resource_hdl)->sensor_id;

    chip = ((emc142x_resource_hdl_t*)resource_hdl)->emc142x_dev_hdl;
    STD_ASSERT(chip != NULL);

    /*All the temperature values are returned in decimal value, so only one byte read is used */
    rc = sdi_smbus_read_byte(chip->bus_hdl,chip->addr.i2c_addr,
                             temp_reg[sensor_id],&buf,SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("emc142x read failure at addr: %d reg: %d rc: %d\n",
                              chip->addr.i2c_addr,temp_reg[sensor_id],rc);
        return rc;
    }

    *temperature = (int) buf;

    return rc;
}

/*
 * Callback function to retrieve the threshold values of the temperature sensor/diode
 * refered by resource
 * Parameter:
 * [in] callback_data - callback data for this function,chip instance is passed as a callback data
 * [in] type - type of threshold (low or high)
 * [out] temperature - pointer to a buffer to get the temperature threshold value of the chip.
 * Return - STD_ERR_OK for success or EOPNOTSUPP in case of not supported threshold types
 */
static t_std_error sdi_emc142x_threshold_get(void *resource_hdl, sdi_threshold_t type,
                                             int *temperature)
{
    uint_t sensor_id = 0;
    sdi_device_hdl_t chip = NULL;
    emc142x_device_t *emc142x_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(temperature != NULL);

    sensor_id = ((emc142x_resource_hdl_t*)resource_hdl)->sensor_id;

    chip = ((emc142x_resource_hdl_t*)resource_hdl)->emc142x_dev_hdl;
    STD_ASSERT(chip != NULL);

    emc142x_data = (emc142x_device_t*)chip->private_data;
    STD_ASSERT(emc142x_data != NULL);

    switch(type)
    {
        case SDI_LOW_THRESHOLD:
            (*temperature) = emc142x_data->low_threshold[sensor_id];
            break;

        case SDI_HIGH_THRESHOLD:
            (*temperature) = emc142x_data->high_threshold[sensor_id];
            break;

        case SDI_CRITICAL_THRESHOLD:
            (*temperature) = emc142x_data->critical_threshold[sensor_id];
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    return rc;
}

/*
 * Callback function to set the threshold values of the temperature sensor/diode
 * refered by resource
 * Parameters:
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] type - type of threshold (low or high)
 * [in] temperature - threshold value to be set for a given chip.
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_emc142x_threshold_set(void *resource_hdl, sdi_threshold_t type,
                                             int temperature)
{
    uint_t sensor_id = 0;
    sdi_device_hdl_t chip = NULL;
    emc142x_device_t *emc142x_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    sensor_id = ((emc142x_resource_hdl_t*)resource_hdl)->sensor_id;

    chip = ((emc142x_resource_hdl_t*)resource_hdl)->emc142x_dev_hdl;
    STD_ASSERT(chip != NULL);

    emc142x_data = (emc142x_device_t*)chip->private_data;
    STD_ASSERT(emc142x_data != NULL);

    if(temperature & (~(0xff)))
    {
        SDI_DEVICE_ERRMSG_LOG("The value %d exceeds the range,it should be in"
                              "the range of -256 to 255",temperature);
        return SDI_DEVICE_ERRCODE(EINVAL);
    }

    switch(type)
    {
        case SDI_LOW_THRESHOLD:
            rc = sdi_smbus_write_byte(chip->bus_hdl,
                                      chip->addr.i2c_addr,
                                      low_limit_reg[sensor_id],
                                      temperature,
                                      SDI_I2C_FLAG_NONE);
            if(rc != STD_ERR_OK)
            {
                SDI_DEVICE_ERRMSG_LOG("emc142x write failure at addr: %d reg: %d rc: %d\n",
                                      chip->addr.i2c_addr,low_limit_reg[sensor_id],rc);
                return rc;
            }
            emc142x_data->low_threshold[sensor_id] = temperature;
            break;

        case SDI_HIGH_THRESHOLD:
            rc = sdi_smbus_write_byte(chip->bus_hdl,
                                      chip->addr.i2c_addr,
                                      high_limit_reg[sensor_id],
                                      temperature,
                                      SDI_I2C_FLAG_NONE);
            if(rc != STD_ERR_OK)
            {
                SDI_DEVICE_ERRMSG_LOG("emc142x write failure at addr: %d reg: %d rc: %d\n",
                                      chip->addr.i2c_addr,high_limit_reg[sensor_id],rc);
                return rc;
            }
            emc142x_data->high_threshold[sensor_id] = temperature;
            break;

        case SDI_CRITICAL_THRESHOLD:
            rc = sdi_smbus_write_byte(chip->bus_hdl,
                                      chip->addr.i2c_addr,
                                      critical_limit_reg[sensor_id],
                                      temperature,
                                      SDI_I2C_FLAG_NONE);
            if(rc != STD_ERR_OK)
            {
                SDI_DEVICE_ERRMSG_LOG("emc142x write failure at addr: %d reg: %d rc: %d\n",
                                      chip->addr.i2c_addr,critical_limit_reg[sensor_id],rc);
                return rc;
            }
            emc142x_data->critical_threshold[sensor_id] = temperature;
            break;

        default:
            return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    return rc;
}

/*
 * Callback function to retrieve the fault status of the temperature sensor/diode refered by resource
 * Parameters:
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [out] status - pointer to a buffer to get the status of the chip
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_emc142x_status_get(void *resource_hdl, bool *status)
{
    uint_t sensor_id = 0;
    uint8_t buf = 0;
    sdi_device_hdl_t chip = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    sensor_id = ((emc142x_resource_hdl_t*)resource_hdl)->sensor_id;

    chip = ((emc142x_resource_hdl_t*)resource_hdl)->emc142x_dev_hdl;
    STD_ASSERT(chip != NULL);

    /*Read the fault status register and check for the specific sensor fault*/
    rc = sdi_smbus_read_byte(chip->bus_hdl,chip->addr.i2c_addr,EMC142x_FAULT_STATUS,
                             &buf,SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("emc142x read failure at addr: %d reg: %d rc: %d\n",
                              chip->addr.i2c_addr,EMC142x_FAULT_STATUS,rc);
        return rc;
    }
    *status = STD_BIT_ARRAY_TEST(&buf,sensor_id) ? true : false;

    return rc;
}

/*
 * Callback function to initialize the temperature of the sensor/diode refered by resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_emc142x_resource_init(void *resource_hdl)
{
    uint_t sensor_id = 0;
    sdi_device_hdl_t chip = NULL;
    emc142x_device_t *emc142x_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    sensor_id = ((emc142x_resource_hdl_t*)resource_hdl)->sensor_id;

    chip = ((emc142x_resource_hdl_t*)resource_hdl)->emc142x_dev_hdl;
    STD_ASSERT(chip != NULL);

    emc142x_data = (emc142x_device_t*)chip->private_data;
    STD_ASSERT(emc142x_data != NULL);

    rc = sdi_emc142x_threshold_set(resource_hdl,
                                   SDI_LOW_THRESHOLD,
                                   emc142x_data->default_low_threshold[sensor_id]);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("Failed to set low threshold for %s\n",
                              emc142x_data->alias[sensor_id]);
        return rc;
    }
    emc142x_data->low_threshold[sensor_id] = emc142x_data->default_low_threshold[sensor_id];

    rc = sdi_emc142x_threshold_set(resource_hdl,
                                   SDI_HIGH_THRESHOLD,
                                   emc142x_data->default_high_threshold[sensor_id]);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("Failed to set high threshold for %s\n",
                              emc142x_data->alias[sensor_id]);
        return rc;
    }
    emc142x_data->high_threshold[sensor_id] = emc142x_data->default_high_threshold[sensor_id];

    rc = sdi_emc142x_threshold_set(resource_hdl,
                                   SDI_CRITICAL_THRESHOLD,
                                   emc142x_data->default_critical_threshold[sensor_id]);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("Failed to set critical threshold for %s\n",
                              emc142x_data->alias[sensor_id]);
    }
    emc142x_data->critical_threshold[sensor_id] = emc142x_data->default_critical_threshold[sensor_id];

    return rc;
}

temperature_sensor_t emc142x_sensor={
        sdi_emc142x_resource_init,
        sdi_emc142x_temperature_get,
        sdi_emc142x_threshold_get,
        sdi_emc142x_threshold_set,
        sdi_emc142x_status_get

};

/* Export the Driver table */
sdi_driver_t emc142x_entry={
        sdi_emc142x_register,
        sdi_emc142x_chip_init
};

/*
 * Creates the resource handle for a specific sensor.
 * Parametrs:
 * [in] dev_hdl - emc142x device handle
 * [in] sensor_id - sensor id for the specific sensor
 * Return emc142x_resource_hdl_t - resource handle for the specific sensor
 */
static emc142x_resource_hdl_t *sdi_emc142x_create_resource_hdl(sdi_device_hdl_t dev_hdl,
                                                               uint_t sensor_id)
{
    emc142x_resource_hdl_t *emc142x_resource = NULL;

    emc142x_resource = calloc(sizeof(emc142x_resource_hdl_t),1);
    STD_ASSERT(emc142x_resource != NULL);

    emc142x_resource->emc142x_dev_hdl = dev_hdl;
    emc142x_resource->sensor_id = sensor_id;

    return emc142x_resource;

}

/*
 * Update the database for a specific sensor.
 * Parametrs:
 * [in] cur_node - temperature sensor node
 * [in] resource_hdl - device handle of emc142x device
 * Return - void
 */
static void sdi_emc142x_device_database_init(std_config_node_t cur_node, void *dev_hdl)
{
    uint_t sensor_id = 0;
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    emc142x_device_t *emc142x_data = NULL;
    size_t node_attr_len = 0;

    STD_ASSERT(dev_hdl != NULL);

    chip = (sdi_device_hdl_t)dev_hdl;
    STD_ASSERT(chip != NULL);

    emc142x_data = (emc142x_device_t*)chip->private_data;
    STD_ASSERT(emc142x_data != NULL);

    if (strncmp(std_config_name_get(cur_node), SDI_DEV_NODE_TEMP_SENSOR,
                strlen(SDI_DEV_NODE_TEMP_SENSOR)) != 0)
    {
        return;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_INSTANCE);
    if(node_attr != NULL)
    {
        sensor_id = (uint_t) strtoul(node_attr, NULL, 0);
    }
    STD_ASSERT(sensor_id < EMC142x_MAX_SENSORS);

    STD_BIT_ARRAY_SET((emc142x_data->connected_sensors),sensor_id);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_TEMP_LOW_THRESHOLD);
    if(node_attr != NULL)
    {
        emc142x_data->default_low_threshold[sensor_id] = (int) strtol(node_attr, NULL, 0);
    }
    else
    {
        emc142x_data->default_low_threshold[sensor_id] = EMC142x_DEFAULT_TLOW;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_TEMP_HIGH_THRESHOLD);
    if(node_attr != NULL)
    {
        emc142x_data->default_high_threshold[sensor_id] = (int) strtol(node_attr, NULL, 0);
    }
    else
    {
        emc142x_data->default_high_threshold[sensor_id] = EMC142x_DEFAULT_THIGH;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_TEMP_CRITICAL_THRESHOLD);
    if(node_attr != NULL)
    {
        emc142x_data->default_critical_threshold[sensor_id] = (int) strtol(node_attr, NULL, 0);
    }
    else
    {
        emc142x_data->default_critical_threshold[sensor_id] = EMC142x_DEFAULT_TCRITICAL;
    }

    node_attr=std_config_attr_get(cur_node, SDI_DEV_ATTR_ALIAS );
    if(node_attr == NULL)
    {
        emc142x_data->alias[sensor_id] = malloc(SDI_MAX_NAME_LEN);
        STD_ASSERT( emc142x_data->alias[sensor_id] != NULL);
        snprintf(emc142x_data->alias[sensor_id],SDI_MAX_NAME_LEN,
                     "emc142x-%d-%d",chip->instance,sensor_id);
    }
    else
    {
        node_attr_len = strlen(node_attr)+1;
        emc142x_data->alias[sensor_id] = malloc(node_attr_len);
        STD_ASSERT( emc142x_data->alias[sensor_id] != NULL);
        memcpy(emc142x_data->alias[sensor_id],node_attr,node_attr_len);
    }

    sdi_resource_add(SDI_RESOURCE_TEMPERATURE,emc142x_data->alias[sensor_id],
                        sdi_emc142x_create_resource_hdl(chip,sensor_id),&emc142x_sensor);
}

/* The configuration file format for the EMC142x device node is as follows
 *<emc142x driver="emc142x" instance="<chip_instance>" addr="<address of the chip>">
 *<temp_sensor instance="<sensor_no>" alias="<sensor alias>" low_threshold="<low threshold value>"
 *high_threshold="<high threshold value>"/>
 *<temp_sensor instance="<sensor_no>" alias="<sensor alias>" low_threshold="< low threshold value>"
 * high_threshold="< high threshold value >"/>
 *</emc142x>
 * Mandatory attributes    : instance and addr
 */

/* This is the registration function for emc142x driver.
 * Parameters:
 * [in] node - emc142x device node from the configuration file
 * [in] bus_handle - Parent bus handle of the device
 * [out] device_hdl - Pointer to the emc142x device handle which will get filled by this function
 * Return - STD_ERR_OK, this function is kept as non void as the driver table function pointer
 * requires it to be non-void
 */

t_std_error sdi_emc142x_register(std_config_node_t node, void *bus_handle, sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    emc142x_device_t *emc142x_data = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_I2C_BUS);

    chip = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(chip != NULL);

    emc142x_data = calloc(sizeof(emc142x_device_t),1);
    STD_ASSERT(emc142x_data != NULL);

    chip->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    chip->addr.i2c_addr = (i2c_addr_t)strtoul(node_attr,NULL,16);

    chip->callbacks = &emc142x_entry;
    chip->private_data = (void*)emc142x_data;

    std_config_for_each_node(node,sdi_emc142x_device_database_init,chip);

    *device_hdl = chip;

    return STD_ERR_OK;
}

/*
 * Does the chip level initialization for the emc142x device
 * [in] device_hdl - device handle of the specific device
 * Return - STD_ERR_OK in case of success or the corresponding error code in case of failure
 */
t_std_error sdi_emc142x_chip_init(sdi_device_hdl_t device_hdl)
{

    uint_t sensor_id = 0;
    emc142x_device_t *emc142x_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    emc142x_data = (emc142x_device_t*)device_hdl->private_data;
    STD_ASSERT(emc142x_data != NULL);

    for (sensor_id = 0; sensor_id < EMC142x_MAX_SENSORS; sensor_id++)
    {
        if(!STD_BIT_ARRAY_TEST((emc142x_data->connected_sensors),sensor_id))
        {
            continue;
        }

        rc = sdi_emc142x_resource_init(sdi_emc142x_create_resource_hdl(device_hdl,sensor_id));
        if(rc != STD_ERR_OK)
        {
            SDI_DEVICE_ERRMSG_LOG("sdi_emc142x_chip_init failed for chip-%d\n",
                                  device_hdl->instance);
            return rc;
        }
    }
    return rc;
}
