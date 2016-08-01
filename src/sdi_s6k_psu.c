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
 * filename: sdi_s6k_psu.c
 */


/*********************************************************************************************
 * sdi_s6k_psu.c
 * Implements the driver for a delta PSU devices which is used in s4000 and s6000.
 * This chip contains one temperature sensor and one Fan in it. It exports each of them
 * as resource to the resource framework.The device is complaint to PMBUS standard and
 * hence the pmbus operations are used to manipulate the device.
 *
 *********************************************************************************************/

#include "sdi_pmbus_dev.h"
#include "std_assert.h"
#include <stdlib.h>

sdi_pmbus_sensor_t sdi_pmbus_sensor_s6k[] = {
    {SDI_PMBUS_TEMPERATURE_1, SDI_PMBUS_LINEAR, "Temperature-1"},
    {SDI_PMBUS_FAN_1, SDI_PMBUS_LINEAR, "Fan-1"},
};

#define SDI_MAX_PMBUS_SENSORS_S6K sizeof(sdi_pmbus_sensor_s6k)/sizeof(sdi_pmbus_sensor_t)

/* Register function for s6k psu driver*/
t_std_error sdi_s6k_psu_register(std_config_node_t node, void *bus_handle,
                                 sdi_device_hdl_t* device_hdl);
/* Initialize the device*/
t_std_error sdi_s6k_psu_chip_init(sdi_device_hdl_t device_hdl);

/* Export the Driver table */
sdi_driver_t s6k_psu_entry = {
        sdi_s6k_psu_register,
        sdi_s6k_psu_chip_init
};

/*
 * The config file format will be as below for s6k psu devices
 *
 * <s6k_psu instance="<instance>" addr="<Address of the device>" fan_speed="<default fan speed>"
 * alias="<Alias name for the particular devide>"
 * </s6k_psu>
 */
/*
 * This the call back function for the device registration
 * [in] node - Config node for the device
 * [in] bus_handle - Parent bus handle of the device
 * [out] device_hdl - pointer to the device handle which will be updated by this function
 * Return - STD_ERR_OK, this function is kept as non void as the driver table
 * function pointer
 */
t_std_error sdi_s6k_psu_register(std_config_node_t node, void *bus_handle,
                                 sdi_device_hdl_t* device_hdl)
{
    sdi_pmbus_dev_t *sdi_pmbus_device = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);

    sdi_pmbus_device = calloc(sizeof(sdi_pmbus_dev_t),1);
    STD_ASSERT(sdi_pmbus_device != NULL);

    sdi_pmbus_device->max_sensors = SDI_MAX_PMBUS_SENSORS_S6K;
    sdi_pmbus_device->sdi_pmbus_sensors = sdi_pmbus_sensor_s6k;

    sdi_pmbus_dev_register(node, bus_handle, sdi_pmbus_device, &s6k_psu_entry, device_hdl);

    return STD_ERR_OK;
}

/*
 * This the call back function for the device initialization
 * [in] device_hdl - Device handle for which the init to be done
 * return - std_error_t
 */
t_std_error sdi_s6k_psu_chip_init(sdi_device_hdl_t device_hdl)
{
    return STD_ERR_OK;
}

