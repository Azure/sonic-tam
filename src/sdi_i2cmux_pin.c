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
 * filename: sdi_i2cmux_pin.c
 */


/******************************************************************************
 * Creates an i2cmux device and registers i2c bus for every mux channel.
 * i2c-mux-pin: its an i2c multiplexer, that multiplexes one i2c bus onto 'n'
 * i2c bus based on pin selection. This device is attached to pseudo bus as it
 * has references to i2c bus (being multiplexed) and pin bus (used for mux
 * selection).
 *
 * todo:
 * 1. auto-generation of instance id.
 * 2. error handling for bus init failures.
 *****************************************************************************/

/**
 * XML Format for i2cmux pin :
 * <sdi_i2cmux_pin instance="0" parent="smbus0" mux_pin_group_bus="i2cmux-gpio">
 *      <i2c channel=0 instance="2" bus_name="smbus0_0"></i2c>
 *      <i2c channel=1 instance="3" bus_name="smbus0_1"></i2c>
 *      <i2c channel=2 instance="4" bus_name="smbus0_2"></i2c>
 *      <i2c channel=3 instance="5" bus_name="smbus0_3"></i2c>
 * </sdi_i2cmux_pin>
 *
 */
/**
For ex: gpio pin1 and gpio pin2 are used to enable a particular i2c mux channel:
    channel     pin2     pin1
        0        0        0
        1        0        1
        2        1        0
        3        1        1
**/

#include "sdi_i2cmux_pin.h"
#include "sdi_i2c_mux_attr.h"
#include "sdi_common_attr.h"
#include "sdi_bus_attr.h"
#include "sdi_device_common.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_i2c_bus_framework.h"
#include "sdi_pin_group_bus_api.h"
#include "sdi_pin_group_bus_framework.h"
#include "std_utils.h"
#include "std_assert.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/**
 * sdi_i2cmux_pin_chan_acquire_bus
 * acquire i2c mux channel bus
 * sequence of operations:
 *  1. acquire mux device lock to prevent other access to mux device
 *  2. select the channel by updating the pin group with channel id for this bus.
 *  3. acquire this mux channel's bus lock
 *  4. acquire i2c bus to which this mux is attached.
 * param[in] bus_handle - i2c mux channel bus handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pin_chan_acquire_bus(sdi_i2c_bus_hdl_t bus_handle)
{
    t_std_error error = STD_ERR_OK;
    sdi_i2cmux_pin_chan_bus_handle_t bus = (sdi_i2cmux_pin_chan_bus_handle_t) bus_handle;
    sdi_i2cmux_pin_hdl_t mux = bus->i2c_mux;
    bool is_pin_group_bus_acquired = false;

    error = std_mutex_lock(&(mux->mux_lock));
    if (error != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d acquiring lock failed with error %d\n",
                              __FUNCTION__, __LINE__, error);
        return error;
    }

    do {
        error = sdi_pin_group_acquire_bus((sdi_pin_group_bus_hdl_t)mux->pingroup_hdl);
        if (error != STD_ERR_OK) {
            break;
        }

        is_pin_group_bus_acquired = true;

        error = sdi_pin_group_write_level(mux->pingroup_hdl, bus->i2c_mux_channel);
        if (error != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d channel select failed with error %d\n",
                    __FUNCTION__, __LINE__, error);
            break;
        }

        error = sdi_i2c_acquire_bus((bus->i2c_mux->i2cbus_hdl));
        if (error != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d acquiring bus failed with error %d\n",
                    __FUNCTION__, __LINE__, error);
            break;
        }
    } while(0);

    if (error != STD_ERR_OK) {
        if(is_pin_group_bus_acquired == true ) {
            sdi_pin_group_release_bus((sdi_pin_group_bus_hdl_t)mux->pingroup_hdl);
        }
        std_mutex_unlock (&(mux->mux_lock));
    }

    return error;
}

/**
 * sdi_i2cmux_pin_chan_release_bus
 * release i2c mux channel bus
 * sequence of operations:
 * 1. release i2c bus to which the mux is attached.
 * 2. release i2c mux channel bus
 * 3. release mux device lock.
 * param[in] bus_handle - i2c mux channel bus handle
 * return none
 */
static void sdi_i2cmux_pin_chan_release_bus(sdi_i2c_bus_hdl_t bus_handle)
{
    sdi_i2cmux_pin_chan_bus_handle_t bus = (sdi_i2cmux_pin_chan_bus_handle_t) bus_handle;
    sdi_i2cmux_pin_hdl_t mux = bus->i2c_mux;

    sdi_i2c_release_bus((mux->i2cbus_hdl));
    sdi_pin_group_release_bus((sdi_pin_group_bus_hdl_t)mux->pingroup_hdl);
    std_mutex_unlock (&(mux->mux_lock));
}

/**
 * sdi_i2cmux_pin_chan_get_capability
 * get the capability of i2c mux channel bus
 * param[out] capability - i2c mux channel bus capability
 * return none
 */
static void sdi_i2cmux_pin_chan_get_capability
    (sdi_i2c_bus_hdl_t bus_handle, sdi_i2c_bus_capability_t *capability)
{
    sdi_i2cmux_pin_chan_bus_handle_t bus = (sdi_i2cmux_pin_chan_bus_handle_t) bus_handle;

    sdi_i2c_bus_get_capability(bus->i2c_mux->i2cbus_hdl, capability);
}

/**
 * sdi_i2cmux_pin_chan_execute
 * execute i2c bus operation
 * param[in] bus_handle - i2c mux channel bus handle
 * param[in] address - i2c address of slave device
 * param[in] operation - i2c bus operation (read/write)
 * param[in] data_type - i2c bus transaction type (byte,word,block)
 * param[in] commandbuf - address offset
 * param[out] buffer - data read from/written to i2c slave
 * param[out] block_len - i2c block data size
 * param[in] flags - PEC
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pin_chan_execute (sdi_i2c_bus_hdl_t bus_handle,
                                                sdi_i2c_addr_t address,
                                                sdi_smbus_operation_t operation,
                                                sdi_smbus_data_type_t data_type,
                                                uint_t commandbuf, void *buffer,
                                                size_t *block_len, uint_t flags)
{
    sdi_i2cmux_pin_chan_bus_handle_t bus = (sdi_i2cmux_pin_chan_bus_handle_t) bus_handle;

    return sdi_smbus_execute(bus->i2c_mux->i2cbus_hdl, address, operation, data_type,
                             commandbuf, buffer, block_len, flags);
}

/**
 * sdi_i2cmux_chan_bus_operations
 * SDI I2C Bus Operations for I2C MUX channel bus
 */
sdi_i2c_bus_ops_t sdi_i2cmux_chan_bus_operations = {
    .sdi_i2c_acquire_bus = sdi_i2cmux_pin_chan_acquire_bus,
    .sdi_smbus_execute = sdi_i2cmux_pin_chan_execute,
    .sdi_i2c_release_bus = sdi_i2cmux_pin_chan_release_bus,
    .sdi_i2c_get_capability = sdi_i2cmux_pin_chan_get_capability,
};

/**
 * sdi_i2cmux_pin driver registration
 */
static t_std_error sdi_i2cmux_pin_driver_register(std_config_node_t node,
                                           void *bus_handle,
                                           sdi_device_hdl_t *device_hdl);

/**
 * sdi_i2cmux_pin driver initialization
 */
static t_std_error sdi_i2cmux_pin_driver_init(sdi_device_hdl_t device_hdl);

/**
 * sdi_i2cmux_pin_entry
 * SDI Driver Object to hold this driver's registration and initialization
 * function
 */
sdi_driver_t sdi_i2cmux_pin_entry = {
    sdi_i2cmux_pin_driver_register,
    sdi_i2cmux_pin_driver_init
};

/**
 * sdi_i2cmux_channel_init
 * Initialize i2c mux channel bus : initialize lock, i2c mux channel bus
 * operations, initialize i2c device list
 * param[in] bus_handle - i2c mux channel bus handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_channel_init(sdi_bus_hdl_t bus_handle)
{
    sdi_i2cmux_pin_chan_bus_handle_t i2cmux_chan_bus =
        (sdi_i2cmux_pin_chan_bus_handle_t) bus_handle;

    STD_ASSERT(i2cmux_chan_bus != NULL);

    i2cmux_chan_bus->i2c_bus.ops = &sdi_i2cmux_chan_bus_operations;

    sdi_bus_init_device_list(bus_handle);

    return STD_ERR_OK;
}

/**
 * sdi_i2cmux_pin_driver_register
 * Register i2c mux device and register every i2c mux channel on this i2c mux
 * param[in] node - i2c mux pin config node
 * param[in] bus_handle - pseudo bus handle
 * param[out] device_hdl - handle populated with i2c mux device handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pin_driver_register(std_config_node_t node,
                                           void *bus_handle,
                                           sdi_device_hdl_t *device_hdl)
{

    char *node_attr = NULL;
    sdi_bus_hdl_t bus = NULL;
    std_config_node_t cur_node = NULL;
    sdi_i2cmux_pin_chan_bus_handle_t i2cmux_chan_bus = NULL;
    sdi_device_hdl_t dev = NULL;

    sdi_i2cmux_pin_t *i2cmux = (sdi_i2cmux_pin_t *)
        calloc (sizeof(sdi_i2cmux_pin_t), 1);

    STD_ASSERT(i2cmux != NULL);

    sdi_bus_init_list(&i2cmux->channel_list);

    dev = (sdi_device_hdl_t) calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev != NULL);

    dev->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev->instance = (uint_t) strtoul (node_attr, NULL, 0);

    /* dev->addr is not used by this device */

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL) {
        snprintf(dev->alias,SDI_MAX_NAME_LEN,"sdi-i2cmux-pin-%u", dev->instance);
    } else {
        safestrncpy(dev->alias,node_attr,SDI_MAX_NAME_LEN);
    }

    dev->callbacks = &sdi_i2cmux_pin_entry;

    dev->private_data = (void *)i2cmux;

    node_attr = std_config_attr_get(node,
        SDI_DEV_ATTR_SDI_I2CMUX_PARENT_I2CBUS);
    STD_ASSERT(node_attr != NULL);
    safestrncpy(i2cmux->i2c_bus_name, node_attr, SDI_MAX_NAME_LEN);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_SDI_I2CMUX_PINBUS);
    STD_ASSERT(node_attr != NULL);
    safestrncpy(i2cmux->pingroup_bus_name, node_attr, SDI_MAX_NAME_LEN);

    if (device_hdl != NULL) {
        *device_hdl = dev;
    }

    for (cur_node = std_config_get_child(node); cur_node != NULL;
            cur_node = std_config_next_node(cur_node)) {

        SDI_DEVICE_TRACEMSG_LOG("%s:%d registering childnode %s\n",
                __FUNCTION__, __LINE__, std_config_name_get(cur_node));

        i2cmux_chan_bus = (sdi_i2cmux_pin_chan_bus_handle_t)
            calloc (sizeof(sdi_i2cmux_pin_chan_bus_t), 1);
        STD_ASSERT(i2cmux_chan_bus != NULL);

        node_attr = std_config_attr_get(cur_node,
                                        SDI_DEV_ATTR_SDI_I2CMUX_CHANNEL);
        STD_ASSERT(node_attr != NULL);
        i2cmux_chan_bus->i2c_mux_channel = (uint_t) strtoul (node_attr, NULL, 0);

        node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_INSTANCE);
        STD_ASSERT(node_attr != NULL);
        bus = (sdi_bus_hdl_t) &(i2cmux_chan_bus->i2c_bus);
        bus->bus_id = (uint_t) strtoul (node_attr, NULL, 0);

        bus->bus_type = SDI_I2C_BUS;
        bus->bus_init = sdi_i2cmux_channel_init;

        node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_BUS_NAME);
        STD_ASSERT(node_attr != NULL);
        safestrncpy(bus->bus_name, node_attr, SDI_MAX_NAME_LEN);

        i2cmux_chan_bus->i2c_mux = i2cmux;

        sdi_i2c_bus_register((sdi_i2c_bus_hdl_t) i2cmux_chan_bus);

        sdi_bus_enqueue_list(&i2cmux->channel_list,
                             (sdi_bus_hdl_t)i2cmux_chan_bus);

        sdi_bus_register_device_list(cur_node, (sdi_bus_hdl_t)i2cmux_chan_bus);

    }

    return STD_ERR_OK;
}


/**
 * sdi_i2cmux_pin_driver_init
 * Initialize i2c mux device
 * param[in] device_hdl - i2c mux device handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cmux_pin_driver_init(sdi_device_hdl_t device_hdl)
{
    t_std_error error = STD_ERR_OK;
    sdi_i2cmux_pin_hdl_t i2cmux = (sdi_i2cmux_pin_hdl_t) (device_hdl->private_data);
    sdi_i2c_bus_hdl_t i2c_bus = NULL;

    i2c_bus = sdi_get_i2c_bus_handle_by_name(i2cmux->i2c_bus_name);
    STD_ASSERT(i2c_bus != NULL);
    i2cmux->i2cbus_hdl = i2c_bus;

    i2cmux->pingroup_hdl = sdi_get_pin_group_bus_handle_by_name(i2cmux->pingroup_bus_name);
    STD_ASSERT(i2cmux->pingroup_hdl != NULL);

    error = std_mutex_lock_init_non_recursive(&(i2cmux->mux_lock));
    if (error != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d lock init failed with error %d\n",
                              __FUNCTION__, __LINE__, error);
        return error;
    }

    sdi_init_bus_for_each_bus_in_list(&i2cmux->channel_list,
                                      sdi_i2cmux_channel_init, NULL);

    return error;
}

