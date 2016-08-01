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
 * filename: sdi_cpld_pin.c
 */


/******************************************************************************
 * Read sdi_cpld.c for documentation on cpld pins
 * xml file format:
 *      <sdi_cpld_pin instance="2" addr="0x7" offset="0x1" bus_name="master_led"
 *            direction="out" level="1" polarity="normal">
 *      </sdi_cpld_pin>
 *        <!-- addr is cpld pin address within cpld --!>
 *        <!-- offset is cpld pin's offset within cpld addr register --!>
 *
 *****************************************************************************/

#include "sdi_cpld.h"
#include "sdi_device_common.h"
#include "sdi_bus_framework.h"
#include "sdi_bus_api.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_pin_group_bus_framework.h"
#include "std_assert.h"
#include "sdi_pin_bus_attr.h"
#include "sdi_bus_attr.h"
#include "sdi_common_attr.h"
#include "std_mutex_lock.h"
#include "std_utils.h"
#include "sdi_cpld_attr.h"
#include "std_bit_ops.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * Read the cpld pin level.
 * sequence of operation:
 * 1. Read value of cpld register using bus read operation.
 * 2. Mask the value with bit corresponding to cpld pin
 * param[in] pin_hdl cpld pin handle
 * param[out] value cpld pin level is read into value
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure,
 * SDI_DEVICE_ERRCODE(ENOTSUP) on un supported operation
 */
static t_std_error sdi_cpld_pin_read_level (sdi_pin_bus_hdl_t pin_hdl,
                                            sdi_pin_bus_level_t *value)
{
    sdi_cpld_pin_t *cpld_pin = (sdi_cpld_pin_t *) pin_hdl;
    sdi_device_hdl_t dev_hdl = cpld_pin->cpld_hdl;
    sdi_bus_hdl_t bus_hdl = (sdi_bus_hdl_t) dev_hdl->bus_hdl;
    uint8_t buffer = 0;
    t_std_error error = STD_ERR_OK;
    sdi_cpld_dev_hdl_t cpld_dev_hdl = (sdi_cpld_dev_hdl_t) dev_hdl->private_data;

    STD_ASSERT(cpld_dev_hdl->width == SDI_CPLD_DEFAULT_REGISTER_WIDTH);

    /* Read the cpld register on which this cpld pin is just a bit */
    error = sdi_bus_read_byte(bus_hdl, dev_hdl->addr,
                             cpld_pin->addr, &buffer);
    if (error != STD_ERR_OK) {
        return error;
    }

    /* Read the cpld pin's bit-offset */
    if (STD_BIT_TEST(buffer, (cpld_pin->offset))) {
        /* if pin is set, return pin level as
            - high when polarity is normal,
            - low when polarity is inverted
        */
        *value = ((pin_hdl->default_polarity == SDI_PIN_POLARITY_NORMAL) ?
            SDI_PIN_LEVEL_HIGH : SDI_PIN_LEVEL_LOW);
    } else {
        /* if pin is not set, return pin level as
            - low when polarity is normal,
            - high when polarity is inverted
        */
        *value = ((pin_hdl->default_polarity == SDI_PIN_POLARITY_NORMAL) ?
            SDI_PIN_LEVEL_LOW : SDI_PIN_LEVEL_HIGH);
    }

    return error;
}

/*
 * Update the cpld pin level with given value
 * sequence of operation:
 * 1. Read cpld register using bus read operation.
 * 2. Mask the register value with bit corresponding to cpld pin and set bit with
 * given input level
 * 3. Write new value onto cpld register
 * param[in] pin_hdl cpld pin handle
 * param[in] value cpld pin level to be written
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure,
 * SDI_DEVICE_ERRCODE(ENOTSUP) on un supported operation
 */
static t_std_error sdi_cpld_pin_write_level(sdi_pin_bus_hdl_t pin_hdl,
                                            sdi_pin_bus_level_t value)
{
    sdi_cpld_pin_t *cpld_pin = (sdi_cpld_pin_t *) pin_hdl;
    sdi_device_hdl_t dev_hdl = cpld_pin->cpld_hdl;
    sdi_bus_hdl_t bus_hdl = (sdi_bus_hdl_t) dev_hdl->bus_hdl;
    uint8_t buffer = 0;
    t_std_error error = STD_ERR_OK;
    sdi_cpld_dev_hdl_t cpld_dev_hdl = (sdi_cpld_dev_hdl_t) dev_hdl->private_data;

    STD_ASSERT(cpld_dev_hdl->width == SDI_CPLD_DEFAULT_REGISTER_WIDTH);

    if (pin_hdl->default_direction == SDI_PIN_BUS_INPUT) {
        /* if the cpld pin is configured as input, write operation is not
         * supported */
        return SDI_DEVICE_ERRCODE(ENOTSUP);
    }

    /* Read the cpld register (on which this cpld pin is just a bit)
     * value to buffer */
    error = sdi_bus_read_byte(bus_hdl, dev_hdl->addr,
                             cpld_pin->addr, &buffer);
    if (error != STD_ERR_OK) {
        return error;
    }

    if (pin_hdl->default_polarity == SDI_PIN_POLARITY_NORMAL) {
        /* When pin's polarity is normal,
        * - if pin level to be set is high, set the cpld pin offset in cpld
        * register (buffer)
        * - if the pin level to be set is low, clear the cpld pin offset in cpld
        * register (buffer)
        */
        (value == SDI_PIN_LEVEL_HIGH) ?
        STD_BIT_SET(buffer, (cpld_pin->offset)) :
            STD_BIT_CLEAR(buffer, (cpld_pin->offset));
    } else {
        /* When pin's polarity is inverted,
        * - if pin level to be set is high, clear the cpld pin offset in cpld
        * register (buffer)
        * - if the pin level to be set is low, set the cpld pin offset in cpld
        * register (buffer)
        */
        (value == SDI_PIN_LEVEL_HIGH) ?
        STD_BIT_CLEAR(buffer, (cpld_pin->offset)) :
            STD_BIT_SET(buffer, (cpld_pin->offset));
    }

    /* Write the modified value of cpld regiter to effect the pin level
     * change */
    error = sdi_bus_write_byte(bus_hdl, dev_hdl->addr,
                              cpld_pin->addr, buffer);
    return error;
}

/*
 * param[in] pin_hdl cpld pin handle
 * param[in] direction pin direction to be configured
 * return SDI_DEVICE_ERRCODE(ENOTSUP)
 */
static inline t_std_error sdi_cpld_pin_set_direction(sdi_pin_bus_hdl_t pin_hdl,
                                               sdi_pin_bus_direction_t direction)
{
    return SDI_DEVICE_ERRCODE(ENOTSUP);
}

/*
 * Get cpld pin configured direction
 * param[in] pin_hdl cpld pin handle
 * param[out] direction configured pin direction
 * return STD_ERR_OK on success.
 */
static inline t_std_error sdi_cpld_pin_get_direction (sdi_pin_bus_hdl_t pin_hdl,
                                               sdi_pin_bus_direction_t *direction)
{
    *direction = pin_hdl->default_direction;
    return STD_ERR_OK;
}

/*
 * param[in] pin_hdl cpld pin handle
 * param[in] polarity pin polarity to be configured
 * return SDI_DEVICE_ERRCODE(ENOTSUP)
 */
static inline t_std_error sdi_cpld_pin_set_polarity (sdi_pin_bus_hdl_t pin_hdl,
                                              sdi_pin_bus_polarity_t polarity)
{
    return SDI_DEVICE_ERRCODE(ENOTSUP);
}

/*
 * Get cpld pin configured polarity
 * param[in] pin_hdl cpld pin handle
 * param[out] polarity configured pin polarity
 * return STD_ERR_OK on success.
 */
static inline t_std_error sdi_cpld_pin_get_polarity (sdi_pin_bus_hdl_t pin_hdl,
                                              sdi_pin_bus_polarity_t *polarity)
{
    *polarity = pin_hdl->default_polarity;
    return STD_ERR_OK;
}

/*
 * CPLD pin bus operations
 */
static sdi_pin_bus_ops_t sdi_cpld_pin_ops = {
    .sdi_pin_bus_read_level = sdi_cpld_pin_read_level,
    .sdi_pin_bus_write_level = sdi_cpld_pin_write_level,
    .sdi_pin_bus_set_direction = sdi_cpld_pin_set_direction,
    .sdi_pin_bus_get_direction = sdi_cpld_pin_get_direction,
    .sdi_pin_bus_set_polarity = sdi_cpld_pin_set_polarity,
    .sdi_pin_bus_get_polarity = sdi_cpld_pin_get_polarity,
};

/*
 * Initialize cpld ouput pin to configured default level
 * param[in] bus_hdl - cpld pin bus handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on error,
 * SDI_DEVICE_ERRCODE(ENOTSUP) when operation not supported
 */
static t_std_error sdi_cpld_pin_driver_init(sdi_bus_hdl_t bus_hdl)
{
    t_std_error error = STD_ERR_OK;
    sdi_pin_bus_hdl_t pin_hdl = (sdi_pin_bus_hdl_t) bus_hdl;

    if (pin_hdl->default_direction == SDI_PIN_BUS_OUTPUT) {
        /* Write the default pin level for an output pin */
        error = sdi_cpld_pin_write_level(pin_hdl, pin_hdl->default_level);
        if (error != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d write pin %s failed with error %d\n",
                                  __FUNCTION__, __LINE__, pin_hdl->bus.bus_name, error);
        }
    }
    return error;
}

/*
 * Create a cpld pin and register the pin with pin bus framework
 * param[in] node - cpld pin config node
 * param[out] bus_hdl - filled with cpld pin bus handle on successful registration
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_cpld_pin_driver_register(std_config_node_t node,
                                                sdi_bus_hdl_t *bus_hdl)
{
    char *node_attr = NULL;
    sdi_pin_bus_hdl_t pin_bus = NULL;
    t_std_error error = STD_ERR_OK;

    sdi_cpld_pin_t *cpld_pin = (sdi_cpld_pin_t *)
                               calloc(sizeof(sdi_cpld_pin_t), 1);
    STD_ASSERT(cpld_pin != NULL);

    pin_bus = &(cpld_pin->bus);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    pin_bus->bus.bus_id = (uint_t) strtoul (node_attr, NULL, 0);

    pin_bus->bus.bus_type = SDI_PIN_BUS;
    pin_bus->bus.bus_init = sdi_cpld_pin_driver_init;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    cpld_pin->addr = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_OFFSET);
    STD_ASSERT(node_attr != NULL);
    cpld_pin->offset = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_DIRECTION);
    if (node_attr != NULL) {
        if ((strncmp(node_attr, SDI_DEV_ATTR_INPUT_PIN,
                SDI_DEV_ATTR_INPUT_PIN_LEN)) == 0) {
            pin_bus->default_direction = SDI_PIN_BUS_INPUT;
        } else if  ((strncmp(node_attr, SDI_DEV_ATTR_OUTPUT_PIN,
                    SDI_DEV_ATTR_OUTPUT_PIN_LEN)) == 0) {
             pin_bus->default_direction = SDI_PIN_BUS_OUTPUT;
        } else {
            SDI_DEVICE_ERRMSG_LOG("%s:%d invalid direction %s\n",
                                  __FUNCTION__, __LINE__, pin_bus->bus.bus_name);
            STD_ASSERT(false);
        }
    } else {
        /* If pin direction is not configured, default it to input pin */
        pin_bus->default_direction = SDI_PIN_BUS_INPUT;
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_POLARITY);
    if (node_attr != NULL) {
        if ((strncmp(node_attr, SDI_DEV_ATTR_POLARITY_NORMAL,
                SDI_DEV_ATTR_POLARITY_NORMAL_LEN)) == 0) {
            pin_bus->default_polarity = SDI_PIN_POLARITY_NORMAL;
        } else if ((strncmp(node_attr, SDI_DEV_ATTR_POLARITY_INVERTED,
                SDI_DEV_ATTR_POLARITY_INVERTED_LEN)) == 0) {
            pin_bus->default_polarity = SDI_PIN_POLARITY_INVERTED;
        } else {
            SDI_DEVICE_ERRMSG_LOG("%s:%d invalid polarity %s\n",
                                  __FUNCTION__, __LINE__, pin_bus->bus.bus_name);
            STD_ASSERT(false);
        }
    } else {
        /* If polarity is not configured, default it to normal */
        pin_bus->default_polarity = SDI_PIN_POLARITY_NORMAL;
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_LEVEL);
    if (node_attr != NULL) {
        pin_bus->default_level = (int) strtoul (node_attr, NULL, 0);
        STD_ASSERT(((pin_bus->default_level == SDI_PIN_LEVEL_LOW) ||
            ((pin_bus->default_level == SDI_PIN_LEVEL_HIGH))));
    } else {
        pin_bus->default_level = SDI_PIN_LEVEL_LOW;
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_BUS_NAME);
    STD_ASSERT(node_attr != NULL);
    safestrncpy(pin_bus->bus.bus_name, node_attr, SDI_MAX_NAME_LEN);

    std_mutex_lock_init_non_recursive(&(pin_bus->lock));
    pin_bus->ops = &sdi_cpld_pin_ops;

    error = sdi_pin_bus_register((sdi_pin_bus_hdl_t )cpld_pin);
    if (error != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d registration failed %s with error %d\n",
                              __FUNCTION__, __LINE__, pin_bus->bus.bus_name, error);
        std_mutex_destroy(&(pin_bus->lock));
        free(cpld_pin);
        return error;
    }

    *bus_hdl = (sdi_bus_hdl_t ) cpld_pin;

    sdi_bus_register_device_list(node, (sdi_bus_hdl_t) cpld_pin);

    return error;
}

/**
 * SDI cpld pin bus driver object to hold cpld pin bus registration and
 * initialization function
 * Note:
 * Every bus driver must export function with name
 * sdi_<bus_driver_name>_entry_callbacks
 * so that the driver framework is able to look up and invoke it to get the
 * callbacks
 */
const sdi_bus_driver_t * sdi_cpld_pin_entry_callbacks(void)
{
     /*Export Bus Driver table*/
     static const sdi_bus_driver_t sdi_cpld_pin_entry = {
        .bus_register = sdi_cpld_pin_driver_register,
        .bus_init = sdi_cpld_pin_driver_init
     };
     return &sdi_cpld_pin_entry;
}
