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
 * filename: sdi_gpio.c
 */


/******************************************************************************
 * Creates and Registers SDI GPIO Pin/PinGroup Bus.
 * Provides interfaces to access direction, level and polarity configuration of
 * GPIO Pin/PinGroup Bus
 *****************************************************************************/

#include "sdi_device_common.h"
#include "sdi_sysfs_gpio_helpers.h"
#include "sdi_driver_internal.h"
#include "sdi_gpio.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_pin_group_bus_framework.h"
#include "std_assert.h"
#include "sdi_pin_bus_attr.h"
#include "sdi_bus_attr.h"
#include "sdi_common_attr.h"
#include "std_utils.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/**
 * For every supported gpio chip, Linux exports the gpio pins via sysfs interface
 * /sys/class/gpio/ to facilitate user-space applications accessing gpio pins.
 * A specific GPIO pin can be used by writing pin number to the EXPORT File
 * under gpio sysfs, and a pin can be unexported when not being used by writing
 * gpio pin number to the UNEXPORT File.
 * A gpio pin direction could be set to input or output by writing in or out
 * to direction file.
 * Similarly level of a gpio pin could be driven by writing 0 or 1 to value file.
 * A gpio pin's logic level could be inverted by toggling value of invert_low
 * file.
 * For more details on accessing GPIO from user-space for Kernel driven gpio bus
 * refer to https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
 */

/**
 * GPIO Pin Configuration Format:
 *   <sdi_gpio instance="0" pin="0" direction="out" level="0"
 *            bus_name="gpio0" polarity="normal">
 *   </sdi_gpio>
 *
 * GPIO Pin Group Configuration Format:
 *   <sdi_gpio_group instance="0" pingroup="1,2" direction="out" level="0"
            bus_name="gpio_group0" polarity="normal">
 *   </sdi_gpio_group>
 *
 * - instance : identifier for a pin
 * - pin : gpio pin number as exported by kernel gpio driver
 * - pingroup : gpio pin numbers used to create a pin group (comma-sperated)
 * - direction : default direction to be configured during initialization (out/in)
 * When not specified, default direction is input
 * - level : default level to be configured during init (0/1)
 * When not specified, default level is low(0) for an output pin/pingroup
 * - bus_name : gpio pin name
 * - polarity : default logic level to be configured during init
 * (normal/inverted). When not specified, default polarity is 'normal'
 *
 */

/* Minimum number of pins required to form a pin group */
#define MIN_NUMBER_OF_PINS_IN_GROUP        2


/* Pin/PinGroup Bus Lock Init failure */
#define BUS_LOCK_INIT_FAILURE             -1

/* Pin/PinGroup Bus Registration Failure */
#define BUS_REGISTRATION_FAILURE          -2

/**
 * Read gpio pin level thats being currently driven by reading the value from
 * /sys/class/gpio/gpio<num>/value
 * param[in] bus - gpio pin bus
 * param[out] value - gpio level read from gpio pin
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_level_read (sdi_pin_bus_hdl_t bus,
                                        sdi_pin_bus_level_t *value)
{
    sdi_gpio_pin_t *gpio_bus = (sdi_gpio_pin_t *) bus;
    /* bus is already validated by its caller (sdi_pin_read_level) */

    return sdi_sysfs_gpio_value_read(
            gpio_bus->gpio_file_tbl->level_fd, (bool *)value);
}

/**
 * Change gpio pin level by writing level to
 * /sys/class/gpio/gpio<num>/value
 * param[in] bus - gpio pin bus
 * param[in] value - can be either 0 or 1
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_level_write (sdi_pin_bus_hdl_t bus,
                                         sdi_pin_bus_level_t value)
{
    sdi_gpio_pin_t *gpio_bus = (sdi_gpio_pin_t *) bus;
    /* bus is already validated by its caller (sdi_pin_write_level) */

    return sdi_sysfs_gpio_value_write(gpio_bus->gpio_file_tbl->level_fd, value);
}

/**
 * Read direction of gpio from sysfs gpio absolute file path
 * param[in] bus - gpio pin bus
 * param[out] direction - gpio direction currently configured on gpio pin.
 * SDI_PIN_BUS_INPUT on input pin, SDI_PIN_BUS_OUTPUT on output pin
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_direction_get (sdi_pin_bus_hdl_t bus,
                                           sdi_pin_bus_direction_t *direction)
{
    sdi_gpio_pin_t *gpio_bus = (sdi_gpio_pin_t *) bus;
    /* bus is already validated by its caller (sdi_pin_get_direction) */

    return sdi_sysfs_gpio_direction_read(
        gpio_bus->gpio_file_tbl->direction_fd, direction);
}

/**
 * Write direction of gpio to sysfs gpio absolute file path
 * param[in] bus - gpio pin bus
 * param[in] direction - gpio directio to be configured.
 * SDI_PIN_BUS_INPUT on input pin, SDI_PIN_BUS_OUTPUT on output pin
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_direction_set (sdi_pin_bus_hdl_t bus,
                                           sdi_pin_bus_direction_t direction)
{
    sdi_gpio_pin_t *gpio_bus = (sdi_gpio_pin_t *) bus;
    /* bus is already validated by its caller (sdi_pin_set_direction) */

    return sdi_sysfs_gpio_direction_write(
        gpio_bus->gpio_file_tbl->direction_fd, direction);
}

/**
 * Read gpio pin's currently configured polarity
 * /sys/class/gpio/gpio<num>/active_low
 * param[in] bus - gpio pin bus
 * param[out] polarity - gpio polarity currently configured on gpio pin, can be
 * either SDI_PIN_POLARITY_NORMAL or SDI_PIN_POLARITY_INVERTED
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_polarity_get(sdi_pin_bus_hdl_t bus,
                                         sdi_pin_bus_polarity_t *polarity)
{
    sdi_gpio_pin_t *gpio_bus = (sdi_gpio_pin_t *) bus;
    /* bus is already validated by its caller (sdi_pin_get_polarity) */

    return sdi_sysfs_gpio_value_read(
            gpio_bus->gpio_file_tbl->polarity_fd, (bool *)polarity);
}

/**
 * Modify gpio pin's polarity active low or high by writing to
 * /sys/class/gpio/gpio<num>/active_low
 * param[in] bus - gpio pin bus
 * param[in] polarity - gpio polarity to be configured, can be either
 * SDI_PIN_POLARITY_NORMAL or SDI_PIN_POLARITY_INVERTED
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_polarity_set(sdi_pin_bus_hdl_t bus,
                                         sdi_pin_bus_polarity_t polarity)
{
    sdi_gpio_pin_t *gpio_bus = (sdi_gpio_pin_t *) bus;
    /* bus is already validated by its caller (sdi_pin_set_polarity) */

    return sdi_sysfs_gpio_value_write(
            gpio_bus->gpio_file_tbl->polarity_fd, polarity);
}


/**
 * gpio pin operations to read/write gpio pin level, direction and polarity
 * This ops is same for every pin exported by this driver.
 */
static sdi_pin_bus_ops_t sdi_gpio_ops = {
    .sdi_pin_bus_read_level = sdi_gpio_level_read,
    .sdi_pin_bus_write_level = sdi_gpio_level_write,
    .sdi_pin_bus_set_direction = sdi_gpio_direction_set,
    .sdi_pin_bus_get_direction = sdi_gpio_direction_get,
    .sdi_pin_bus_set_polarity = sdi_gpio_polarity_set,
    .sdi_pin_bus_get_polarity = sdi_gpio_polarity_get,
};

/**
 * Configure gpio pin default direction, level and polarity as specified in
 * default configuration
 * param[in] gpio_pin - gpio pin object with default
 * configuration information
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_default_configuration_set(
    sdi_gpio_pin_t *gpio_pin)
{
    t_std_error err = STD_ERR_OK;
    uint_t gpio_no = gpio_pin->gpio_num;
    sdi_pin_bus_hdl_t pin_bus = &(gpio_pin->bus);

    err = sdi_gpio_direction_set(pin_bus, pin_bus->default_direction);
    if (err != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u direction %d can't be set, err: %d\n",
                __FUNCTION__, __LINE__, pin_bus->bus.bus_name, gpio_no,
                pin_bus->default_direction, err);
        return err;
    }
    if (pin_bus->default_direction == SDI_PIN_BUS_OUTPUT) {
        err = sdi_gpio_level_write(pin_bus, pin_bus->default_level);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u level %d can't be set, err: %d\n",
                __FUNCTION__, __LINE__, pin_bus->bus.bus_name, gpio_no,
                pin_bus->default_level, err);
            return err;
        }
    }

    err = sdi_gpio_polarity_set(pin_bus, pin_bus->default_polarity);
    if (err != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u polarity %d can't be set, err: %d\n",
            __FUNCTION__, __LINE__, pin_bus->bus.bus_name, gpio_no,
            pin_bus->default_polarity, err);
        return err;
    }

    return err;
}

/**
 * Export gpio pin,
 * Initialize gpio file table with file handle for gpio sysfs
 * file: value, direction and active_lowi for the gpio pin,
 * Initialize the gpio pin with default configuration
 * param[in] bus_hdl - gpio pin handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on error.
 */
static t_std_error sdi_gpio_driver_init(sdi_bus_hdl_t bus_hdl)
{
    sdi_gpio_pin_t *gpio_pin = (sdi_gpio_pin_t *) bus_hdl;
    t_std_error err = STD_ERR_OK;
    uint_t gpio_no = 0;

    STD_ASSERT(gpio_pin != NULL);

    gpio_no = gpio_pin->gpio_num;

    err = sdi_export_gpio(gpio_no);
    if (err != STD_ERR_OK) {
        return err;
    }

    err = sdi_gpio_file_table_init(gpio_no, gpio_pin->gpio_file_tbl);
    if (err != STD_ERR_OK) {
        return err;
    }

    err = sdi_gpio_default_configuration_set(gpio_pin);
    if (err != STD_ERR_OK) {
        return err;
    }

    sdi_bus_init_device_list(bus_hdl);

    return err;
}


/**
 * SDI GPIO Kernel Interface driver registration
 * Creates gpio pin object for every defined gpio pin and registers with
 * pin bus framework.
 * param[in] node - gpio bus node handle obtained by parsing configuration
 * param[out] bus - to be filled with gpio bus handle
 * return returns STD_ERR_OK on success,
 */
static t_std_error sdi_gpio_driver_register(std_config_node_t node,
                                                  sdi_bus_hdl_t *bus)
{
    sdi_gpio_pin_t *gpio_pin = NULL;
    sdi_pin_bus_hdl_t pin_bus = NULL;
    char *node_attr = NULL;
    t_std_error error = STD_ERR_OK;

    STD_ASSERT(bus != NULL);

    gpio_pin = (sdi_gpio_pin_t *)
        calloc(sizeof(sdi_gpio_pin_t), 1);

    STD_ASSERT(gpio_pin != NULL);

    pin_bus = &(gpio_pin->bus);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    pin_bus->bus.bus_id = (uint_t) strtoul (node_attr, NULL, 0);

    pin_bus->bus.bus_type = SDI_PIN_BUS;
    pin_bus->bus.bus_init = sdi_gpio_driver_init;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_NUMBER);
    STD_ASSERT(node_attr != NULL);
    gpio_pin->gpio_num = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_DIRECTION);
    if (node_attr != NULL) {
        if ((strncmp(node_attr, SDI_DEV_ATTR_INPUT_PIN,
                SDI_DEV_ATTR_INPUT_PIN_LEN)) == 0) {
            pin_bus->default_direction = SDI_PIN_BUS_INPUT;
        } else if  ((strncmp(node_attr, SDI_DEV_ATTR_OUTPUT_PIN,
                    SDI_DEV_ATTR_OUTPUT_PIN_LEN)) == 0) {
             pin_bus->default_direction = SDI_PIN_BUS_OUTPUT;
        } else {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u invalid direction\n",
                __FUNCTION__, __LINE__, pin_bus->bus.bus_name,
                gpio_pin->gpio_num);
            STD_ASSERT(false);
        }
    } else {
        /* If gpio pin direction is not configured, default it to input pin */
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
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u invalid polarity\n",
                __FUNCTION__, __LINE__, pin_bus->bus.bus_name,
                gpio_pin->gpio_num);
            STD_ASSERT(false);
        }
    } else {
        /* If gpio polarity is not configured, default it to normal */
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
    strncpy(pin_bus->bus.bus_name, node_attr, SDI_MAX_NAME_LEN);

    do {
        if (std_mutex_lock_init_non_recursive(&(pin_bus->lock)) != STD_ERR_OK) {
            error = BUS_LOCK_INIT_FAILURE;
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s gpio pin %u lock init failed\n",
                __FUNCTION__, __LINE__, pin_bus->bus.bus_name,
                gpio_pin->gpio_num);
            break;
        }

        pin_bus->ops = &sdi_gpio_ops;

        gpio_pin->gpio_file_tbl = (sdi_gpio_file_fd_tbl_t *)
                                   calloc(sizeof(sdi_gpio_file_fd_tbl_t), 1);

        STD_ASSERT(gpio_pin->gpio_file_tbl != NULL);

        if (sdi_pin_bus_register((sdi_pin_bus_hdl_t )gpio_pin) != STD_ERR_OK) {
            error = BUS_REGISTRATION_FAILURE;
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s gpio pin %u registration failed\n",
                __FUNCTION__, __LINE__, pin_bus->bus.bus_name,
                gpio_pin->gpio_num);
            break;
        }

    } while (0);

    if (error == STD_ERR_OK) {
        *bus = (sdi_bus_hdl_t ) gpio_pin;
        sdi_bus_register_device_list(node, (sdi_bus_hdl_t) gpio_pin);
    }
    else {
        if (error == BUS_REGISTRATION_FAILURE) {
            std_mutex_destroy(&(pin_bus->lock));
            free(gpio_pin->gpio_file_tbl);
        }
        free(gpio_pin);
    }

    return error;
}

/**
 * SDI gpio Driver Object to hold gpio registration and
 * initialization function
 * Note:
 * Every bus driver must export function with name
 * sdi_<bus_driver_name>_entry_callbacks
 * so that the driver framework is able to look up and invoke it to get the
 * callbacks
 */
const sdi_bus_driver_t * sdi_gpio_entry_callbacks(void)
{
     /*Export Bus Driver table*/
     static const sdi_bus_driver_t sdi_gpio_entry = {
        .bus_register = sdi_gpio_driver_register,
        .bus_init = sdi_gpio_driver_init
     };
     return &sdi_gpio_entry;
}

/**
 * Read value of the GPIO Pin Group
 * param[in] bus_hdl - gpio pin bus handle
 * param[out] value - GPIO pin group level currently configured in pin group
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_level_read(sdi_pin_group_bus_hdl_t bus_hdl,
                                                   uint_t *value)
{
    sdi_gpio_group_t *gpio_group_hdl = (sdi_gpio_group_t *) bus_hdl;
    /* bus_hdl is already validated by its caller (sdi_pin_group_read_level) */

    uint_t *gpio_group = gpio_group_hdl->gpio_group;
    uint_t gpio_index = 0;
    uint_t gpio_cnt = gpio_group_hdl->gpio_count;
    sdi_pin_bus_level_t gpio_level = SDI_PIN_LEVEL_LOW;
    t_std_error err = STD_ERR_OK;
    sdi_gpio_file_fd_tbl_t *gpio_file_tbl = NULL;

    *value = 0;

    for (gpio_index = 0; gpio_index < gpio_cnt; gpio_index++) {
        gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
        err = sdi_sysfs_gpio_value_read(gpio_file_tbl->level_fd,
                                        (bool *)&gpio_level);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u level can't be read, err: %d\n",
                __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
                gpio_group[gpio_index], err);
            return err;
        }
        if (gpio_level == SDI_PIN_LEVEL_HIGH) {
            *value |= (1 << (gpio_cnt - gpio_index - 1));
        }
    }

    return err;
}
/**
 * Write value of the Pin Group
 * param[in] bus_hdl - gpio pin bus handle
 * param[in] value - level to be configured in given gpio pin group
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_level_write(sdi_pin_group_bus_hdl_t bus_hdl,
                                                    uint_t value)
{
    sdi_gpio_group_t *gpio_group_hdl = (sdi_gpio_group_t *) bus_hdl;
    /* bus_hdl is already validated by its caller (sdi_pin_group_write_level) */

    uint_t *gpio_group = gpio_group_hdl->gpio_group;
    uint_t gpio_index = 0;
    uint_t gpio_cnt = gpio_group_hdl->gpio_count;
    sdi_pin_bus_level_t gpio_level = SDI_PIN_LEVEL_LOW;
    t_std_error err = STD_ERR_OK;
    sdi_gpio_file_fd_tbl_t *gpio_file_tbl = NULL;

    STD_ASSERT((value < (1 << gpio_cnt)));

    for (gpio_index = 0; gpio_index < gpio_cnt; gpio_index++) {
        if (value & ((gpio_cnt - gpio_index ))) {
            gpio_level = SDI_PIN_LEVEL_HIGH;
        } else {
            gpio_level = SDI_PIN_LEVEL_LOW;
        }
        gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
        err = sdi_sysfs_gpio_value_write(gpio_file_tbl->level_fd,
                                        gpio_level);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u level can't be set to %d, err: %d\n",
                __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
                gpio_group[gpio_index], gpio_level, err);
            return err;
        }
    }
    return err;
}

/**
 * Configure pin group direction to Input/Output
 * param[in] bus_hdl - gpio pin bus handle
 * param[in] direction - direction to be configured to the given GPIO pin group
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_direction_set(sdi_pin_group_bus_hdl_t bus_hdl,
                                                      sdi_pin_bus_direction_t direction)
{
    sdi_gpio_group_t *gpio_group_hdl = (sdi_gpio_group_t *) bus_hdl;
    /* bus_hdl is already validated by its caller (sdi_pin_group_set_direction) */

    uint_t *gpio_group = gpio_group_hdl->gpio_group;
    uint_t gpio_index = 0;
    uint_t gpio_cnt = gpio_group_hdl->gpio_count;
    t_std_error err = STD_ERR_OK;
    sdi_gpio_file_fd_tbl_t *gpio_file_tbl = NULL;

    for (gpio_index = 0; gpio_index < gpio_cnt; gpio_index++) {
        gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
        err = sdi_sysfs_gpio_direction_write(gpio_file_tbl->direction_fd,
                                            direction);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u direction %d can't be set to %d,"
                " err: %d\n", __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
                gpio_group[gpio_index], direction, err);
            return err;
        }
    }

    return err;
}

/**
 * Get configured pin group direction
 * param[in] bus_hdl - gpio pin bus handle
 * param[out] direction - GPIO pin group direction currently configured in pin group
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_direction_get(sdi_pin_group_bus_hdl_t bus_hdl,
                                                      sdi_pin_bus_direction_t *direction)
{
    sdi_gpio_group_t *gpio_group_hdl = (sdi_gpio_group_t *) bus_hdl;
    /* bus_hdl is already validated by its caller (sdi_pin_group_get_direction) */

    uint_t *gpio_group = gpio_group_hdl->gpio_group;
    uint_t gpio_index = 0;
    uint_t gpio_cnt = gpio_group_hdl->gpio_count;
    t_std_error err = STD_ERR_OK;
    sdi_pin_bus_direction_t first_pin_dir = SDI_PIN_BUS_INPUT;
    sdi_gpio_file_fd_tbl_t *gpio_file_tbl = NULL;

    gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
    err = sdi_sysfs_gpio_direction_read(gpio_file_tbl->direction_fd,
                                        &first_pin_dir);
    if (err != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u direction can't be read, err: %d\n",
            __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
            gpio_group[gpio_index], err);
        return err;
    }

    /* Ensure direction of all gpio pins in pin group match the diretion of
     * first gpio pin */
    for (gpio_index = 1; gpio_index < gpio_cnt; gpio_index++) {
        gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
        err = sdi_sysfs_gpio_direction_read(gpio_file_tbl->direction_fd,
                                        direction);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u direction can't be read, err: %d\n",
                __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
                gpio_group[gpio_index], err);
            return err;
        }
        if (*direction != first_pin_dir) {
            return SDI_DEVICE_ERRNO;
        }
    }

    return err;
}

/**
 * Set the polarity of the pin group
 * param[in] bus_hdl - gpio pin bus handle
 * param[in] polarity - polarity to be configured to the given GPIO pin group
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_polarity_set(sdi_pin_group_bus_hdl_t bus_hdl,
                                                     sdi_pin_bus_polarity_t polarity)
{
    sdi_gpio_group_t *gpio_group_hdl = (sdi_gpio_group_t *) bus_hdl;
    /* bus_hdl is already validated by its caller (sdi_pin_group_set_polarity) */

    uint_t *gpio_group = gpio_group_hdl->gpio_group;
    uint_t gpio_index = 0;
    uint_t gpio_cnt = gpio_group_hdl->gpio_count;
    t_std_error err = STD_ERR_OK;
    sdi_gpio_file_fd_tbl_t *gpio_file_tbl = NULL;

    for (gpio_index = 0; gpio_index < gpio_cnt; gpio_index++) {
        gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
        err = sdi_sysfs_gpio_value_write(gpio_file_tbl->polarity_fd,
                                            polarity);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u polarity %d can't be set to %d,"
                " err: %d\n", __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
                gpio_group[gpio_index], polarity, err);
            return err;
        }
    }

    return err;
}

/**
 * Get current configured pin group polarity
 * param[in] bus_hdl - gpio pin bus handle
 * param[in] polarity - current polarity configuration of the given GPIO pin group
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_polarity_get(sdi_pin_group_bus_hdl_t bus_hdl,
                                                     sdi_pin_bus_polarity_t *polarity)
{
    sdi_gpio_group_t *gpio_group_hdl = (sdi_gpio_group_t *) bus_hdl;
    /* bus_hdl is already validated by its caller (sdi_pin_group_get_polarity) */

    uint_t *gpio_group = gpio_group_hdl->gpio_group;
    uint_t gpio_index = 0;
    uint_t gpio_cnt = gpio_group_hdl->gpio_count;
    t_std_error err = STD_ERR_OK;
    sdi_pin_bus_polarity_t first_pin_polarity = SDI_PIN_POLARITY_NORMAL;
    sdi_gpio_file_fd_tbl_t *gpio_file_tbl = NULL;

    gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
    err = sdi_sysfs_gpio_value_read(gpio_file_tbl->polarity_fd,
                                       (bool *) &first_pin_polarity);
    if (err != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u polarity can't be read, err: %d\n",
            __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
            gpio_group[gpio_index], err);
        return err;
    }

    /* Ensure polarity of all gpio pins in pin group match the polarity of
     * first gpio pin */
    for (gpio_index = 1; gpio_index < gpio_cnt; gpio_index++) {
        gpio_file_tbl = gpio_group_hdl->gpio_file_tbl[gpio_index];
        err = sdi_sysfs_gpio_value_read(gpio_file_tbl->polarity_fd,
                                           (bool *)polarity);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO %u polarity can't be read, err: %d\n",
                __FUNCTION__, __LINE__, bus_hdl->bus.bus_name,
                gpio_group[gpio_index], err);
            return err;
        }
        if (*polarity != first_pin_polarity) {
            return SDI_DEVICE_ERRNO;
        }
    }

    return err;
}

/**
 * GPIO Pin Group operations to get/set gpio pin group level, direction and
 * polarity */
static sdi_pin_group_bus_ops_t sdi_gpio_group_ops = {
    .sdi_pin_group_bus_read_level = sdi_gpio_group_level_read,
    .sdi_pin_group_bus_write_level = sdi_gpio_group_level_write,
    .sdi_pin_group_bus_set_direction = sdi_gpio_group_direction_set,
    .sdi_pin_group_bus_get_direction = sdi_gpio_group_direction_get,
    .sdi_pin_group_bus_set_polarity = sdi_gpio_group_polarity_set,
    .sdi_pin_group_bus_get_polarity = sdi_gpio_group_polarity_get,
};

/**
 * Configure gpio pin group with default level, direction and polarity
 * param[in] gpio_group - gpio pin group object pointer
 * returns STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error set_gpio_group_default_configuration(
    sdi_gpio_group_t *gpio_group)
{
    t_std_error err = STD_ERR_OK;
    sdi_pin_group_bus_hdl_t pin_group_bus = &(gpio_group->bus);

    err = sdi_gpio_group_direction_set(pin_group_bus,
            pin_group_bus->default_direction);
    if (err != STD_ERR_OK) {
        return err;
    }

    if (pin_group_bus->default_direction == SDI_PIN_BUS_OUTPUT) {
        err = sdi_gpio_group_level_write(pin_group_bus,
                pin_group_bus->default_level);
        if (err != STD_ERR_OK) {
            return err;
        }
    }

    err = sdi_gpio_group_polarity_set(pin_group_bus,
        pin_group_bus->default_polarity);
    if (err != STD_ERR_OK) {
        return err;
    }
    return err;
}

/**
 * Create gpio pin group, parse pins from pin group and populate gpio pin group
 * object
 * param[in] gpio_group - pointer to gpio pin group object
 * param[in] pin_group_str - pin group string with gpio pins seperated by comma
 * for ex: 1,2
 * @todo take range of gpio pins ex: 1-4
 * return none
 */
static void create_gpio_group(sdi_gpio_group_t *gpio_group,
                             const char *pin_group_str)
{
    char delimiter = ',';
    size_t count = 0;
    const char *token = NULL;
    std_parsed_string_t handle;
    if (std_parse_string(&handle, pin_group_str, &delimiter)) {
        count = std_parse_string_num_tokens(handle);
        STD_ASSERT(count >= MIN_NUMBER_OF_PINS_IN_GROUP);

        gpio_group->gpio_count = (uint_t)count;

        gpio_group->gpio_group = (uint_t *)
            calloc(sizeof(uint_t), gpio_group->gpio_count);
        STD_ASSERT(gpio_group != NULL);

        gpio_group->gpio_file_tbl = (sdi_gpio_file_fd_tbl_t **)
            calloc(sizeof(sdi_gpio_file_fd_tbl_t *),
            gpio_group->gpio_count);

        STD_ASSERT(gpio_group->gpio_file_tbl != NULL);

        count = 0;
        while((token = std_parse_string_next(handle,&count))) {
            gpio_group->gpio_group[count-1] =
                (uint_t) strtoul(token, NULL, 0);
            gpio_group->gpio_file_tbl[count-1] =
                (sdi_gpio_file_fd_tbl_t *)
                calloc(sizeof(sdi_gpio_file_fd_tbl_t), 1);
            STD_ASSERT(gpio_group->gpio_file_tbl[count-1] != NULL);

        }
        std_parse_string_free(handle);
    }
}

/**
 * SDI GPIO Group Initiaizliation
 * - export every gpio pin in pin group
 * - initialize gpio file table for every gpio pin in pin group
 * - configure default values for every pin in pin group
 * param[in] bus_hdl - gpio pin group handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_driver_init(sdi_bus_hdl_t bus_hdl)
{
    sdi_gpio_group_t *gpio_group = (sdi_gpio_group_t *) bus_hdl;
    sdi_pin_group_bus_hdl_t pin_group_bus = NULL;
    t_std_error err = STD_ERR_OK;
    uint_t gpio_no = 0;
    uint_t gpio_index = 0;

    STD_ASSERT(gpio_group != NULL);

    pin_group_bus = &(gpio_group->bus);

    for (gpio_index = 0; gpio_index < gpio_group->gpio_count;
        gpio_index++) {
        gpio_no = gpio_group->gpio_group[gpio_index];
        err = sdi_export_gpio(gpio_no);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s gpio pin %u with id %d exported\n",
                __FUNCTION__, __LINE__, pin_group_bus->bus.bus_name,
                gpio_no, pin_group_bus->bus.bus_id);
        }

        err = sdi_gpio_file_table_init(gpio_no,
                    gpio_group->gpio_file_tbl[gpio_index]);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s gpio pin %u id %d file table init failed\n",
                __FUNCTION__, __LINE__, pin_group_bus->bus.bus_name,
                gpio_no, pin_group_bus->bus.bus_id);
            return err;
        }

    }

    err = set_gpio_group_default_configuration(gpio_group);
    if (err != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s gpio pin %u id %d setting default config failed\n",
            __FUNCTION__, __LINE__, pin_group_bus->bus.bus_name,
            gpio_no, pin_group_bus->bus.bus_id);
    }

    return err;
}

/**
 * SDI GPIO Group Kernel Interface driver registration
 * Creates gpio pin group object for every defined gpio pin group
 * and registers with pin group bus framework.
 * param[in] node - gpio pin group bus node handle obtained by parsing configuration
 * param[out] bus - to be filled with gpio pin group bus handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_gpio_group_driver_register(std_config_node_t node,
                                                        sdi_bus_hdl_t *bus)
{
    sdi_gpio_group_t *gpio_group = NULL;
    char *node_attr = NULL;
    sdi_pin_group_bus_hdl_t pin_group_bus = NULL;
    t_std_error error = STD_ERR_OK;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_GROUP);
    STD_ASSERT(node_attr != NULL);

    gpio_group = (sdi_gpio_group_t *)
        calloc(sizeof(sdi_gpio_group_t), 1);

    STD_ASSERT(gpio_group != NULL);

    pin_group_bus = &(gpio_group->bus);
    pin_group_bus->default_level = SDI_PIN_LEVEL_LOW;
    pin_group_bus->default_direction = SDI_PIN_BUS_INPUT;
    pin_group_bus->default_polarity = SDI_PIN_POLARITY_NORMAL;

    create_gpio_group(gpio_group, node_attr);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_LEVEL);
    if (node_attr != NULL) {
        pin_group_bus->default_level = (uint_t) strtoul (node_attr, NULL, 0);
    }

    pin_group_bus->bus.bus_type = SDI_PIN_GROUP_BUS;
    pin_group_bus->bus.bus_init = sdi_gpio_group_driver_init;

    pin_group_bus->ops = &sdi_gpio_group_ops;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    pin_group_bus->bus.bus_id = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_DIRECTION);
    if (node_attr != NULL) {
        if ((strncmp(node_attr, SDI_DEV_ATTR_INPUT_PIN,
                SDI_DEV_ATTR_INPUT_PIN_LEN)) == 0) {
            pin_group_bus->default_direction = SDI_PIN_BUS_INPUT;
        } else if  ((strncmp(node_attr, SDI_DEV_ATTR_OUTPUT_PIN,
                    SDI_DEV_ATTR_OUTPUT_PIN_LEN)) == 0) {
             pin_group_bus->default_direction = SDI_PIN_BUS_OUTPUT;
        } else {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO id %u invalid direction\n",
                __FUNCTION__, __LINE__, pin_group_bus->bus.bus_name,
                pin_group_bus->bus.bus_id);
            STD_ASSERT(false);
        }
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_POLARITY);
    if (node_attr != NULL) {
        if ((strncmp(node_attr, SDI_DEV_ATTR_POLARITY_NORMAL,
                SDI_DEV_ATTR_POLARITY_NORMAL_LEN)) == 0) {
            pin_group_bus->default_polarity = SDI_PIN_POLARITY_NORMAL;
        } else if ((strncmp(node_attr, SDI_DEV_ATTR_POLARITY_INVERTED,
                SDI_DEV_ATTR_POLARITY_INVERTED_LEN)) == 0) {
            pin_group_bus->default_polarity = SDI_PIN_POLARITY_INVERTED;
        } else {
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s GPIO id %u invalid polarity\n",
                __FUNCTION__, __LINE__, pin_group_bus->bus.bus_name,
                pin_group_bus->bus.bus_id);
            STD_ASSERT(false);
        }
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_BUS_NAME);
    STD_ASSERT(node_attr != NULL);
    strncpy(pin_group_bus->bus.bus_name, node_attr, SDI_MAX_NAME_LEN);

    do {
        if (std_mutex_lock_init_non_recursive(&(pin_group_bus->lock)) != STD_ERR_OK) {
            error = BUS_LOCK_INIT_FAILURE;
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s gpio pin group id %u lock init failed\n",
                    __FUNCTION__, __LINE__, pin_group_bus->bus.bus_name,
                pin_group_bus->bus.bus_id);
            break;
        }

        if (sdi_pin_group_bus_register(pin_group_bus) != STD_ERR_OK) {
            error = BUS_REGISTRATION_FAILURE;
            SDI_DEVICE_ERRMSG_LOG("%s:%d Bus %s gpio pin group id %u registration failed\n",
                __FUNCTION__, __LINE__, pin_group_bus->bus.bus_name,
                pin_group_bus->bus.bus_id);
            break;
        }
    } while (0);

    if (error == STD_ERR_OK) {
        *bus = (sdi_bus_hdl_t ) gpio_group;
        sdi_bus_register_device_list(node, (sdi_bus_hdl_t) pin_group_bus);
    } else {
        if (error == BUS_REGISTRATION_FAILURE) {
            free(gpio_group->gpio_group);
            free(gpio_group->gpio_file_tbl);
            std_mutex_destroy(&(pin_group_bus->lock));
        }
        free(pin_group_bus);
    }

    return error;
}


/**
 * SDI gpio group object to hold gpio group registration and
 * initialization function
 * Note:
 * Every bus driver must export function with name
 * sdi_<bus_driver_name>_entry_callbacks
 * so that the driver framework is able to look up and invoke it to get the
 * callbacks
 */
const sdi_bus_driver_t * sdi_gpio_group_entry_callbacks(void)
{
     /*Export Bus Driver table*/
     static const sdi_bus_driver_t sdi_gpio_group_entry = {
        .bus_register = sdi_gpio_group_driver_register,
        .bus_init = sdi_gpio_group_driver_init
     };
     return &sdi_gpio_group_entry;
}
