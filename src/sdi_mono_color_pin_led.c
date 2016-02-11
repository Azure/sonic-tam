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
 * filename: sdi_mono_color_pin_led.c
 */


/******************************************************************************
* Implements the driver for mono color led
* LED Device controlled via pin bus for mono-color LEDs and pin
* group bus for multi-color LEDs and registers SDI_RESOURCE_LED with callbacks
* required to turn on/off LED Device
*
* xml format:
* <sdi_mono_color_led instance="9" led_on_value="1" led_off_value="0" alias="post_in_progress_led" />
*
* note
* 1) By default, led is on by driving high on pin bus, and off by driving low
* on pin bus. In case where the polarity is inverted, pin bus should specify
* the polarity as inverted to make this driver transparent of polarity.
*
******************************************************************************/

#include "std_error_codes.h"
#include "sdi_resource_internal.h"
#include "sdi_led_internal.h"
#include "sdi_pin.h"
#include "sdi_pin_bus_api.h"
#include "sdi_pin_group_bus_api.h"
#include "std_assert.h"
#include "sdi_common_attr.h"
#include "sdi_device_common.h"
#include "std_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SDI_LED_OFF_NOT_SUPPORTED -1

/*Node name used for representing ON state for mono color led*/
#define SDI_DEV_ATTR_MONO_COLOR_LED_ON_VALUE    "led_on_value"

/*Node name used for representing OFF state for mono color led*/
#define SDI_DEV_ATTR_MONO_COLOR_LED_OFF_VALUE    "led_off_value"

/* Private data for LEDs, used for pin groups */
typedef struct mono_color_pin_group_led {
    uint_t on_value;
    uint_t off_value;
} mono_color_pin_group_led_t;

static t_std_error sdi_mono_color_pin_led_register (std_config_node_t node,
                                                    void *bus_handle,
                                                    sdi_device_hdl_t* device_hdl);

/*Export Driver table*/
sdi_driver_t sdi_mono_color_pin_led_entry = {
    sdi_mono_color_pin_led_register,
/*Init function will be set to NULL for pin-led. All pins are set to default by
 * pin-bus driver*/
    NULL
};

/**
 * This API will turn-on the LED and it needs to be used on LEDs which are
 * driving on pin-bus
 *
 * resource_hdl[in] - handle of the led
 *
 * return t_std_error
 */
t_std_error sdi_mono_color_pin_led_sensor_on (void *resource_hdl)
{
    sdi_device_hdl_t dev_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);
    dev_hdl = (sdi_device_hdl_t) resource_hdl;

    return sdi_pin_write_level((sdi_pin_bus_hdl_t)dev_hdl->bus_hdl,
                               SDI_PIN_LEVEL_HIGH);
}

/**
 * This API will turn-off the LED and it needs to be used on LEDs which are
 * driving on pin-bus
 *
 * resource_hdl[in] - handle of the led
 *
 * return t_std_error
 */
t_std_error sdi_mono_color_pin_led_sensor_off (void *resource_hdl)
{
    sdi_device_hdl_t dev_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);
    dev_hdl = (sdi_device_hdl_t) resource_hdl;

    return sdi_pin_write_level((sdi_pin_bus_hdl_t)dev_hdl->bus_hdl,
                               SDI_PIN_LEVEL_LOW);
}

sdi_led_sensor_t sdi_mono_color_pin_led_sensor = {
    sdi_mono_color_pin_led_sensor_on,
    sdi_mono_color_pin_led_sensor_off
};

/**
 * This API will turn-on the LED and it needs to be used on LEDs which are
 * driving on pin group bus
 *
 * resource_hdl[in] - handle of the led
 *
 * return t_std_error
 */
t_std_error sdi_mono_color_pin_group_led_sensor_on (void *resource_hdl)
{
    sdi_device_hdl_t dev_hdl = NULL;
    mono_color_pin_group_led_t *led = NULL;
    t_std_error rc = STD_ERR_OK;
    sdi_pin_group_bus_hdl_t bus_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);
    dev_hdl = (sdi_device_hdl_t) resource_hdl;

    led = (mono_color_pin_group_led_t *)dev_hdl->private_data;
    bus_hdl = (sdi_pin_group_bus_hdl_t)dev_hdl->bus_hdl;

    rc = sdi_pin_group_acquire_bus(bus_hdl);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Acquiring lock is failed with error no : %d", rc);
        return rc;
    }

    rc = sdi_pin_group_write_level(bus_hdl, led->on_value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Writing the value to pin group bus is failed with rc : %d",
                              rc);
    }
    sdi_pin_group_release_bus(bus_hdl);

    return rc;
}

/**
 * This API will turn-off the LED and it needs to be used on LEDs which are
 * driving on pin group bus
 *
 * resource_hdl[in] - handle of the led
 *
 * return t_std_error
 */
t_std_error sdi_mono_color_pin_group_led_sensor_off (void *resource_hdl)
{
    sdi_device_hdl_t dev_hdl = NULL;
    mono_color_pin_group_led_t *led = NULL;
    t_std_error rc = STD_ERR_OK;
    sdi_pin_group_bus_hdl_t bus_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);
    dev_hdl = (sdi_device_hdl_t) resource_hdl;

    led = (mono_color_pin_group_led_t *)dev_hdl->private_data;

    if (led->off_value == SDI_LED_OFF_NOT_SUPPORTED){
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    bus_hdl = (sdi_pin_group_bus_hdl_t)dev_hdl->bus_hdl;
    rc = sdi_pin_group_acquire_bus(bus_hdl);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Acquiring lock is failed with errorno : %d", rc);
        return rc;
    }

    rc = sdi_pin_group_write_level(bus_hdl, led->off_value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Writing the value to pin group bus is failed with rc : %d",
                               rc);
    }
    sdi_pin_group_release_bus(bus_hdl);
    return rc;
}

sdi_led_sensor_t sdi_mono_color_pin_group_led_sensor = {
    sdi_mono_color_pin_group_led_sensor_on,
    sdi_mono_color_pin_group_led_sensor_off
};

/**
 * Register function for pin LEDs. Can be used on all LEDs are driving on pin
 * bus and pin group bus
 *
 * node [in] - Device node from configuration file
 * bus_handle [in] - Parent bus handle of the device
 * device_hdl[out] - Pointer to the device handle which is filled by this
 * function
 *
 * return std_error_t
 */
static t_std_error sdi_mono_color_pin_led_register (std_config_node_t node,
                                                    void *bus_handle,
                                                    sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t dev_hdl = NULL;
    sdi_led_sensor_t *ops;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT((((sdi_bus_t*)bus_handle)->bus_type == SDI_PIN_BUS) ||
               (((sdi_bus_t*)bus_handle)->bus_type == SDI_PIN_GROUP_BUS));

    dev_hdl = (sdi_device_hdl_t) calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev_hdl != NULL);

    dev_hdl->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->instance = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL){
        snprintf(dev_hdl->alias, SDI_MAX_NAME_LEN, "mono-color-led-%u",
                 dev_hdl->instance);
    } else {
        safestrncpy(dev_hdl->alias, node_attr, SDI_MAX_NAME_LEN);
    }

    if (sdi_is_pin_group_bus(bus_handle) == true){
        mono_color_pin_group_led_t *led = NULL;

        led = (mono_color_pin_group_led_t *) calloc (sizeof(mono_color_pin_group_led_t), 1);
        STD_ASSERT(led != NULL);

        node_attr = std_config_attr_get(node,
                                        SDI_DEV_ATTR_MONO_COLOR_LED_ON_VALUE);
        STD_ASSERT(node_attr != NULL);
        led->on_value = (uint_t) strtoul (node_attr, NULL, 0);

        node_attr = std_config_attr_get(node,
                                        SDI_DEV_ATTR_MONO_COLOR_LED_OFF_VALUE);
        if (node_attr != NULL){
            led->off_value = (uint_t) strtoul (node_attr, NULL, 0);
        } else {
            /*For some of multicolor LED's OFF is not supported*/
            led->off_value = SDI_LED_OFF_NOT_SUPPORTED;
        }
        ops = &sdi_mono_color_pin_group_led_sensor;
        dev_hdl->private_data = (void *) led;
    } else {
        dev_hdl->private_data = NULL;
        ops = &sdi_mono_color_pin_led_sensor;
    }

    dev_hdl->callbacks = &sdi_mono_color_pin_led_entry;

    sdi_resource_add(SDI_RESOURCE_LED, dev_hdl->alias, (void*)dev_hdl, ops);

    if(device_hdl != NULL) {
        *device_hdl = dev_hdl;
    }

    return STD_ERR_OK;
}

