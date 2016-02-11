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
 * filename: sdi_seven_segment_pin_led.c
 */

/******************************************************************************
* Implements the driver for seven segment LED which controlled by pin group bus
* for displaying digits on the LED.
******************************************************************************/
#include "sdi_led_internal.h"
#include "sdi_pin_group_bus_api.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_pin_bus_api.h"
#include "sdi_device_common.h"
#include "sdi_common_attr.h"
#include "sdi_resource_internal.h"

#include "std_assert.h"
#include "std_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Attribute used to represent the pin bus which controls the digit LED */
#define SDI_DIGIT_LED_CTRL_BUS  "led_control_bus"

/* Attribute used to represent the number of digits supported by this LED */
#define SDI_LED_NUM_OF_DIGIT_SUPPORTED 1

static t_std_error sdi_seven_segment_led_register (std_config_node_t node, void *bus_handle,
                                                    sdi_device_hdl_t *device_hdl);


static inline bool is_display_string_valid(const char *display_string)
{
    if( ( ((*display_string) >= '0') && ((*display_string) <= '9') ) ||
        ( ((*display_string) >= 'a') && ((*display_string) <= 'f') ) ) {
        return true;
    }

    return false;
}

/**
 * Sets the specified digit on the digital led
 *
 * resource_hdl[in] - Handle of the resource
 * display_string[in] - Value to be displayed
 *
 * return t_std_error
 */
static t_std_error sdi_seven_segment_led_set (void *resource_hdl, const char *display_string)
{
    sdi_device_hdl_t digit_led = NULL;
    uint_t led_set_value = 0;
    t_std_error rc = STD_ERR_OK;
    sdi_pin_group_bus_hdl_t bus_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(display_string != NULL);

    digit_led = (sdi_device_hdl_t) resource_hdl;

    if (strlen(display_string) != SDI_LED_NUM_OF_DIGIT_SUPPORTED) {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    if (is_display_string_valid(display_string) != true ) {
        return SDI_DEVICE_ERRCODE(EINVAL);
    }

    led_set_value = strtoul(display_string, NULL, 16);

    bus_hdl = (sdi_pin_group_bus_hdl_t)digit_led->bus_hdl;

    rc = sdi_pin_group_acquire_bus(bus_hdl);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    rc = sdi_pin_group_write_level(bus_hdl, led_set_value);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("Writing the value to pin group bus is failed with rc : %d",
                              rc);
    }
    sdi_pin_group_release_bus(bus_hdl);
    return rc;
}

/**
 * Turn-on the seven segment LED
 *
 * resource_hdl[in] - Handle of the resource
 *
 * return t_std_error
 */
static t_std_error sdi_seven_segment_led_on (void *resource_hdl)
{
    sdi_device_hdl_t digit_led = NULL;
    sdi_pin_bus_hdl_t led_ctrl_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);

    digit_led = (sdi_device_hdl_t) resource_hdl;
    led_ctrl_hdl = (sdi_pin_bus_hdl_t)digit_led->private_data;
    STD_ASSERT(led_ctrl_hdl != NULL);

    return sdi_pin_write_level(led_ctrl_hdl, SDI_PIN_LEVEL_HIGH);
}

/**
 * Turn-off the digital led
 *
 * resource_hdl[in] - Handle of the resource
 *
 * return t_std_error
 */
static t_std_error sdi_seven_segment_led_off (void *resource_hdl)
{
    sdi_device_hdl_t digit_led = NULL;
    sdi_pin_bus_hdl_t led_ctrl_hdl = NULL;

    STD_ASSERT(resource_hdl != NULL);

    digit_led = (sdi_device_hdl_t) resource_hdl;
    led_ctrl_hdl = (sdi_pin_bus_hdl_t)digit_led->private_data;
    STD_ASSERT(led_ctrl_hdl != NULL);

    return sdi_pin_write_level(led_ctrl_hdl, SDI_PIN_LEVEL_LOW);
}

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t * sdi_seven_segment_led_entry_callbacks (void)
{
    /*Export Driver table*/
    static const sdi_driver_t sdi_seven_segment_led_entry = {
        sdi_seven_segment_led_register,
        /* Init function will be set to NULL for pin-led. All pins are set to
         * default by pin-bus driver */
        NULL
    };

    return &sdi_seven_segment_led_entry;
}


static sdi_digital_display_led_t sdi_seven_segment_led = {
    sdi_seven_segment_led_on,
    sdi_seven_segment_led_off,
    sdi_seven_segment_led_set
};

/**
 * xml format:
 * <sdi_seven_segment_led instance="<instance number>"
 *                          led_control_bus="<bus name which controls the LED>"
 *                          alias="<led name>" />
 */

/**
 * Register function for single digit dispaly led which are driving on pin group
 * bus
 *
 * node[in] - Device node from configuration file
 * bus_handle[in] - Parent bus handle of the device
 * device_hdl[out] - Device handler
 *
 * return std_error_t
 */
static t_std_error sdi_seven_segment_led_register (std_config_node_t node, void *bus_handle,
                                                    sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t dev_hdl = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT( ((sdi_bus_t*)bus_handle)->bus_type == SDI_PIN_GROUP_BUS);

    dev_hdl = (sdi_device_hdl_t) calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev_hdl != NULL);

    dev_hdl->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->instance = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL){
        snprintf(dev_hdl->alias, SDI_MAX_NAME_LEN, "seven_segment_led_%u",
                 dev_hdl->instance);
    } else {
        safestrncpy(dev_hdl->alias, node_attr, SDI_MAX_NAME_LEN);
    }

    node_attr = std_config_attr_get(node, SDI_DIGIT_LED_CTRL_BUS);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->private_data = (void *) sdi_get_pin_bus_handle_by_name(node_attr);

    dev_hdl->callbacks = sdi_seven_segment_led_entry_callbacks();

    sdi_resource_add(SDI_RESOURCE_DIGIT_DISPLAY_LED, dev_hdl->alias, (void*)dev_hdl, &sdi_seven_segment_led);

    *device_hdl = dev_hdl;

    return STD_ERR_OK;
}
