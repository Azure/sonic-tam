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
 * filename: sdi_sfp.c
 */


/******************************************************************************
 * sdi_sfp.c
 * Implements the driver for SFP. This file contains the APIs for Driver
 * registration and init functions. It also contains the functions which are
 * used to control SFP's using CPLD pins.
 *
 *****************************************************************************/
#include "sdi_sfp.h"
#include "sdi_media.h"
#include "sdi_media_internal.h"
#include "sdi_media_attr.h"
#include "sdi_resource_internal.h"
#include "sdi_common_attr.h"
#include "sdi_device_common.h"
#include "sdi_pin_group_bus_framework.h"
#include "sdi_pin_group_bus_api.h"
#include "std_error_codes.h"
#include "std_assert.h"
#include "std_bit_ops.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* sfp driver init function */
static t_std_error sdi_sfp_init (sdi_device_hdl_t device_hdl);

/* register function for sfp driver */
static t_std_error sdi_sfp_register (std_config_node_t node, void *bus_handle,
                                      sdi_device_hdl_t* device_hdl);

/**
 * Gets the presence status of sfp module
 * resource_hdl[in] - Handle of the sfp resource
 * pres[out]      - presence status
 * return t_std_error
 */
static t_std_error sdi_sfp_presence_get (sdi_resource_hdl_t resource_hdl, bool *pres)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t value = 0;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(pres != NULL);

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    rc = sdi_pin_group_acquire_bus(sfp_priv_data->mod_pres_hdl);
    if (rc != STD_ERR_OK){
        return rc;
    }

    rc = sdi_pin_group_read_level(sfp_priv_data->mod_pres_hdl, &value);
    if (rc != STD_ERR_OK){
        SDI_DEVICE_ERRMSG_LOG("presence status get failed for %s",
                              sfp_device->alias);
    }

    sdi_pin_group_release_bus(sfp_priv_data->mod_pres_hdl);

    if( rc == STD_ERR_OK) {
        if(  (STD_BIT_TEST(value, sfp_priv_data->mod_pres_bitmask)) == 0 ) {
            *pres = false;
        } else {
            *pres = true;
        }
    }
    return rc;
}

/**
 * Set the port LED based on the speed settings of the port
 * resource_hdl[in] - handle to sfp
 * channel[in] - Channel number. Should be 0 in sfp
 * speed[in] - LED mode setting is derived from speed
 * return - standard t_std_error
 */
t_std_error sdi_sfp_led_set (sdi_resource_hdl_t resource_hdl, uint_t channel,
                             sdi_media_speed_t speed)
{
    sdi_device_hdl_t sfp_device = NULL;
    sfp_device_t *sfp_priv_data = NULL;
    t_std_error rc = STD_ERR_OK;
    uint_t value = 0;

    STD_ASSERT(resource_hdl != NULL);

    if(channel != SDI_SFP_CHANNEL_NUM) {
        return (SDI_DEVICE_ERRCODE(EINVAL));
    }

    sfp_device = (sdi_device_hdl_t)resource_hdl;
    sfp_priv_data = (sfp_device_t *)sfp_device->private_data;
    STD_ASSERT(sfp_priv_data != NULL);

    if(sfp_priv_data->port_led_control_flag == false) {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    rc = sdi_pin_group_acquire_bus(sfp_priv_data->port_led.bus_hdl);
    if(rc != STD_ERR_OK) {
        return rc;
    }

    do {
        rc = sdi_pin_group_read_level(sfp_priv_data->port_led.bus_hdl, &value);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("tx control status get failed for %s rc : %d",
                                  sfp_device->alias, rc);
            break;
        }

        if(speed == SDI_MEDIA_SPEED_1G) {
            if(sfp_priv_data->port_led.led_1g_mode_value == SDI_SFP_LED_HIGH_VALUE) {
                STD_BIT_SET(value, sfp_priv_data->port_led.bitmask);
            } else {
                STD_BIT_CLEAR(value, sfp_priv_data->port_led.bitmask);
            }
        } else if(speed == SDI_MEDIA_SPEED_10G) {
            if(sfp_priv_data->port_led.led_10g_mode_value == SDI_SFP_LED_HIGH_VALUE) {
                STD_BIT_SET(value, sfp_priv_data->port_led.bitmask);
            } else {
                STD_BIT_CLEAR(value, sfp_priv_data->port_led.bitmask);
            }
        } else {
            rc = SDI_DEVICE_ERRCODE(EINVAL);
            break;
        }

        rc = sdi_pin_group_write_level(sfp_priv_data->port_led.bus_hdl, value);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("tx control status set failed for %s rc : %d",
                                  sfp_device->alias, rc);
        }
    }while(0);

    sdi_pin_group_release_bus(sfp_priv_data->port_led.bus_hdl);
    return rc;
}

/* Callback handlers for SFP */
static media_ctrl_t sfp_media = {
    .presence_get = sdi_sfp_presence_get,
    .module_monitor_status_get = sdi_sfp_module_monitor_status_get,
    .channel_monitor_status_get = sdi_sfp_channel_monitor_status_get,
    .channel_status_get = sdi_sfp_channel_status_get,
    .tx_control = sdi_sfp_tx_control,
    .tx_control_status_get = sdi_sfp_tx_control_status_get,
    .speed_get = sdi_sfp_speed_get,
    .is_dell_qualified = sdi_sfp_is_dell_qualified,
    .parameter_get = sdi_sfp_parameter_get,
    .vendor_info_get = sdi_sfp_vendor_info_get,
    .transceiver_code_get = sdi_sfp_transceiver_code_get,
    .dell_product_info_get = sdi_sfp_dell_product_info_get,
    .threshold_get = sdi_sfp_threshold_get,
    .module_monitor_threshold_get = sdi_sfp_module_monitor_threshold_get,
    .channel_monitor_threshold_get = sdi_sfp_channel_monitor_threshold_get,
    .module_control = NULL,  /* sdi_media_module_control is not supported on sfp */
    .module_control_status_get = NULL,  /* sdi_media_module_control_status_get is not supported on sfp */
    .module_monitor_get = sdi_sfp_module_monitor_get,
    .channel_monitor_get = sdi_sfp_channel_monitor_get,
    .feature_support_status_get = sdi_sfp_feature_support_status_get,
    .led_set = sdi_sfp_led_set,
    .read = sdi_sfp_read,
    .write = sdi_sfp_write,
    .media_phy_autoneg_set = sdi_sfp_phy_autoneg_set,
    .media_phy_speed_set = sdi_sfp_phy_speed_set,
    .media_phy_mode_set = sdi_sfp_phy_mode_set
};

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t * sdi_sfp_entry_callbacks(void)
{
    /*Export Driver table*/
    static const sdi_driver_t sfp_entry = {
        sdi_sfp_register,
        sdi_sfp_init
    };

    return &sfp_entry;
}

/**
 * initialize the device
 * device_hdl[in] - Handle to the device
 * return         - t_std_error
 */
static t_std_error sdi_sfp_init (sdi_device_hdl_t device_hdl)
{
    return STD_ERR_OK;
}

/*
 * The configuration file format for SFP node is as follows
 *  sfp instance="<port_number>"
 *  addr="<i2c address for sfp>"
 *  mod_sel_bus="<pin group bus name for selecting module>"
 *  mod_sel_value="<pin value which needs to be wriiten on pin group bus for selecting module>"
 *  mod_pres_bus="<pin group bus name for knowing the presence status of sfp>"
 *  mod_pres_bitmask="<presence check bit number for this instance of sfp on mod_pres_bus>"
 *  mod_tx_control_bus="<pin group bus name for tx control>"
 *  mod_tx_control_bitmask="<tx control bit number for this instance of sfp on mod_tx_control_bus>"
 *  mod_rx_los_bus="<pin group bus name for getting rx los(loss of signal)>"
 *  mod_rx_los_bitmask="<rx los bit number for this instance of sfp on mod_rx_los_bus>"
 *  mod_tx_fault_bus="<pin group bus name for getting tx fault>"
 *  mod_tx_fault_bitmak="<tx fault bit number for this instance of sfp on mod_tx_fault_bus>"
 */

/**
 * Register function for SFP devices
 * node[in]         - Device node from configuration file
 * bus_handle[in]   - Parent bus handle of the device
 * device_hdl[out]  - Device handle which is filled by this function
 * return           - t_std_error
 */
static t_std_error sdi_sfp_register (std_config_node_t node, void *bus_handle,
                                     sdi_device_hdl_t* device_hdl)
{
    sdi_device_hdl_t dev_hdl = NULL;
    sfp_device_t *sfp_data = NULL;
    char *node_attr = NULL;
    char *led_node_attr = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_I2C_BUS);

    dev_hdl = calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev_hdl != NULL);

    sfp_data = calloc(sizeof(sfp_device_t), 1);
    STD_ASSERT(sfp_data != NULL);

    dev_hdl->bus_hdl = bus_handle;
    dev_hdl->callbacks = sdi_sfp_entry_callbacks();

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->addr.i2c_addr = (i2c_addr_t) strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->instance = strtoul(node_attr, NULL, 0);
    snprintf(dev_hdl->alias, SDI_MAX_NAME_LEN, "sfp-%u", dev_hdl->instance);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_SELECTION_BUS);
    STD_ASSERT(node_attr != NULL);
    if(strncmp(node_attr, SDI_MEDIA_MODULE_SEL_ALWAYS_ENABLED,
                strlen(SDI_MEDIA_MODULE_SEL_ALWAYS_ENABLED)) == 0 ) {
        /* Only one module present on the channel */
        sfp_data->mod_sel_hdl = NULL;
        sfp_data->mod_sel_value = 0;
    } else {
        sfp_data->mod_sel_hdl = sdi_get_pin_group_bus_handle_by_name(node_attr);

        node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_SELECTION_VALUE);
        STD_ASSERT(node_attr != NULL);
        sfp_data->mod_sel_value = strtoul(node_attr, NULL, 16);
    }

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_PRESENCE_BUS);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_pres_hdl = sdi_get_pin_group_bus_handle_by_name(node_attr);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_PRESENCE_BITMASK);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_pres_bitmask = strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_TX_CONTROL_BUS);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_tx_control_hdl = sdi_get_pin_group_bus_handle_by_name(node_attr);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_TX_CONTROL_BITMASK);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_tx_control_bitmask = strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_RX_LOS_BUS);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_sfp_rx_los_hdl = sdi_get_pin_group_bus_handle_by_name(node_attr);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_RX_LOS_BITMASK);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_sfp_rx_los_bitmask = strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_TX_FAULT_BUS);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_sfp_tx_fault_hdl = sdi_get_pin_group_bus_handle_by_name(node_attr);

    node_attr = std_config_attr_get(node, SDI_MEDIA_MODULE_TX_FAULT_BITMASK);
    STD_ASSERT(node_attr != NULL);
    sfp_data->mod_sfp_tx_fault_bitmask = strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_MEDIA_PORT_LED_BUS);
    if(node_attr == NULL) {
        sfp_data->port_led_control_flag = false;
    } else {
        sfp_data->port_led_control_flag = true;
        sfp_data->port_led.bus_hdl = sdi_get_pin_group_bus_handle_by_name(node_attr);

        led_node_attr = std_config_attr_get(node, SDI_MEDIA_PORT_LED_BITMASK);
        STD_ASSERT(led_node_attr != NULL);
        sfp_data->port_led.bitmask = strtoul(led_node_attr, NULL, 16);

        led_node_attr = std_config_attr_get(node, SDI_MEDIA_PORT_LED_1G_MODE_VALUE);
        STD_ASSERT(led_node_attr != NULL);
        sfp_data->port_led.led_1g_mode_value = strtoul(led_node_attr, NULL, 16);

        led_node_attr = std_config_attr_get(node, SDI_MEDIA_PORT_LED_10G_MODE_VALUE);
        STD_ASSERT(led_node_attr != NULL);
        sfp_data->port_led.led_10g_mode_value = strtoul(led_node_attr, NULL, 16);
    }

    dev_hdl->private_data = (void *)sfp_data;

    sdi_resource_add(SDI_RESOURCE_MEDIA, dev_hdl->alias, (void *)dev_hdl,
                     &sfp_media);

    *device_hdl = dev_hdl;

    return STD_ERR_OK;
}

