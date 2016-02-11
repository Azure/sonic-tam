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
 * filename: sdi_cpld.c
 */


/******************************************************************************
 * CPLD produces pins and pin groups to
 * - indicate presence status of components like fan tray, psu, media and
 * sensors etc.
 * - reset CPU, NIC, NPU, USB and few system components,
 * - control ON/OFF LED, Multi-Color LED, Digit LED
 * - configure media : mode selection, mux selection, tx enable/disable, fault
 * and rx loss configuration
 * SDI CPLD driver creates pins and pin groups as specified by platform
 * configuration and registers with pin/pin group framework.
 *
 * Depending on the platform design, CPLD could be accessed over I2C or LPC bus.
 *
 * todo
 * - Currently cpld access over i2c bus is only supported. Extend the support for
 * cpld over LPC/other bus when LPC/other Bus Driver is supported
 * - Only cpld registers byte access is supported for now (width=1byte). Extend
 * the support to more than 1 byte when required.
 * - max width of pin group is limited by uint_t data type. Extend the
 * support to 'n' width when required.
 * - inverted polarity is handled by cpld driver now and not by cpld, when cpld
 * supports invert polarity push the logic to cpld
 *
 * note
 * - CPLD pin corresponds to a bit in a CPLD register. In configuration,
 * every cpld pin needs to contain attribute for cpld reigster address and the
 * bit offset.
 * - CPLD Pin group could be formed by set of bits in a cpld register or group
 * of consecutive cpld registers, with start and end offset on same register
 * or on different register with-in the group. In configuration, a cpld pin group
 * needs to contain attributes for cpld start register address, end register
 * address (if cpld pin group spans across muiltple cpld registers), start bit
 * offset and end bit offset.
 *
 * Refer to below xml format for exact configuration format:
 *
 * xml file format:
 * <cpld instance=0 alias=master addr="0x64">
 *        <sdi_cpld_pin instance="2" addr="0x7" offset="0x1" bus_name="master_led"
 *            direction="out" level="1" polarity="normal">
 *      </sdi_cpld_pin>
 *        <!-- addr is cpld pin address within cpld --!>
 *        <!-- offset is cpld pin's offset within cpld addr register --!>
 *
 *      <sdi_cpld_pin_group instance="1" start_addr="0x7" start_offset="5"
 *      end_offset="8"  bus_name="board_revision" direction="in"
 *      polarity="normal">
 *        </sdi_cpld_pin_group>
 *        <!-- start_addr is cpld pin group's register's start address --!>
 *        <!- start_offset is cpld pin group's bit offset within start_addr --!>
 *        <!-- end_addr is cpld pin group's register's end address --!>
 *        <!- end_offset is cpld pin group's bit offset within end_addr --!>
 *
 *      <!-- width of cpld register is optional if its 1byte --!>
 *        <!-- level is optional for input pin --!>
 * </cpld>
 *****************************************************************************/

#include "sdi_cpld.h"
#include "sdi_device_common.h"
#include "sdi_bus_framework.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_pin_group_bus_framework.h"
#include "std_assert.h"
#include "sdi_pin_bus_attr.h"
#include "sdi_bus_attr.h"
#include "sdi_common_attr.h"
#include "std_mutex_lock.h"
#include "std_utils.h"
#include "sdi_cpld_attr.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* cpld device driver registration function */
static t_std_error sdi_cpld_register(std_config_node_t node, void *bus_handle,
                                     sdi_device_hdl_t* device_hdl);

/* cpld device driver initialization function */
static t_std_error sdi_cpld_init(sdi_device_hdl_t device_hdl);

/*
 * cpld device driver object
 */
sdi_driver_t sdi_cpld_entry = {
    sdi_cpld_register,
    sdi_cpld_init
};

/*
 * Create cpld device and register cpld pin/pin group bus attached to the
 * cpld device
 * param[in] node - cpld device configuration node
 * param[in] bus_handle - bus handle to which cpld is attached.
 * param[out] device_hdl - populated with cpld device handle on successful registration
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_cpld_register(std_config_node_t node, void *bus_handle,
                                     sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    std_config_node_t cur_node = NULL;
    sdi_device_hdl_t dev_hdl = NULL;
    sdi_bus_driver_t *bus_driver = NULL;
    sdi_bus_hdl_t bus = NULL;
    t_std_error err = STD_ERR_OK;
    sdi_cpld_pin_t *cpld_pin = NULL;
    sdi_cpld_pin_group_t *cpld_pin_group = NULL;
    sdi_cpld_dev_hdl_t cpld_dev_hdl = NULL;

    dev_hdl = (sdi_device_hdl_t) calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(dev_hdl != NULL);

    cpld_dev_hdl = (sdi_cpld_dev_hdl_t) calloc(sizeof(sdi_cpld_device_t), 1);
    STD_ASSERT(cpld_dev_hdl != NULL);

    dev_hdl->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    dev_hdl->instance = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (node_attr == NULL) {
        snprintf(dev_hdl->alias, SDI_MAX_NAME_LEN, "sdi-cpld-%d",
            dev_hdl->instance);
    } else {
        safestrncpy(dev_hdl->alias,node_attr,SDI_MAX_NAME_LEN);
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_START_ADDR);
    STD_ASSERT(node_attr != NULL);
    cpld_dev_hdl->start_addr = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_END_ADDR);
    STD_ASSERT(node_attr != NULL);
    cpld_dev_hdl->end_addr = (uint_t) strtoul (node_attr, NULL, 0);

    STD_ASSERT(cpld_dev_hdl->start_addr < cpld_dev_hdl->end_addr);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_WIDTH);
    if (node_attr == NULL) {
        cpld_dev_hdl->width = SDI_CPLD_DEFAULT_REGISTER_WIDTH;
    } else {
        cpld_dev_hdl->width = (uint_t) strtoul (node_attr, NULL, 0);
        STD_ASSERT(cpld_dev_hdl->width == SDI_CPLD_DEFAULT_REGISTER_WIDTH);
    }

    dev_hdl->callbacks = &sdi_cpld_entry;

    dev_hdl->private_data = cpld_dev_hdl;

    if (((sdi_bus_hdl_t)bus_handle)->bus_type == SDI_I2C_BUS) {
        node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
        STD_ASSERT(node_attr != NULL);
        dev_hdl->addr.i2c_addr = (i2c_addr_t) strtoul(node_attr, NULL, 16);
    }
    sdi_bus_init_list(&dev_hdl->bus_list);

    *device_hdl = dev_hdl;

    for (cur_node = std_config_get_child(node); cur_node != NULL;
        cur_node = std_config_next_node(cur_node)) {

        bus_driver = sdi_bus_get_symbol(cur_node);
        STD_ASSERT (bus_driver != NULL);

        err = bus_driver->bus_register(cur_node, &bus);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d registering bus %s failed %d\n",
                                  __FUNCTION__, __LINE__,
                                  std_config_name_get(cur_node));
        }

        sdi_bus_enqueue_list(&dev_hdl->bus_list, bus);
        if (bus->bus_type == SDI_PIN_BUS) {
            cpld_pin = (sdi_cpld_pin_t *) bus;
            cpld_pin->cpld_hdl = dev_hdl;
            STD_ASSERT(cpld_pin->addr >= cpld_dev_hdl->start_addr);
            STD_ASSERT(cpld_pin->addr <= cpld_dev_hdl->end_addr);
        } else if (bus->bus_type == SDI_PIN_GROUP_BUS) {
            cpld_pin_group = (sdi_cpld_pin_group_t *) bus;
            cpld_pin_group->cpld_hdl = dev_hdl;
            STD_ASSERT(cpld_pin_group->start_addr >= cpld_dev_hdl->start_addr);
            STD_ASSERT(cpld_pin_group->start_addr <= cpld_dev_hdl->end_addr);
            STD_ASSERT(cpld_pin_group->end_addr >= cpld_dev_hdl->start_addr);
            STD_ASSERT(cpld_pin_group->end_addr <= cpld_dev_hdl->end_addr);
        } else {
            STD_ASSERT(false);
        }

        bus = NULL;
        bus_driver = NULL;
    }

    return STD_ERR_OK;
}

/*
 * Initialize each pin/pingroup bus attached to cpld device
 * param[in] device_hdl - cpld device handle
 * param[in] STD_ERR_OK on success
 */
static t_std_error sdi_cpld_init(sdi_device_hdl_t device_hdl)
{
    sdi_init_bus_for_each_bus_in_list(&device_hdl->bus_list,
                                      sdi_bus_init, NULL);
    return STD_ERR_OK;
}
