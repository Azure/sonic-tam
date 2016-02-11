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
 * filename: sdi_pseudo_bus.c
 */


/******************************************************************************
 * Creates and registers a pseudo bus.
 * Ex: in i2c-mux-gpio: an i2c(parent) bus is multiplexed by 2 gpio pins to 4
 * i2c bus: four i2c buses can't be direclty attached under the parent i2c bus
 * as the mux selection is controlled by 2 gpio pins that itself is a pin bus.
 * All the four i2c bus have hybrid parent: i2c bus and pin buses. To handle
 * such cases, a virtual bus is created, and the virtual bus registers the
 * i2c-mux-gpio device that exports 4 i2c bus, where every i2c bus will have
 * pointer to mux data structure that defines parent i2c bus, mux select gpio
 * pins.
 *
 * 1. Pseudo bus is required to adhere to our design: framework registers buses
 * first, and buses registers devices, devices could also registers buses(if
 * applicable, like in this case specified below).
 * 2. This will create sdi_bus_t with bus type SDI_PSEUDO_BUS, instance=<num>,
 * bus_name=”pseudo_bus_0”(just to satisfy bus framework definition/in future
 * could have grouping of same devices under a particular pseudobus than having
 * just one pseudo bus for overall sdi ), and register with bus framework.
 * 3. Pseudo bus data structure will initialize a list and adds every child device
 * handle sdi_device_hdl_t directly attached to bus
 * 4. As part of pseudo_bus registration, will register every child device
 * attached to pseudo_bus
 * 5. As part of bus-initialization, all child devices will be initialized using
 * pseudo_bus handle and child device handle.
 *
 * <pseudo_bus>
 *        <device1 ..>
 *        </device1>
 *        <device2 ..>
 *        </device2>
 * </pseudo_bus>
 *
 *****************************************************************************/
#include "sdi_bus_framework.h"
#include "std_assert.h"
#include "std_config_node.h"
#include "std_utils.h"

#include <stdlib.h>
#include <string.h>

static t_std_error sdi_pseudo_bus_init(sdi_bus_hdl_t bus)
{
    sdi_bus_init_device_list(bus);
    return STD_ERR_OK;
}

static t_std_error sdi_pseudo_bus_register (std_config_node_t node,
                                            sdi_bus_hdl_t *bus_hdl)
{
    sdi_bus_hdl_t bus = NULL;

    STD_ASSERT(bus_hdl != NULL);

    bus = (sdi_bus_hdl_t) calloc(sizeof(sdi_bus_t), 1);
    STD_ASSERT(bus != NULL);
    bus->bus_type = SDI_PSEUDO_BUS;
    bus->bus_id = 0;
    bus->bus_init = sdi_pseudo_bus_init;
    safestrncpy(bus->bus_name, std_config_name_get(node), SDI_MAX_NAME_LEN);
    sdi_bus_register(bus);
    *bus_hdl = bus;

    sdi_bus_register_device_list(node, bus);

    return STD_ERR_OK;
}

/**
 * SDI Bus Driver Object required to register and initialize pseudo bus
 * Note:
 * Every bus driver must export function with name
 * sdi_<bus_driver_name>_entry_callbacks
 * so that the driver framework is able to look up and invoke it to get the
 * callbacks
 */
const sdi_bus_driver_t * sdi_pseudo_bus_entry_callbacks(void)
{
     /*Export Bus Driver table*/
     static const sdi_bus_driver_t sdi_pseudo_bus_entry = {
        .bus_register = sdi_pseudo_bus_register,
        .bus_init = sdi_pseudo_bus_init
     };
     return &sdi_pseudo_bus_entry;
}
