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
 * filename: sdi_io_bus.c
 */


/******************************************************************************
 * Defines data structure for IO bus.
 * IO bus may contain devices which produces pins and pin groups to
 * - indicate presence status of components like fan tray, psu, etc.
 * - reset CPU, NIC, NPU, USB and few system components,
 * SDI IO devices under IO bus creates pins and pin groups as specified by
 * platform configuration and registers with pin/pin group framework.
 * Depending on the platform design, IO could be accessed.
 *
 * xml file format:
 *      <sdi_io bus_name="io_bus">
 *         <!-- IO devices under IO Bus.  --!>
 *      </sdi_io>
 *****************************************************************************/

#include "sdi_bus_framework.h"
#include "sdi_device_common.h"
#include "sdi_bus_attr.h"
#include "sdi_common_attr.h"
#include "std_assert.h"
#include "std_utils.h"
#include <stdlib.h>

typedef sdi_bus_hdl_t sdi_io_bus_hdl_t;

/*
 * Initialize io driver.
 * param[in] bus_hdl - bus handle
 * return STD_ERR_OK.
 */
static t_std_error sdi_io_driver_init(sdi_bus_hdl_t bus_hdl)
{
    sdi_bus_init_device_list(bus_hdl);
    return  STD_ERR_OK;
}

/*
 * Create a io bus and register with bus framework
 * param[in] node     - config node
 * param[out] bus_hdl - filled with bus handle on successful registration
 * return STD_ERR_OK for success and the respective error code in case of failure.
 */
static t_std_error sdi_io_driver_register(std_config_node_t node,
                                                            sdi_bus_hdl_t *bus_hdl)
{
    char *node_attr = NULL;
    t_std_error error = STD_ERR_OK;
    sdi_io_bus_hdl_t io_bus = (sdi_io_bus_hdl_t )calloc(sizeof(sdi_bus_t), 1);

    STD_ASSERT(io_bus != NULL);

    io_bus->bus_type = SDI_IO_BUS;
    io_bus->bus_init = sdi_io_driver_init;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_BUS_NAME);
    STD_ASSERT(node_attr != NULL);
    safestrncpy(io_bus->bus_name, node_attr, SDI_MAX_NAME_LEN);

    error = sdi_bus_register((sdi_bus_hdl_t )io_bus);
    if (error != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d registration failed %s with error %d\n",
                              __FUNCTION__, __LINE__, io_bus->bus_name, error);
        free(io_bus);
        return error;
    }

    *bus_hdl = (sdi_bus_hdl_t ) io_bus;

    sdi_bus_register_device_list(node, (sdi_bus_hdl_t) io_bus);

    return error;
}

/*
 * sdi_io_entry_callbacks
 * SDI IO Bus Driver Object to hold this driver's registration and initialization
 * function
 *
 * Note:
 * Every bus driver must export function with name
 * sdi_<bus_driver_name>_entry_callbacks
 * so that the driver framework is able to look up and invoke it to get the
 * callbacks
 */
const sdi_bus_driver_t * sdi_io_entry_callbacks(void)
{
     /*Export Bus Driver table*/
     static const sdi_bus_driver_t sdi_io_entry = {
        .bus_register = sdi_io_driver_register,
        .bus_init = sdi_io_driver_init
     };
     return &sdi_io_entry;
}
