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
 * filename: sdi_cpld.h
 */


/******************************************************************************
 * Defines data structure for CPLD, and pins/pin groups produced by CPLD
 * CPLD produces pins and pin groups to
 * - indicate presence status of components like fan tray, psu, media and
 * sensors etc.
 * - reset CPU, NIC, NPU, USB and few system components,
 * - control ON/OFF LED, Multi-Color LED, Digit LED
 * - configure media : mode selection, mux selection, tx enable/disable, fault
 * and rx loss configuration
 * SDI CPLD driver creates pins and pin groups as specified by platform
 * configuration and registers with pin/pin group framework.
 * Depending on the platform design, CPLD could be accessed over I2C or LPC bus.
 *****************************************************************************/

#ifndef __SDI_CPLD_H__
#define __SDI_CPLD_H__

#include "sdi_pin.h"
#include "sdi_pin_group.h"

/**
 * sdi_cpld_device_t
 * SDI CPLD Device Object
 */
typedef struct sdi_cpld_device_ {
    uint_t width; /* Width of every cpld register in bytes */
    uint_t start_addr; /* CPLD Register start offset address for the pin group */
    uint_t end_addr; /* CPLD Register end offset address for the pin group */
} sdi_cpld_device_t;

/**
 * sdi_cpld_dev_hdl_t
 * Handle to cpld device Object
 */
typedef sdi_cpld_device_t *sdi_cpld_dev_hdl_t;

/**
 * sdi_cpld_pin_t
 * SDI CPLD Pin Bus Object Registered by every CPLD Pin
 */
typedef struct sdi_cpld_pin_ {
    sdi_pin_bus_t bus; /* SDI PIN Bus Object */
    uint_t addr; /* CPLD Register offset address used for the pin */
    uint_t offset; /* CPLD Register's bit offset used for the pin counted from
                        1 starting LSB */
    sdi_device_hdl_t cpld_hdl; /* CPLD Device Handle */
} sdi_cpld_pin_t;

/**
 * sdi_cpld_pin_group_t
 * SDI CPLD Pin Group Bus Object Registered by every CPLD Pin Group
 */
typedef struct sdi_cpld_pin_group_ {
    sdi_pin_group_bus_t bus; /* SDI cpld pin group Bus object */
    uint_t start_addr; /* CPLD Register start offset address for the pin group */
    uint_t start_offset; /* CPLD Register start bit offset address for the pin
                            group counted from 1 starting LSB */
    uint_t end_addr; /* CPLD Register end offset address for the pin group */
    uint_t end_offset; /* CPLD Register end bit offset address for the pin
                            group counted from 1 starting LSB */
    uint_t length; /* number of cpld registers in the pin group */
    sdi_device_hdl_t cpld_hdl; /* CPLD Device Handle */
} sdi_cpld_pin_group_t;

#endif /* __SDI_CPLD_H__ */
