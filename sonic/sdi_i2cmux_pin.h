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
 * filename: sdi_i2cmux_pin.h
 */


/******************************************************************************
 * @file sdi_i2cmux_pin.h
 * @brief Creates an i2cmux device and registers i2c bus for every mux channel.
 * i2c-mux-pin: its an i2c multiplexer, that multiplexes one i2c bus onto 'n'
 * i2c bus based on pin selection. This device is attached to pseudo bus as it
 * has references to i2c bus (being multiplexed) and pin bus (used for mux
 * selection).
 *
 * SCL/SDA of the master I2C bus is multiplexed to bus segment 1..M
 * according to the settings of the GPIO pins 1..N.
 *
 *****************************************************************************/
/*
 * For example: an i2c bus from cpu multiplexed using i2c mux via gpio 1..N
 * is shown below
 *
 *  ----------   I2C BUS    ----------  Bus segment 1  --------------
 * |          |  SCL/SDA   |          |-------------- | Device set 1 |
 * |          |------------|          |                   --------------
 * |          |            |   I2C    | Bus segment 2  --------------
 * |  CPU     | GPIO 1..N  |   MUX    |---------------| Device set 2 |
 * |          |------------|          |                --------------
 * |          |            |          | Bus segment M  --------------
 * |          |            |          |---------------| Device set 3 |
 *  ----------              ----------                 --------------
 *
 */

#ifndef __SDI_I2CMUX_PIN_H__
#define __SDI_I2CMUX_PIN_H__

#include "sdi_i2c.h"
#include "sdi_pin_group.h"

/**
 * @struct sdi_i2cmux_pin_t
 * @brief data structure for i2cmux_pin created during device registration
 * contains
 * - bus id of i2c bus being multiplexed
 * - bus id of pingroup used for multiplexing
 * - handle of i2c bus and pin group bus used in selection logic
 */
typedef struct sdi_i2cmux_pin_ {
    sdi_pin_group_bus_hdl_t pingroup_hdl; /**< pin group bus handle used to
                                                select a mux channel */
    char pingroup_bus_name[SDI_MAX_NAME_LEN]; /**< pin group bus name */
    sdi_i2c_bus_hdl_t i2cbus_hdl; /**< parent i2c bus handle */
    char i2c_bus_name[SDI_MAX_NAME_LEN]; /**< parent i2c bus name */
    std_mutex_type_t mux_lock; /**< lock to synchronize accessing i2c mux */
    sdi_bus_list_t channel_list; /**< list to maintain i2c mux channel */
} sdi_i2cmux_pin_t;

/**
 * @typedef sdi_i2cmux_pin_hdl_t
 * Device handle for sdi i2cmux_pin device
 */
typedef sdi_i2cmux_pin_t *sdi_i2cmux_pin_hdl_t;

/**
 * @struct sdi_i2cmux_pin_chan_bus_t
 * @brief data structure for i2c mux channel
 * contains
 * - i2c bus object
 * - pointer to i2cmux_pin
 * - channel id to select to activate this i2c mux channel
 * - lock to syncrhonize access to this i2c bus
 */
typedef struct sdi_i2cmuxchan_bus_ {
    sdi_i2c_bus_t i2c_bus; /**< i2c bus object for every i2c mux channel */
    uint_t i2c_mux_channel; /**< channel number used to select this i2c mux channel */
    sdi_i2cmux_pin_t *i2c_mux; /**< reference to i2c mux pin device to access i2c
                                bus handle and pin group bus handle */
} sdi_i2cmux_pin_chan_bus_t;

/**
 * @typedef sdi_i2cmux_pin_chan_bus_hdl_t
 * Bus handle for mux channel exported by sdi i2cmux_pin device
 */
typedef sdi_i2cmux_pin_chan_bus_t *sdi_i2cmux_pin_chan_bus_handle_t;

#endif /* __SDI_I2CMUX_PIN_H__ */
