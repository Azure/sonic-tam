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
 * filename: sdi_gpio.h
 */


/******************************************************************************
 * @file sdi_gpio.h
 * @brief Defines data structure for sdi gpio pin and gpio pin group.
 * Gpio pin and gpio pin group are used to access gpios exported by kernel gpio
 * driver via sysfs interface.
 *****************************************************************************/

#ifndef __SDI_GPIO_H___
#define __SDI_GPIO_H___

#include "sdi_pin.h"
#include "sdi_pin_group.h"

#include <stdio.h>

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
 * @def Direction in/out string maximum length includes null terminator, \n
 * Used while writing direcion to gpio sysfs direction file.
 */
#define MAX_DIRECTION_LEN        5
/**
 * @def Value 0/1 string maximum length includes null terminator, \n
 * Used while writing value to gpio sysfs value/invert_low file.
 */
#define MAX_VALUE_STR_LEN        3

/**
 * @struct sdi_gpio_file_fd_tbl_t
 * SDI GPIO File handle table. Stores file pointer for sysfs gpio value,
 * direction and active_low file path
 */
typedef struct sdi_gpio_file_descriptor_table {
    int level_fd; /**< SDI GPIO Pin level file descriptor */
    int direction_fd; /**< SDI GPIO Pin direction file descriptor */
    int polarity_fd; /**< SDI GPIO Pin polarity file descriptor */
} sdi_gpio_file_fd_tbl_t;

/**
 * @struct sdi_gpio_pin_t
 * SDI PIN Bus Structure Registered by every exported GPIO PIN
 */
typedef struct sdi_gpio_pin {
    sdi_pin_bus_t bus; /**< SDI Pin Bus Object */
    uint_t gpio_num; /**< SDI GPIO Pin number */
    sdi_gpio_file_fd_tbl_t *gpio_file_tbl; /**< SDI GPIO File hdl table */
} sdi_gpio_pin_t;

/**
 * @struct sdi_gpio_group_t
 * SDI PIN Group Bus Structure Registered by every exported GPIO PIN Group
 */
typedef struct sdi_gpio_pingroup {
    sdi_pin_group_bus_t bus; /**< SDI Pin Bus Group Object */
    uint_t *gpio_group; /**< Pointer to group of gpio pins */
    sdi_gpio_file_fd_tbl_t **gpio_file_tbl; /**< Pointer to SDI GPIO
        File hdl table Pointers of each GPIO in group */
    uint_t gpio_count; /**< Number of gpio pins in the group */
} sdi_gpio_group_t;

#endif /* __SDI_GPIO_H___ */
