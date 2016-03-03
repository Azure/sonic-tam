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
 * filename: sdi_sysfs_gpio_helpers.h
 */


/******************************************************************************
 * @file sdi_sysfs_gpio_helpers.h
 * @brief implements wrappers for exporting gpio, read/write gpio sysfs files
 *****************************************************************************/

#ifndef __SDI_SYSFS_GPIO_HELPERS_H__
#define __SDI_SYSFS_GPIO_HELPERS_H__

#include "sdi_gpio.h"

/**
 * @brief sdi_gpio_is_exported
 * Verify whether the given gpio pin is already exported by checking for
 * existence of /sys/class/gpio/gpio<num> directory
 * @param[in] gpio_no - gpio number
 * @return true if gpio is exported, false if not exported
 */
bool sdi_gpio_is_exported(uint_t gpio_no);

/**
 * @brief sdi_sysfs_gpio_export
 * Export a gpio pin by writing gpio pin number to
 * /sys/class/gpio/export file
 * @param[in] gpio_no - gpio number
 * @return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_export(uint_t gpio_no);

/**
 * @brief sdi_export_gpio
 * Export a gpio pin if not already exported.
 * @param[in] gpio_no - gpio number
 * @return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_export_gpio(uint_t gpio_no);

/**
 * @brief sdi_gpio_file_table_init
 * Initialize file table with file pointers for gpio sysfs direction, polarity
 * value file
 * @param[in] gpio_num - gpio number
 * @param[in] gpio_file_tbl - pointer to gpio file table
 * @return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_gpio_file_table_init(uint_t gpio_num,
                                    sdi_gpio_file_fd_tbl_t *gpio_file_tbl);

/**
 * @brief sdi_sysfs_gpio_value_read
 * Read the value from sysfs gpio file (could be gpio's value, active_low)
 * @param[in] gpio_file_fd - sysfs gpio file descriptor
 * ex: /sys/class/gpio/gpio0/value or /sys/class/gpio/gpio0/active_low
 * @param[out] value - value read from given file path
 * @return STD_ERR_OK on successful read, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_value_read(int gpio_file_fd, bool *value);

/**
 * @brief sdi_sysfs_gpio_value_write
 * Write a value to sysfs gpio file (could be gpio's value, active_low)
 * @param[in] gpio_file_fd - sysfs gpio file descriptor
 * ex: /sys/class/gpio/gpio0/value or /sys/class/gpio/gpio0/active_low
 * @param[in] value - value to be written to gpio sysfs file path
 * @return STD_ERR_OK on successful write, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_value_write(int gpio_file_fd, bool value);

/**
 * @brief sdi_sysfs_gpio_direction_read
 * Read direction of gpio from sysfs gpio absolute file path
 * @param[in] gpio_file_fd - sysfs gpio file descriptor
 * @param[out] direction - SDI_PIN_BUS_INPUT on input pin, SDI_PIN_BUS_OUTPUT on
 * output pin
 * @return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_direction_read(int gpio_file_fd,
                                         sdi_pin_bus_direction_t *direction);

/**
 * @brief sdi_sysfs_gpio_direction_write
 * Write direction of gpio to sysfs gpio absolute file path
 * @param[in] gpio_file_fd - sysfs gpio file descriptor
 * @param[in] direction - SDI_PIN_BUS_INPUT on input pin, SDI_PIN_BUS_OUTPUT on
 * output pin
 * @return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_direction_write(int gpio_file_fd,
                                          sdi_pin_bus_direction_t direction);

#endif /* __SDI_SYSFS_GPIO_HELPERS_H__ */
