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
 * filename: sdi_sysfs_gpio_helpers.c
 */


/******************************************************************************
 * @file sdi_sysfs_gpio_helpers.c
 * @brief implements wrappers for exporting gpio, read/write gpio sysfs files
 *****************************************************************************/

#include "sdi_sysfs_gpio_helpers.h"
#include "sdi_device_common.h"
#include "sdi_pin_bus_attr.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <linux/limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define GPIO_EXPORT_PATH                "%s/class/gpio/export"
#define GPIO_PATH                       "%s/class/gpio/gpio%u"
#define GPIO_DIRECTION_FILE_PATH        GPIO_PATH"/direction"
#define GPIO_POLARITY_FILE_PATH         GPIO_PATH"/active_low"
#define GPIO_LEVEL_FILE_PATH            GPIO_PATH"/value"
/**
 * Invalid file descriptor
 */
#define SDI_INVALID_FILE_FD             -1

/**
 * sdi_gpio_is_exported
 * Verify whether the given gpio pin is already exported by checking for
 * existence of /sys/class/gpio/gpio<num> directory
 * param[in] gpio_no - gpio number
 * return true if gpio is exported, false if not exported
 */
bool sdi_gpio_is_exported(uint_t gpio_no)
{
    char gpio_dir_path[PATH_MAX] = {0};
    struct stat file_stat = {0};

    snprintf(gpio_dir_path, PATH_MAX, GPIO_PATH, SYSFS_PATH,
        gpio_no);
    if (stat(gpio_dir_path, &file_stat)) {
        return false;
    } else {
        return true;
    }
}


/**
 * sdi_sysfs_gpio_export
 * Export a gpio pin by writing gpio pin number to
 * /sys/class/gpio/export file
 * param[in] gpio_no - gpio number
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_export(uint_t gpio_no)
{
    char file[PATH_MAX] = {0};
    char gpio[PATH_MAX] = {0};
    t_std_error err = STD_ERR_OK;
    int fd = SDI_INVALID_FILE_FD,write_len = 0;

    snprintf(file, PATH_MAX, GPIO_EXPORT_PATH, SYSFS_PATH);
    snprintf(gpio, PATH_MAX, "%u\n", gpio_no);
    write_len = strlen(gpio);

    fd = open(file, O_WRONLY);
    if (fd < 0) {
        SDI_DEVICE_ERRMSG_LOG("file %s doesn't exist\n",
            file);
        return SDI_DEVICE_ERRNO;
    }
    err = write(fd, gpio, write_len);
    if (err < write_len) {
        err = write(fd, gpio, write_len);
        if (err < write_len) {
            err = SDI_DEVICE_ERRNO;
            SDI_DEVICE_ERRMSG_LOG("file write failed with %d\n",err);
        } else {
            err = STD_ERR_OK;
        }
    } else {
        err = STD_ERR_OK;
    }
    close(fd);
    return err;
}

/**
 * sdi_export_gpio
 * Export a gpio pin if not already exported.
 * param[in] gpio_no - gpio number
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_export_gpio(uint_t gpio_no)
{
    t_std_error err = STD_ERR_OK;

    /** Checks whether the gpio pin already got populated **/
    if (sdi_gpio_is_exported(gpio_no)) {
        SDI_DEVICE_ERRMSG_LOG("GPIO %u was already initialized\n", gpio_no);
    } else {
       /** Else expose gpio pin to the user space **/
        err = sdi_sysfs_gpio_export(gpio_no);
        if (err != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("GPIO %u can't be exported, err : %d\n",
                gpio_no, err);
            return err;
        }
        if (!sdi_gpio_is_exported(gpio_no)) {
            /** If gpio pin doesn't get created after exporting,its an error **/
            SDI_DEVICE_ERRMSG_LOG("GPIO %u is not be exported\n", gpio_no);
            return SDI_DEVICE_ERRNO;
        }
    }
    return err;
}

/**
 * sdi_gpio_file_table_init
 * Initialize file table with file pointers for gpio sysfs direction, polarity
 * value file
 * param[in] gpio_num - gpio number
 * param[in] gpio_file_tbl - pointer to gpio file table
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_gpio_file_table_init(uint_t gpio_num,
        sdi_gpio_file_fd_tbl_t *gpio_file_tbl)
{
    char file[PATH_MAX] = {0};
    t_std_error err = STD_ERR_OK;

    do {
        snprintf(file, PATH_MAX, GPIO_LEVEL_FILE_PATH, SYSFS_PATH, gpio_num);
        gpio_file_tbl->level_fd = open(file, O_RDWR);
        if (gpio_file_tbl->level_fd < 0) {
            err = SDI_DEVICE_ERRNO;
            SDI_DEVICE_ERRMSG_LOG("file open failed with %d\n", err);
            break;
        }

        snprintf(file, PATH_MAX, GPIO_DIRECTION_FILE_PATH, SYSFS_PATH, gpio_num);
        gpio_file_tbl->direction_fd = open(file, O_RDWR);
        if (gpio_file_tbl->direction_fd < 0) {
            err = SDI_DEVICE_ERRNO;
            SDI_DEVICE_ERRMSG_LOG("file open failed with %d\n", err);
            break;
        }

        snprintf(file, PATH_MAX, GPIO_POLARITY_FILE_PATH, SYSFS_PATH, gpio_num);
        gpio_file_tbl->polarity_fd = open(file, O_RDWR);
        if (gpio_file_tbl->polarity_fd < 0) {
            err = SDI_DEVICE_ERRNO;
            SDI_DEVICE_ERRMSG_LOG("file open failed with %d\n", err);
            break;
        }
    } while(0);

    if (err != STD_ERR_OK) {
        if (gpio_file_tbl->level_fd >= 0) {
            close(gpio_file_tbl->level_fd);
            gpio_file_tbl->level_fd = SDI_INVALID_FILE_FD;
        }
        if (gpio_file_tbl->direction_fd >= 0) {
            close(gpio_file_tbl->direction_fd);
            gpio_file_tbl->direction_fd = SDI_INVALID_FILE_FD;
        }
        if (gpio_file_tbl->polarity_fd >= 0) {
            close(gpio_file_tbl->polarity_fd);
            gpio_file_tbl->polarity_fd = SDI_INVALID_FILE_FD;
        }
    }
    return err;
}


/**
 * sdi_sysfs_gpio_value_read
 * Read the value from sysfs gpio file (could be gpio's value, active_low)
 * param[in] gpio_file_fd - sysfs gpio file descriptor
 * ex: /sys/class/gpio/gpio0/value or /sys/class/gpio/gpio0/active_low
 * param[out] value - value read from given file path
 * return STD_ERR_OK on successful read, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_value_read(int gpio_file_fd, bool *value)
{
    uint8_t ch = 0;

    lseek(gpio_file_fd,0, SEEK_SET);
    read(gpio_file_fd, &ch, 1);
    if ((ch - '0') != 0) {
         *value = true;
    } else {
         *value = false;
    }
    return STD_ERR_OK;
}


/**
 * sdi_sysfs_gpio_value_write
 * Write a value to sysfs gpio file (could be gpio's value, active_low)
 * param[in] gpio_file_fd - sysfs gpio file descriptor
 * ex: /sys/class/gpio/gpio0/value or /sys/class/gpio/gpio0/active_low
 * param[in] value - value to be written to gpio sysfs file path
 * return STD_ERR_OK on successful write, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_value_write(int gpio_file_fd, bool value)
{
    char buf[MAX_VALUE_STR_LEN] = {0};
    t_std_error error = STD_ERR_OK;

    snprintf(buf, MAX_VALUE_STR_LEN, "%d\n", value);
    lseek(gpio_file_fd, 0, SEEK_SET);
    error = write(gpio_file_fd, buf, 1);
    if(error < 1){
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("file write failed with %d\n", error);
    } else {
        error = STD_ERR_OK;
    }
    return error;
}


/**
 * sdi_sysfs_gpio_direction_read
 * Read direction of gpio from sysfs gpio absolute file path
 * param[in] gpio_file_fd - sysfs gpio file descriptor
 * param[out] direction - SDI_PIN_BUS_INPUT on input pin, SDI_PIN_BUS_OUTPUT on
 * output pin
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_direction_read(int gpio_file_fd,
    sdi_pin_bus_direction_t *direction)
{
    char buf[MAX_DIRECTION_LEN] = {0};

    lseek(gpio_file_fd,0, SEEK_SET);
    read(gpio_file_fd, buf, MAX_DIRECTION_LEN);

    if (!strncmp(buf, SDI_DEV_ATTR_INPUT_PIN, SDI_DEV_ATTR_INPUT_PIN_LEN)) {
        *direction = SDI_PIN_BUS_INPUT;
    } else if (!strncmp(buf, SDI_DEV_ATTR_OUTPUT_PIN,
            SDI_DEV_ATTR_OUTPUT_PIN_LEN)) {
        *direction = SDI_PIN_BUS_OUTPUT;
    } else {
        return SDI_DEVICE_ERRCODE(EINVAL);
    }
    return STD_ERR_OK;
}

/**
 * sdi_sysfs_gpio_direction_write
 * Write direction of gpio to sysfs gpio absolute file path
 * param[in] gpio_file_fd - sysfs gpio file descriptor
 * param[in] direction - SDI_PIN_BUS_INPUT on input pin, SDI_PIN_BUS_OUTPUT on
 * output pin
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
t_std_error sdi_sysfs_gpio_direction_write(int gpio_file_fd,
    sdi_pin_bus_direction_t direction)
{
    char buf[MAX_DIRECTION_LEN] = {0};
    t_std_error error = STD_ERR_OK;

    if (direction == SDI_PIN_BUS_INPUT) {
        snprintf(buf, MAX_DIRECTION_LEN, "%s\n", SDI_DEV_ATTR_INPUT_PIN);
    } else if (direction == SDI_PIN_BUS_OUTPUT) {
        snprintf(buf, MAX_DIRECTION_LEN, "%s\n", SDI_DEV_ATTR_OUTPUT_PIN);
    } else {
        return SDI_DEVICE_ERRCODE(EINVAL);
    }

    lseek(gpio_file_fd,0, SEEK_SET);
    error = write(gpio_file_fd, buf, MAX_DIRECTION_LEN);
    if(error < 0){
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("file write failed with %d\n", error);
    } else {
        error = STD_ERR_OK;
    }
    return error;
}
