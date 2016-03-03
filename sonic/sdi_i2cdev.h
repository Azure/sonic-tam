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
 * filename: sdi_i2cdev.h
 */


/******************************************************************************
 * Defines Data Structures for SDI I2C Interface driver for KERNEL
 * driven I2C Bus
 *****************************************************************************/

#ifndef __SDI_I2CDEV_H___
#define __SDI_I2CDEV_H___
#include "sdi_i2c.h"
#include <linux/limits.h>

/**
 * Defines Data Structures to access I2C bus exported by kernel i2c
 * driver.
 *
 * Linux kernel i2c drivers exports
 * - sysfs i2c bus interface
 * to query i2c controller name, to bind i2c slaves attached to i2c bus etc.
 * - devfs i2c bus interface
 * to allow slaves read slave registers by sending ioctl to /dev/i2c-<bus>
 *
 * sdi_sys_i2c creates i2c bus handle that
 * - encapsulates sdi_i2c_bus_t to identify i2c bus and i2c bus operations
 * - i2c bus devfs and sysyfs name
 * - lock object to syncrhonize access to i2c bus
 * - capabilities exported by i2c bus etc.
 *
 * For details on accessing kernel i2c bus from user-space, please refer to
 * Linux kernel documentation Documentation/i2c/dev-interface
 *
 */

/**
 * Attribute used for representing the sysfs name
 */
#define SDI_DEV_ATTR_SYSFS_NAME        "sysfs_name"

/**
 * SDI I2C BUS Object for Kernel driver I2C Bus
 */
typedef struct sdi_sys_i2c_bus_ {
    sdi_i2c_bus_t bus; /* Common SDI I2C Bus Object used as bus handle */
    std_mutex_type_t lock; /* Mutex for bus lock  */
    char kernel_i2cdev_name[PATH_MAX]; /* Kernel /dev/i2cN interface backed by
        this I2C Bus */
    int i2cdev_fd; /* File descriptor opened on Kernel /dev/i2cN device file */
    char kernel_sysfs_name[PATH_MAX]; /* Sysfs name for this kernel driven I2C
        Bus */
    sdi_i2c_bus_capability_t capability; /* Funcionality supported by the i2c
        bus. Data type is unsigned long as expected by ioctl call */
} sdi_sys_i2c_bus_t;

#endif /* __SDI_I2CDEV_H___ */
