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
 * filename: sdi_i2cdev.c
 */


/******************************************************************************
 * Implements SDI I2C Bus Operations for Kernel driven I2C BUS and
 * Registers i2c bus with SDI I2C Bus Framework and registers devices attached
 * to the i2c bus
 * @todo:
 * Additionally 16bit offset read and write supported, which used smbus protocol.
 * However, need to implement it with i2c protocol.
 * To unblock S3000 activities, these changes are pushed.
 *****************************************************************************/

#include "std_assert.h"
#include "sdi_device_common.h"
#include "std_error_codes.h"
#include "std_config_node.h"
#include "sdi_driver_internal.h"
#include "sdi_i2cdev.h"
#include "sdi_common_attr.h"
#include "sdi_bus_attr.h"
#include "std_utils.h"
#include "std_time_tools.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define SDI_MAX_IIC_RETRY 5
#define SDI_IIC_WAIT_TIME 1000 /*1ms*/

/**
 * format for i2c node representation in configuration:
 * <sys_i2c instance="0" sysfs_name="SMBus SCH adapter at 0400"
 *            bus_name="smbus0">
 * <!-- sysfs_name is name of the kernel driven i2c bus as in sysfs i2c name file
 *        bus_name is the name of the i2c bus used during look-up operations
 * -->
 *    <i2c_slave_node(s) attributes=..></i2c_slave_node(s)>
 *    <!-- one ore more i2c slaves like tmp75 sensor, eeprom device -->
 * </sys_i2c>
 *
 * bus_name is optional. If bus_name is not specified, it is constructed by
 * appending instance to node name.
 *
 * @todo pending
 * - generate i2c bus instance internally instead of fetching from config file
 * - add support for unsupported SMBUS operations when need arises
 * - add support for I2C Operations when need arises
 *
 */

/**
 * Current directory, used during look-up of i2c sysfs name
 */
#define CURRENT_DIRECTORY                "."

/**
 * String Length of CURRENT_DIRECTORY
 */
#define CURRENT_DIRECTORY_NAME_LEN        (1)

/**
 * Previous/Parent directory, used during look-up of i2c sysfs name
 */
#define PARENT_DIRECTORY                ".."

/**
 * String Length of PARENT_DIRECTORY
 */
#define PARENT_DIRECTORY_NAME_LEN        (2)

/**
Invalid file descriptor
*/
#define INVALID_FILE_FD                    -1

/**
SMBUS receive byte command offset
*/
#define SDI_SMBUS_RECV_BYTE_CMD_OFFSET    0

/**
 * SMBUS 16 bit execute command default offset
 */
#define SDI_SMBUS_16BIT_CMD_DEF_OFFSET    0

static inline t_std_error sdi_smbus_write_byte(int i2cdev_fd,
                                               sdi_smbus_operation_t operation,
                                               sdi_smbus_data_type_t data_type,
                                               uint_t commandbuf,
                                               void *buffer);

/**
 * sdi_get_i2c_device_path_by_sysfs_name
 * When I2C Controller is controlled by kernel driver, kernel exports
 * /dev/i2c-N device interface to facilitate user-space applications to
 * access I2C Controller.
 * In case of more than one I2C Controllers driven by kernel, order of
 * enumeration of I2C Controllers can change from one boot to the next.
 * For ex: When system boots, X I2C Controller could get enumerated as
 * /dev/i2c0 on reboot of the system, the same X I2C Controller can get
 * enumerated as /dev/i2c1. To facilitate user to find the i2c dev path
 * for every i2c controller, kernel exports I2C Controller's name in
 * sysfs path for every enumerated I2C bus.
 * This routine finds the i2c device path based on the I2C Controller name.
 *
 * param[in] bus_name I2C Controller's bus name for which i2c dev path
 *    needs to be found
 * param[out] i2c_dev_path I2c dev path for i2c controller, allocated for
 *    PATH_MAX length by caller. ex: "/dev/i2c-1"
 * ex:
 * root:/sys/bus/i2c/devices/i2c-1# cat name
 *  SMBus iSMT adapter at ff782000
 * bus name : SMBus iSMT adapter at ff782000
 * i2c_dev_path : /dev/i2c-1
 * return STD_ERR_OK on success, SDI_DEVICE_ERR_PARAM when given bus_name
 * doesn't exist, SDI_DEVICE_ERRNO for any other error.
 */
static t_std_error sdi_get_i2c_device_path_by_sysfs_name(const char *bus_name,
        char *i2c_dev_path)
{
    DIR *dir = NULL;
    FILE *fp = NULL;
    struct dirent *ent = NULL;
    char dev_name[NAME_MAX] = {0};
    char *sptr = NULL;
    char *str = NULL;
    char i2cdev[PATH_MAX] = {0};
    char name[PATH_MAX] = {0};
    t_std_error rc = SDI_DEVICE_ERR_PARAM;

    snprintf(i2cdev, PATH_MAX, "%s/class/i2c-dev", SYSFS_PATH);
    if ((dir = opendir(i2cdev)) == NULL) {
        rc = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d opendir %s failed with err %d\n",
            __FUNCTION__, __LINE__, i2cdev, rc);
        return rc;
    }

    /** /sys/class/i2cdev has a dir for each i2c node created by kernel.
        Each i2c node directory has bus name under device/name.
        Find the match for our bus and that is the node/devid of our interest */
    while ((ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, CURRENT_DIRECTORY, CURRENT_DIRECTORY_NAME_LEN) == 0) {
            continue;
        }
        if (strncmp(ent->d_name, PARENT_DIRECTORY, PARENT_DIRECTORY_NAME_LEN) == 0) {
            continue;
        }

        snprintf(name, PATH_MAX, "%s/%s/device/name", i2cdev, ent->d_name);
        if ((fp = fopen(name, "r")) == NULL) {
            rc = SDI_DEVICE_ERRNO;
            SDI_DEVICE_ERRNO_LOG();
            break;
        }
        sptr = fgets(dev_name, NAME_MAX, fp);
        dev_name[NAME_MAX-1] = 0;
        fclose(fp);

        if (sptr == NULL) {
            SDI_DEVICE_ERRNO_LOG();
            continue;
        }
        if ((str = strchr(dev_name, (int)'\n')) != NULL) {
            *str = 0;
        }

        if (strncmp(dev_name, (char *)bus_name, NAME_MAX) == 0) {
            snprintf(i2c_dev_path, PATH_MAX, "/dev/%s", ent->d_name);
            rc = STD_ERR_OK;
            break;
        }
    }
    closedir(dir);
    return rc;
}

/**
 * sdi_i2cdev_acquire_bus
 * Lock the i2c bus before executing a transaction.
 * When the lock is already acquired, this routine will sleep wait until
 * the lock is released.
 * param[in] i2c_bus sdi i2c bus handle
 * return STD_ERR_OK on success, non-zero error code on mutex lock failure
 */
static t_std_error sdi_i2cdev_acquire_bus (sdi_i2c_bus_hdl_t i2c_bus)
{
    t_std_error error = STD_ERR_OK;

    sdi_sys_i2c_bus_t * bus = (sdi_sys_i2c_bus_t *) i2c_bus;

    error = std_mutex_lock (&(bus->lock));
    if (error != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d acquire lock failed\n",
            __FUNCTION__, __LINE__, i2c_bus->bus.bus_id);
    }

    return error;
}

/**
 * sdi_sys_smbus_execute
 * Execute the I2C SMBUS transaction by issuing an ioctl to kernel smbus driver
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[in] commandbuf - Address offset of SMBUS Transaction
 * param[out] data - Holds SMBUS Transaction data read/to write.
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static t_std_error sdi_sys_smbus_execute(int i2cdev_fd,
    sdi_smbus_operation_t operation, sdi_smbus_data_type_t data_type,
    uint_t commandbuf, union i2c_smbus_data *data)
{
    t_std_error error = STD_ERR_OK;
    uint_t retry_count = SDI_MAX_IIC_RETRY;
    struct i2c_smbus_ioctl_data cmd;
    cmd.read_write = operation;
    cmd.command = (uint8_t) commandbuf;
    cmd.size = data_type;
    cmd.data = data;

    do {
        error = ioctl(i2cdev_fd, I2C_SMBUS, &cmd);
        if (error != STD_ERR_OK) {
            retry_count--;
            std_usleep(SDI_IIC_WAIT_TIME);
        }
    } while( (error != STD_ERR_OK) && (retry_count != 0) );

    std_usleep(SDI_IIC_WAIT_TIME);

    if (error != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d smbus transaction on i2cdev_fd %d,"
                "operation %d command %d size %d data %p failed %d\n",
                __FUNCTION__, __LINE__, i2cdev_fd, operation, commandbuf, data_type,
                data, error);
    } else if(retry_count != SDI_MAX_IIC_RETRY) {
        SDI_DEVICE_ERRMSG_LOG("%s:%d smbus transaction on i2cdev_fd %d, operation %d command %d size %d data %p is succeeded after %u retries\n",
                              __FUNCTION__, __LINE__, i2cdev_fd, operation, commandbuf, data_type, data, (SDI_MAX_IIC_RETRY - retry_count));
    }

    return error;
}

/**
 * sdi_smbus_recv_byte
 * Read a byte using I2C from I2C Bus File descriptor opened on i2cdev_fd
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[out] buffer - Store the result of I2C SMBUS Byte Read Operation
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_smbus_recv_byte(int i2cdev_fd,
    sdi_smbus_operation_t operation, sdi_smbus_data_type_t data_type,
    void *buffer)

{
    t_std_error error = STD_ERR_OK;
    union i2c_smbus_data data = { .byte = 0 };

    error = sdi_sys_smbus_execute(i2cdev_fd, operation, data_type,
                SDI_SMBUS_RECV_BYTE_CMD_OFFSET, &data);
    if (error == STD_ERR_OK) {
        *(uint8_t *)buffer = SDI_MAX_BYTE_VAL & data.byte;
    }
    return error;
}

/**
 * sdi_smbus_send_byte
 * Write a byte using I2C from I2C Bus File descriptor opened on i2cdev_fd
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[in] commandbuf - Address offset for SMBUS Transaction
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_smbus_send_byte(int i2cdev_fd,
    sdi_smbus_operation_t operation, sdi_smbus_data_type_t data_type,
    uint_t commandbuf)
{
    return sdi_sys_smbus_execute(i2cdev_fd, operation, data_type,
        commandbuf, NULL);
}


/**
 * sdi_smbus_read_byte
 * Read a byte from offset specified by commandbuf using I2C from I2C Bus File
 * descriptor opened on i2cdev_fd
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[in] commandbuf - Address offset for SMBUS Transaction
 * param[out] buffer - Store the result of I2C Read in Buffer
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_smbus_read_byte(int i2cdev_fd,
    sdi_smbus_operation_t operation, sdi_smbus_data_type_t data_type,
    uint_t commandbuf, void *buffer)

{
    union i2c_smbus_data data = { .byte = 0 };
    t_std_error error = STD_ERR_OK;

    error = sdi_sys_smbus_execute(i2cdev_fd, operation, data_type,
        commandbuf, &data);
    if (error == STD_ERR_OK) {
        *(uint8_t *)buffer = SDI_MAX_BYTE_VAL & data.byte;
    }
    return error;
}

/**
 * sdi_smbus_write_byte
 * Write a byte at offset specified by commandbuf using I2C from I2C Bus File
 * descriptor opened on i2cdev_fd
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[in] commandbuf - Address offset for SMBUS Transaction
 * param[out] buffer - Write the byte in Buffer to I2C Bus
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_smbus_write_byte(int i2cdev_fd,
    sdi_smbus_operation_t operation, sdi_smbus_data_type_t data_type,
    uint_t commandbuf, void *buffer)
{
    union i2c_smbus_data data = { .byte = (*(uint8_t *)buffer) };

    return sdi_sys_smbus_execute(i2cdev_fd, operation, data_type,
            commandbuf, &data);
}

/**
 * sdi_i2c_read
 * Read a byte from offset specified by commandbuf using I2C from I2C Bus File
 * descriptor opened on i2cdev_fd
 * Note: This api is used, when cmdlen is 2 or 16bit offset. for other cases,
 * use smbus apis.
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] address   - I2C slave Address
 * param[in] cmd : list of read offsets
 * param[in] cmdlen : no. of offsets
 * param[out] buf : read from slave via i2c
 * param[in] buflen : no.of bytes to read
 * param[in] flag : options if any to be sent @sa sdi_i2c_flags for
 * supported flags
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_i2c_read(int i2cdev_fd, sdi_i2c_addr_t address,
                                            const uint8_t *cmd, uint_t cmdlen,
                                            void *buf, uint_t buflen, uint_t flag)
{
    union i2c_smbus_data data = { .byte = 0 };
    t_std_error error = STD_ERR_OK;

    if (cmdlen == 2) {
        uint8_t buffer = *cmd;

        error = sdi_smbus_write_byte(i2cdev_fd, SDI_SMBUS_WRITE, I2C_SMBUS_BYTE_DATA,
                                    *(cmd++), &buffer);
        if (error != STD_ERR_OK) {
            return error;
        }
        error = sdi_sys_smbus_execute(i2cdev_fd, SDI_SMBUS_READ, I2C_SMBUS_BYTE,
                                      SDI_SMBUS_16BIT_CMD_DEF_OFFSET, &data);
    } else {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }
    if (error == STD_ERR_OK) {
        *(uint8_t *)buf = data.byte;
    }
    return error;
}

/**
 * sdi_i2c_write
 * Write a byte at offset specified by offset using I2C from I2C Bus File
 * descriptor opened on i2cdev_fd
 * Note: This api is used, when cmdlen is 2 or 16bit offset. for other cases,
 * use smbus apis.
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] address   - I2C device Address
 * param[in] cmd : list of write offsets
 * param[in] cmdlen : no. of offsets
 * param[in] buf : data to be written to slave via i2c
 * param[in] buflen : no.of bytes to write
 * param[in] flag : options if any to be sent @sa sdi_i2c_flags for
 * supported flags
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_i2c_write(int i2cdev_fd, sdi_i2c_addr_t address,
                                            const uint8_t *cmd, uint_t cmdlen,
                                            void *buf, uint_t buflen, uint_t flag)
{
    union i2c_smbus_data data;

    if (cmdlen == 2) {
        data.word = (*cmd | ((*(uint8_t *)buf) << BITS_PER_BYTE));
        return sdi_sys_smbus_execute(i2cdev_fd, SDI_SMBUS_WRITE, I2C_SMBUS_WORD_DATA,
                                     *(cmd++), &data);
    } else {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }
}

/**
 * sdi_smbus_read_word
 * Read a word from offset specified by commandbuf using I2C from I2C Bus File
 * descriptor opened on i2cdev_fd
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[in] commandbuf - Address offset for SMBUS Transaction
 * param[out] buffer - Store the result of I2C Read in Buffer
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_smbus_read_word(int i2cdev_fd,
    sdi_smbus_operation_t operation, sdi_smbus_data_type_t data_type,
    uint_t commandbuf, void *buffer)

{
    t_std_error error = STD_ERR_OK;
    union i2c_smbus_data data = { .word = 0 };

    error = sdi_sys_smbus_execute(i2cdev_fd, operation, data_type,
            commandbuf, &data);
    if (error == STD_ERR_OK) {
        *(uint16_t *)buffer = SDI_MAX_WORD_VAL & data.word;
    }
    return error;
}

/**
 * sdi_smbus_write_word
 * Write a word at offset specified by commandbuf using I2C from I2C Bus File
 * descriptor opened on i2cdev_fd
 * param[in] i2cdev_fd - opened file descriptor for i2c bus
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[in] commandbuf - Address offset for SMBUS Transaction
 * param[out] buffer - Write the byte in Buffer to I2C Bus
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure
 */
static inline t_std_error sdi_smbus_write_word(int i2cdev_fd,
    sdi_smbus_operation_t operation, sdi_smbus_data_type_t data_type,
    uint_t commandbuf, void *buffer)
{
    union i2c_smbus_data data = { .word = (*(uint16_t *)buffer) };

    return sdi_sys_smbus_execute(i2cdev_fd, operation, data_type,
            commandbuf, &data);
}

/**
 * sdi_i2cdev_i2c_execute
 * Execute I2C transaction
 * param[in] i2c_bus - I2C Bus handle
 * param[in] address - I2C Slave Address
 * param[in] operation - I2C Read/Write Operation
 * @param[in] cmd : list of read/write offsets
 * @param[in] cmdlen : no. of offsets
 * @param[out] buffer - data read from/written to slave
 * @param[in] buflen : no.of bytes to read/write
 * param[in] flags - options if any to be send to i2c execute
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure,
 * SDI_DEVICE_ERRCODE(ENOTSUP) on unsupported operation
 */
static t_std_error sdi_i2cdev_i2c_execute(sdi_i2c_bus_hdl_t i2c_bus,
                   sdi_i2c_addr_t address, sdi_i2c_operation_t operation,
                   const uint8_t *cmd, uint_t cmdlen, void *buffer, uint_t buflen,
                   uint_t flags)
{
    sdi_sys_i2c_bus_t * bus = (sdi_sys_i2c_bus_t *) i2c_bus;
    int i2cdev_fd = bus->i2cdev_fd;
    t_std_error error = STD_ERR_OK;

    if (ioctl(i2cdev_fd, I2C_SLAVE, address) != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d set slave %2x failed %d",
            __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, address, error);
        return error;
    }

    switch (operation) {
        case SDI_I2C_WRITE:
             error = sdi_i2c_write(i2cdev_fd, address, cmd, cmdlen, buffer,
buflen, flags);
             break;
        case SDI_I2C_READ:
             error = sdi_i2c_read(i2cdev_fd, address, cmd, cmdlen, buffer,
buflen, flags);
             break;
        default:
             error = SDI_DEVICE_ERRCODE(ENOTSUP);
             SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d unsupported operation",
                                   __FUNCTION__, __LINE__, i2c_bus->bus.bus_id);
             break;
    }

    if (error != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("i2c bus %d operation:%d failed %d",
                               i2c_bus->bus.bus_id, operation, error);
    }

    return error;
}


/**
 * sdi_i2cdev_smbus_execute
 * Execute I2C transaction
 * param[in] i2c_bus - I2C Bus handle
 * param[in] address - I2C Slave Address
 * param[in] operation - SMBUS Read/Write Operation
 * param[in] data_type - SMBUS Transaction size
 * param[in] commandbuf - Address offset for SMBUS Transaction
 * param[out] block_len - Length of block data read from/written to I2C Slave
 * only for SDI_SMBUS_BLOCK_DATA SMBUS Transaction. Though BLOCK transfer is not
 * supported now, block_len is added in order to avoid changing the signature of
 * this function when its supported later. For Read BLOCK operation, number of
 * bytes actually read is filled in block_len on successful return of this
 * function. For Write BLOCK operation, number of bytes to be written as part of
 * BLOCK operation is specified.
 * in : Number of bytes to read as input; On return,
 * out: store the number of bytes read from I2C Bus
 * param[out] buffer - Data Read From/Written to I2C Bus
 * param[in] flags - Supported flags: PEC
 * return STD_ERR_OK on Success, SDI_DEVICE_ERRNO on Failure,
 * SDI_DEVICE_ERRCODE(ENOTSUP) on unsupported operation
 */
static t_std_error sdi_i2cdev_smbus_execute(sdi_i2c_bus_hdl_t i2c_bus,
    sdi_i2c_addr_t address,    sdi_smbus_operation_t operation,
    sdi_smbus_data_type_t data_type, uint_t commandbuf,
    void *buffer, size_t *block_len,
    uint_t flags)
{
    sdi_sys_i2c_bus_t * bus = (sdi_sys_i2c_bus_t *) i2c_bus;
    int i2cdev_fd = bus->i2cdev_fd;
    t_std_error error = STD_ERR_OK;

    if (ioctl(i2cdev_fd, I2C_SLAVE, address) != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d set slave %2x failed %d\n",
            __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, address, error);
        return error;
    }

    /*Enabling the SMBUS PEC */
    if(flags == SDI_I2C_FLAG_PEC)
    {
        if (ioctl(i2cdev_fd, I2C_PEC, 1) != STD_ERR_OK) {
            error = SDI_DEVICE_ERRNO;
            SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d set PEC failed %d\n",
                __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, error);
            return error;
        }
    }

    switch (data_type) {
        case SDI_SMBUS_BYTE:
            if (operation == SDI_SMBUS_WRITE) {
                error = sdi_smbus_send_byte(i2cdev_fd, operation,
                    I2C_SMBUS_BYTE, commandbuf);
            } else {
                error = sdi_smbus_recv_byte(i2cdev_fd, operation,
                    I2C_SMBUS_BYTE, buffer);
            }
            break;
        case SDI_SMBUS_BYTE_DATA:
            if (operation == SDI_SMBUS_WRITE) {
                error = sdi_smbus_write_byte(i2cdev_fd,
                    operation, I2C_SMBUS_BYTE_DATA, commandbuf, buffer);
            } else {
                error = sdi_smbus_read_byte(i2cdev_fd,
                    operation, I2C_SMBUS_BYTE_DATA, commandbuf, buffer);
            }
            break;
        case SDI_SMBUS_WORD_DATA:
            if (operation == SDI_SMBUS_WRITE) {
                error = sdi_smbus_write_word(i2cdev_fd,
                    operation, I2C_SMBUS_WORD_DATA, commandbuf, buffer);
            } else {
                error = sdi_smbus_read_word(i2cdev_fd,
                    operation, I2C_SMBUS_WORD_DATA, commandbuf, buffer);
            }
            break;
        default:
            error = SDI_DEVICE_ERRCODE(ENOTSUP);
            SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d unsupported data type %d\n",
                __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, data_type);
            break;
    }

    if (error != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("i2c bus %d operation:%d datatype:%d failed %d\n",
                               i2c_bus->bus.bus_id, operation, data_type, error);
    }

    /*Disabling the PEC after the operation is completed*/
    if(flags == SDI_I2C_FLAG_PEC)
    {
        if (ioctl(i2cdev_fd, I2C_PEC, 0) != STD_ERR_OK) {
            error = SDI_DEVICE_ERRNO;
            SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d set PEC failed %d\n",
                __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, error);
        }
    }

    return error;
}

/**
 * sdi_i2cdev_release_bus
 * UnLock the i2c bus after executing a transaction.
 * param[in] i2c_bus i2c bus handle
 * return none
 */
static void sdi_i2cdev_release_bus (sdi_i2c_bus_hdl_t i2c_bus)
{
    sdi_sys_i2c_bus_t * bus = (sdi_sys_i2c_bus_t *) i2c_bus;

    std_mutex_unlock (&(bus->lock));
}

/**
 * sdi_sys_i2c_get_capability
 * Get the capability of I2C BUS
 * param[in] i2c_bus i2c bus handle
 * param[out] capability filled by this function as suppported by this i2c bus
 * return none
 */
void sdi_sys_i2c_get_capability (sdi_i2c_bus_hdl_t i2c_bus,
    sdi_i2c_bus_capability_t *capability)
{
    sdi_sys_i2c_bus_t *bus = (sdi_sys_i2c_bus_t *) i2c_bus;
    *capability = bus->capability;
}

/**
 * sdi_i2cdev_bus_ops
 * SDI I2C Bus Operations for interfacing with kernel i2c driver
 */
static sdi_i2c_bus_ops_t sdi_i2cdev_bus_ops = {
    .sdi_i2c_acquire_bus = sdi_i2cdev_acquire_bus,
    .sdi_smbus_execute = sdi_i2cdev_smbus_execute,
    .sdi_i2c_execute = sdi_i2cdev_i2c_execute,
    .sdi_i2c_release_bus = sdi_i2cdev_release_bus,
    .sdi_i2c_get_capability = sdi_sys_i2c_get_capability,
};

/**
 * sdi_i2cdev_driver_init
 * SDI I2C kernel interface driver Initialization Function
 * As the kernel i2c driver initializes and controls the i2c bus,
 * and sdi_i2c driver just acts as an interface with that
 * kernel i2c driver, sdi_i2cdev_driver_init maps sysfs i2c bus name
 * with the devfs i2c bus and opens the devfs i2c bus for performing any
 * transactions on this i2c bus.
 * param[in] bus - (i2c) bus handle
 * return STD_ERR_OK on success,
 * SDI_DEVICE_ERR_PARAM on failure to find /dev/i2c for specified sysfs name,
 * SDI_DEVICE_ERRNO on other failures.
 */
static t_std_error sdi_i2cdev_driver_init(sdi_bus_hdl_t bus)
{
    t_std_error error = STD_ERR_OK;
    sdi_sys_i2c_bus_t  *sys_i2c_bus = (sdi_sys_i2c_bus_t *) bus;
    sdi_i2c_bus_hdl_t i2c_bus = NULL;

    STD_ASSERT(sys_i2c_bus != NULL);

    i2c_bus = &(sys_i2c_bus->bus);

    if(strlen(sys_i2c_bus->kernel_i2cdev_name) == 0)
    {
        /*Get the dev name from sysfs name if it is not passed via config*/
        error = sdi_get_i2c_device_path_by_sysfs_name(
                                                      sys_i2c_bus->kernel_sysfs_name,
                                                      sys_i2c_bus->kernel_i2cdev_name);
        if (error != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d get device path failed %d\n",
                                  __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, error);
            return error;
        }
    }
    sys_i2c_bus->i2cdev_fd = open(sys_i2c_bus->kernel_i2cdev_name,
            O_RDWR);
    if (sys_i2c_bus->i2cdev_fd < 0) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d open device path failed %d\n",
            __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, error);
        return error;
    }

    error = ioctl(sys_i2c_bus->i2cdev_fd, I2C_FUNCS,
            &sys_i2c_bus->capability);
    if (error != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %d get funcs failed %d\n",
            __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, error);
        return error;
    }

    sdi_bus_init_device_list(bus);

    return error;
}

/**
 * sdi_i2cdev_driver_register
 * SDI I2C Kernel Interface driver registration with bus framework
 * param[in] node - i2c bus node handle obtained by parsing configuration
 * param[out] bus - i2c bus handle populated after successful registration
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_i2cdev_driver_register(std_config_node_t node,
        sdi_bus_hdl_t *bus)
{

    char *node_attr = NULL;
    char *str = NULL;
    t_std_error error = STD_ERR_OK;
    sdi_i2c_bus_hdl_t i2c_bus = NULL;

    sdi_sys_i2c_bus_t *sys_i2c_bus = (sdi_sys_i2c_bus_t *)
        calloc (sizeof (sdi_sys_i2c_bus_t), 1);

    STD_ASSERT(sys_i2c_bus != NULL);

    i2c_bus = &(sys_i2c_bus->bus);
    i2c_bus->bus.bus_type = SDI_I2C_BUS;
    i2c_bus->bus.bus_init = sdi_i2cdev_driver_init;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);

    STD_ASSERT(node_attr != NULL);

    i2c_bus->bus.bus_id =  (uint_t) strtoul (node_attr, NULL, 0);

    str = std_config_attr_get(node, SDI_DEV_ATTR_BUS_NAME);
    if (str == NULL) {
            snprintf(i2c_bus->bus.bus_name, SDI_MAX_NAME_LEN, "%s-%d",
                std_config_name_get(node), i2c_bus->bus.bus_id);
    } else {
            safestrncpy(i2c_bus->bus.bus_name, str, SDI_MAX_NAME_LEN);
    }

    i2c_bus->ops = &sdi_i2cdev_bus_ops;

    error = std_mutex_lock_init_non_recursive(&(sys_i2c_bus->lock));
    if (error != STD_ERR_OK) {
        error = SDI_DEVICE_ERRNO;
        free(sys_i2c_bus);
        SDI_DEVICE_ERRMSG_LOG("%s:%d i2c bus %u lock init failed %d\n",
            __FUNCTION__, __LINE__, i2c_bus->bus.bus_id, error);
        return error;
    }

    str = std_config_attr_get(node, SDI_DEV_ATTR_SYSFS_NAME);
    STD_ASSERT(str!=NULL);
    safestrncpy(sys_i2c_bus->kernel_sysfs_name, str, PATH_MAX);

    str = std_config_attr_get(node, SDI_DEV_ATTR_DEV_NAME);
    if (str != NULL) {
        /*fill the dev name directly if it is passed*/
        safestrncpy(sys_i2c_bus->kernel_i2cdev_name, str, PATH_MAX);
    }

    sys_i2c_bus->i2cdev_fd = INVALID_FILE_FD;

    sdi_bus_register((sdi_bus_hdl_t) sys_i2c_bus);

    *bus = (sdi_bus_hdl_t) sys_i2c_bus;

    sdi_bus_register_device_list(node, (sdi_bus_hdl_t) sys_i2c_bus);

    return error;
}


/**
 * sys_i2c_entry_callbacks
 * SDI Bus Driver Object to hold this driver's registration and initialization
 * function
 *
 * Note:
 * Every bus driver must export function with name sdi_<bus_driver_name>_entry_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_bus_driver_t * sys_i2c_entry_callbacks(void)
{
     /*Export Bus Driver table*/
     static const sdi_bus_driver_t sys_i2c_entry = {
        .bus_register = sdi_i2cdev_driver_register,
        .bus_init = sdi_i2cdev_driver_init
     };
     return &sys_i2c_entry;
}

