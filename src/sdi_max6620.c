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
 * filename: sdi_max6620.c
 */


/******************************************************************************
  * sdi_max6620.c
  * Implements the driver for max6620 fan controller chip. The MAX6620 chip can control up to
  * four fans.It supports two methods for fan control viz RPM mode and DAC mode. This driver
  * supports only the RPM mode.The device provides the register interface for configuration and status
  * control of the fans.For more information about the MAX6620, refer the data sheet in below link
  * http://datasheets.maximintegrated.com/en/ds/MAX6620.pdf
  ******************************************************************************/

#include "sdi_max6620.h"
#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_fan_internal.h"
#include "sdi_common_attr.h"
#include "sdi_fan_resource_attr.h"
#include "sdi_i2c_bus_api.h"
#include "sdi_device_common.h"
#include "std_assert.h"
#include "std_utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define DRV_VOLT_TO_SPEED_PERCENT(duty) ((duty * 100)/0x1ff)
#define SPEED_PERCENT_TO_RPM(max_speed, percent) (( (max_speed) * percent)/100)
#define DATA_TO_DRV_VOLT(data) (((data[0] << 8) | data[1]) >> 7)

typedef struct max6620_fan_data
{
    /*Number of tach pulse per revolution for the fan*/
    uint_t no_of_tach_pulse;
    /*Tach count period for the fan*/
    uint_t tach_count_period;
    /* Maximum Speed(in RPM) for the fan*/
    uint_t max_speed;
    /*Alias name for the fan*/
    char *alias;
}max6620_fan_data_t;

/*
 * MAX6620 device private data
 */
typedef struct max6620_device
{
    /*indicates if the fan must run at full speed when a fault happens*/
    bool is_full_speed_on_fail;
    max6620_fan_data_t max6620_fan[MAX6620_MAX_FANS];
    uint_t             fan_faults;
} max6620_device_t;

typedef struct max6620_resource_hdl
{
    sdi_device_hdl_t max6620_dev_hdl;
    uint_t fan_id;
}max6620_resource_hdl_t;

/* Sets the speed of the fan referred by resource*/
static t_std_error sdi_max6620_fan_speed_set(void *resource_hdl, uint_t speed);
/* This is the registration function for max6620 driver.*/
t_std_error sdi_max6620_register(std_config_node_t node, void *bus_handle,
                                 sdi_device_hdl_t* device_hdl);
/*Does the Chip level initialization for max6620*/
t_std_error sdi_max6620_chip_init(sdi_device_hdl_t device_hdl);

/*
 * Creates the resource handle for a specific fan.
 * Parametrs:
 * [in] dev_hdl - max6620 device handle
 * [in] fan_id - id for the specific fan
 * Return max6620_resource_hdl_t - resource handle for the specific fan
 */
static max6620_resource_hdl_t *sdi_max6620_create_resource_hdl(sdi_device_hdl_t dev_hdl, uint_t fan_id)
{
    max6620_resource_hdl_t *max6620_resource = NULL;

    STD_ASSERT(dev_hdl != NULL);

    max6620_resource = calloc(sizeof(max6620_resource_hdl_t), 1);
    STD_ASSERT(max6620_resource != NULL);

    max6620_resource->max6620_dev_hdl = dev_hdl;
    max6620_resource->fan_id = fan_id;

    return max6620_resource;
}

/* This function will enable the full speed when fault occurs
 * Parameters:
 * [in] device_hdl - device handle of the specific device
 * [in] enable - flag to enable or disable the feature
 * Return - STD_ERR_OK for success and the respective error code from i2c api in case of failure
 */
static t_std_error sdi_max6620_onfail_fullspeed_enable(sdi_device_hdl_t device_hdl, bool enable)
{
    max6620_device_t *max6620_data = NULL;
    uint8_t buf = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    max6620_data = (max6620_device_t*)device_hdl->private_data;
    STD_ASSERT(max6620_data != NULL);

    rc = sdi_smbus_read_byte(device_hdl->bus_hdl, device_hdl->addr.i2c_addr,
                             MAX6620_GLOBALCFG, &buf,SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 read failure at addr: %d reg: %d rc: %d\n",
                              device_hdl->addr.i2c_addr, MAX6620_GLOBALCFG, rc);
        return rc;
    }

    if (enable)
    {
        buf |= MAX6620_FAN_CTRL_FAN_FULL_SPD_MASK;
    }
    else
    {
        buf &= (~MAX6620_FAN_CTRL_FAN_FULL_SPD_MASK);
    }

    rc = sdi_smbus_write_byte(device_hdl->bus_hdl, device_hdl->addr.i2c_addr,
                              MAX6620_GLOBALCFG, buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 write failure at addr: %d reg: %d rc: %d\n",
                              device_hdl->addr.i2c_addr, MAX6620_GLOBALCFG, rc);
    }

    return rc;
}

/* Sets the fan fault mask. this will be used to clear all the fault masks
 * Parameters:
 * [in] device_hdl - device handle of the specific device
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_clear_fan_fault(sdi_device_hdl_t device_hdl)
{
    uint8_t buf = 0x00;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    rc = sdi_smbus_write_byte(device_hdl->bus_hdl, device_hdl->addr.i2c_addr,
                              MAX6620_FANFAULT, buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 write failure at addr: %d reg: %d rc: %d\n",
                              device_hdl->addr.i2c_addr, MAX6620_FANFAULT, rc);
    }

    return rc;
}

/* Sets the tach count period for a given fan
 * Parameters:
 * [in] device_hdl - device handle of the specific device
 * [in] fan_id - the id of the fan that is of interest
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_fan_tach_count_period_set(sdi_device_hdl_t device_hdl,
                                                         uint_t fan_id)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t buf = 0, data = 0;
    sdi_device_hdl_t chip = NULL;
    max6620_device_t *max6620_data = NULL;

    STD_ASSERT(device_hdl != NULL);

    chip = (sdi_device_hdl_t)device_hdl;
    STD_ASSERT(chip != NULL);

    max6620_data = (max6620_device_t*)chip->private_data;
    STD_ASSERT(max6620_data != NULL);

    /*Setting the speed range to the optimum*/
    if((max6620_data->max6620_fan[fan_id].max_speed > 0) &&
       (max6620_data->max6620_fan[fan_id].max_speed <= 500))
    {
        max6620_data->max6620_fan[fan_id].tach_count_period = 1;
        data = 0;
    }
    else if((max6620_data->max6620_fan[fan_id].max_speed > 500) &&
            (max6620_data->max6620_fan[fan_id].max_speed <= 1000))
    {
        max6620_data->max6620_fan[fan_id].tach_count_period = 2;
        data = 1;
    }
    else if((max6620_data->max6620_fan[fan_id].max_speed) > 1000 &&
            (max6620_data->max6620_fan[fan_id].max_speed <= 2000))
    {
        max6620_data->max6620_fan[fan_id].tach_count_period = 4;
        data = 2;
    }
    else if((max6620_data->max6620_fan[fan_id].max_speed) > 2000 &&
            (max6620_data->max6620_fan[fan_id].max_speed <= 4000))
    {
        max6620_data->max6620_fan[fan_id].tach_count_period = 8;
        data = 3;
    }
    else if((max6620_data->max6620_fan[fan_id].max_speed > 4000) &&
            (max6620_data->max6620_fan[fan_id].max_speed <= 8000))
    {
        max6620_data->max6620_fan[fan_id].tach_count_period = 16;
        data = 4;
    }
    else
    {
        max6620_data->max6620_fan[fan_id].tach_count_period = 32;
        data = 5;
    }

    /*Bit 5:7 are the TACH period fields*/
    data = data << 5;

    rc = sdi_smbus_read_byte(device_hdl->bus_hdl, device_hdl->addr.i2c_addr,
                             MAX6620_FANDYN(fan_id), &buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 read failure at addr: %d reg: %d rc: %d\n",
                              device_hdl->addr.i2c_addr, MAX6620_FANDYN(fan_id), rc);
        return rc;
    }

    buf = (buf & ~(MAX6620_FAN_CTRL_SPEED_RANGE_MASK)) |
        (data & MAX6620_FAN_CTRL_SPEED_RANGE_MASK);


    rc = sdi_smbus_write_byte(device_hdl->bus_hdl, device_hdl->addr.i2c_addr,
                              MAX6620_FANDYN(fan_id), buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 write failure at addr: %d reg: %d rc: %d\n",
                              device_hdl->addr.i2c_addr, MAX6620_FANDYN(fan_id), rc);
        return rc;
    }

    return rc;
}

/* Retrieve the tach count for a given fan
 * Parameters:
 * [in] resource_hdl - Resource handle for the specific resource
 * [out] tach_count - tach count will be returned in this
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_fan_tach_count_get(max6620_resource_hdl_t* resource_hdl, uint_t *tach_count)
{
    sdi_device_hdl_t chip = NULL;
    uint8_t buf[2] = {0};
    uint_t fan_id = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(tach_count != NULL);

    fan_id = resource_hdl->fan_id;

    chip = resource_hdl->max6620_dev_hdl;
    STD_ASSERT(chip != NULL);

    rc = sdi_smbus_read_word(chip->bus_hdl, chip->addr.i2c_addr, MAX6620_FANTACHCNT(fan_id),
                             (uint16_t*)&buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 read failure at addr: %d reg: %d rc: %d\n",
                              chip->addr.i2c_addr, MAX6620_FANTACHCNT(fan_id), rc);
        return rc;
    }

    /* Bits-0:7 in MSB and Bits-5:7 only used in the LSB */
    *tach_count = TACH_COUNT_VAL(buf[1],buf[0]);

    return rc;

}

/* Configures the target tach count for a given fan
 * Parameters:
 * [in] resource_hdl - Resource handle for the specific resource
 * [in] target_tach_count - target tach count value
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_fan_target_tach_count_set(max6620_resource_hdl_t* resource_hdl, uint_t target_tach_count)
{
    sdi_device_hdl_t chip = NULL;
    uint8_t buf[2] = {0}, data[2] = {0};
    uint_t fan_id = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    fan_id = resource_hdl->fan_id;

    chip = resource_hdl->max6620_dev_hdl;
    STD_ASSERT(chip != NULL);


    data[0] = ( ( target_tach_count & 0x7FF ) >> 3 )& 0xff ; /*Bit 10:3 -> 7: 0 - msb data */
    data[1] = ( ( target_tach_count & 0x7   ) << 5 )& 0xff ; /*Bit 2: 0 -> 7 :5 - lsb data */

    rc = sdi_smbus_read_word(chip->bus_hdl, chip->addr.i2c_addr, MAX6620_FANTGTTACHCNT(fan_id),
                             (uint16_t*)buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 read failure at addr: %d reg: %d rc: %d\n",
                              chip->addr.i2c_addr, MAX6620_FANTGTTACHCNT(fan_id), rc);
        return rc;
    }

    buf[0] = data[0];
    buf[1] = (buf[1] & ~(0xE0)) | (data[1] & 0xE0);

    rc = sdi_smbus_write_word(chip->bus_hdl, chip->addr.i2c_addr, MAX6620_FANTGTTACHCNT(fan_id),
                              *((uint16_t*)buf), SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 write failure at addr: %d reg: %d rc: %d\n",
                              chip->addr, MAX6620_FANTGTTACHCNT(fan_id), rc);
    }

    return rc;
}

/* Retrieve the current fan drive voltage for a given fan
 * Parameters:
 * [in] resource_hdl - Resource handle for the specific resource
 * [out] volt - current fan drive voltage
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_fan_current_drive_volt_get(max6620_resource_hdl_t* resource_hdl,
                                                          uint16_t *volt)
{
    sdi_device_hdl_t chip = NULL;
    uint_t fan_id = 0;
    uint8_t data[2] = {0};
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    fan_id = resource_hdl->fan_id;

    chip = resource_hdl->max6620_dev_hdl;
    STD_ASSERT(chip != NULL);

    rc = sdi_smbus_read_word(chip->bus_hdl, chip->addr.i2c_addr, MAX6620_FANTGTDRVVOLT(fan_id),
                             (uint16_t*)data, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 read failure at addr: %d reg: %d rc: %d\n",
                              chip->addr.i2c_addr, MAX6620_FANTGTDRVVOLT(fan_id), rc);
    }

    *volt = DATA_TO_DRV_VOLT(data);

    return rc;
}

/* Helper function to init the fan
 * Parameters:
 * [in] device_hdl - Handle of the max6620 device
 * [in] fan_id - Id of the fan that is of interest
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_init_fan(sdi_device_hdl_t device_hdl, uint_t fan_id)
{
    t_std_error rc = STD_ERR_OK;
    uint8_t data = 0;
    uint16_t drv_volt = 0;
    uint_t speed_percent = 0, rpm = 0;
    sdi_device_hdl_t chip = NULL;
    max6620_device_t *max6620_data = NULL;

    STD_ASSERT(device_hdl != NULL);

    chip = (sdi_device_hdl_t)device_hdl;
    STD_ASSERT(chip != NULL);

    max6620_data = (max6620_device_t*)chip->private_data;
    STD_ASSERT(max6620_data != NULL);


    /*Sets the tach count Period */
    rc = sdi_max6620_fan_tach_count_period_set(device_hdl, fan_id);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620_fan_tach_count_period_set failed for fan- %d rc: %d\n",
                              fan_id, rc);
    }

    /*Set the initial RPM corresponding to the current drive voltage*/
    rc = sdi_max6620_fan_current_drive_volt_get(sdi_max6620_create_resource_hdl(device_hdl, fan_id),
                                                &drv_volt);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_max6620_fan_current_drive_volt_get failed for fan- %d rc: %d\n",
                              fan_id, rc);
        return rc;
    }

    speed_percent = DRV_VOLT_TO_SPEED_PERCENT(drv_volt);
    rpm = SPEED_PERCENT_TO_RPM(max6620_data->max6620_fan[fan_id].max_speed, speed_percent);

    rc = sdi_max6620_fan_speed_set(sdi_max6620_create_resource_hdl(device_hdl, fan_id), rpm);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620_fan_speed_set failed for fan- %d rc: %d\n", fan_id, rc);
        return rc;
    }

    /*Init Configuration:
    * Low Locked polarity
    * Tach Count enable
    * Tach Input disable
    * Spin up time - 1s
    * RPM mode
    */
    data =    MAX6620_FAN_CTRL_RPM_MODE_EN_MASK | MAX6620_FAN_CTRL_SPIN_UP_1S_MASK ;

    rc = sdi_smbus_write_byte(device_hdl->bus_hdl, device_hdl->addr.i2c_addr,
                              MAX6620_FANCFG(fan_id), data, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 write failure at addr: %d reg: %d rc: %d\n",
                              device_hdl->addr.i2c_addr, MAX6620_FANCFG(fan_id), rc);
    }

    return rc;
}

 /*
 * Callback function to initialize the  fan referred by resource
 * [in] resource_hdl - callback data for this function
 * [in] max_rpm - Maximum speed of the fan referred by resource
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_resource_init(void *resource_hdl, uint_t max_rpm)
{
    sdi_device_hdl_t chip = NULL;
    max6620_device_t *max6620_data = NULL;
    uint_t fan_id = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);

    fan_id = ((max6620_resource_hdl_t*)resource_hdl)->fan_id;

    chip = ((max6620_resource_hdl_t*)resource_hdl)->max6620_dev_hdl;
    STD_ASSERT(chip != NULL);

    max6620_data = (max6620_device_t*)chip->private_data;
    STD_ASSERT(max6620_data != NULL);

    if(max_rpm != 0)
    {
        max6620_data->max6620_fan[fan_id].max_speed = max_rpm;
    }

    rc = sdi_max6620_init_fan(chip, fan_id);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("sdi_max6620_init failed.  rc: %d\n", rc);
    }

    return rc;
}

/*
 * Callback function to retrieve the speed of the fan referred by resource
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [out] speed - pointer to a buffer to get the fan speed
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
/*  Formula for calculating speed
 *  RPM  =  60 x SR x 8192
 *          --------------
 *          NP x tach count
 * where
 * NP = number of tach pulse per revolution.
 * SR = TachCount Period
 *
 * Here
 * SR = Speed Range (1,2,4,8,16,32) Value in the fan_dynamic register - 06h,07h,08h,09h
 * NP = For a nominal fan it will be two.
 */
static t_std_error sdi_max6620_fan_speed_get(void *resource_hdl, uint_t *speed)
{
    uint_t tach_count = 0;
    uint_t fan_id = 0;
    sdi_device_hdl_t chip = NULL;
    max6620_device_t *max6620_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(speed != NULL);

    fan_id = ((max6620_resource_hdl_t*)resource_hdl)->fan_id;

    chip = ((max6620_resource_hdl_t*)resource_hdl)->max6620_dev_hdl;
    STD_ASSERT(chip != NULL);

    max6620_data = (max6620_device_t*)chip->private_data;
    STD_ASSERT(max6620_data != NULL);

    rc = sdi_max6620_fan_tach_count_get(resource_hdl, &tach_count);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620_fan_tach_count_get failed. rc: %d\n",rc);
        return rc;
    }

    if(tach_count == 0 )
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 tach_count is zero. rc: %d\n",rc);
    }
    else if(tach_count == MAX6620_MAX_TACH_COUNT)
    {
        *speed = 0;
    }
    else
    {
        *speed = ( (max6620_data->max6620_fan[fan_id].tach_count_period) * ( 60 * MAX6620_FREQUENCY_HZ ) )
            / ( (max6620_data->max6620_fan[fan_id].no_of_tach_pulse) * tach_count );
    }

     return rc;
}

/*
 * Callback function to set the speed of the fan referred by resource
 * Parameters:
 * [in] resource_hdl - callback data for this function,chip instance is passed as a callback data
 * [in] speed - Speed to be set
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_fan_speed_set(void *resource_hdl, uint_t speed)
{
    uint_t tgt_tach_count = 0;
    uint_t fan_id = 0;
    sdi_device_hdl_t chip = NULL;
    max6620_device_t *max6620_data = NULL;
    t_std_error rc = STD_ERR_OK;

    fan_id = ((max6620_resource_hdl_t*)resource_hdl)->fan_id;

    chip = ((max6620_resource_hdl_t*)resource_hdl)->max6620_dev_hdl;
    STD_ASSERT(chip != NULL);

    max6620_data = (max6620_device_t*)chip->private_data;
    STD_ASSERT(max6620_data != NULL);

    if ( ( speed < 0 ) || ( speed > max6620_data->max6620_fan[fan_id].max_speed ) )
    {
        return SDI_DEVICE_ERRCODE(EOPNOTSUPP);
    }

    if(speed == 0)
    {
        tgt_tach_count = MAX6620_MAX_TACH_COUNT;
    }
    else
    {
        tgt_tach_count = ((max6620_data->max6620_fan[fan_id].tach_count_period)
                          * ( 60 * MAX6620_FREQUENCY_HZ ) )
            / ( (max6620_data->max6620_fan[fan_id].no_of_tach_pulse) * speed );
    }

    rc = sdi_max6620_fan_target_tach_count_set(resource_hdl, tgt_tach_count);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620_fan_target_tach_count_set failed. rc: %d\n", rc);
    }

    return rc;
}

/*
 * Callback function to retrieve the fault status of the fan referred by resource
 * Parameters:
 * [in] resource_hdl - callback data for this function
 * [in] status - pointer to a buffer to get the fault status of the fan
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
static t_std_error sdi_max6620_fan_status_get(void *resource_hdl, bool *status)
{
    sdi_device_hdl_t chip = NULL;
    uint8_t buf = 0;
    uint_t fan_id = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    fan_id = ((max6620_resource_hdl_t*)resource_hdl)->fan_id;

    chip = ((max6620_resource_hdl_t*)resource_hdl)->max6620_dev_hdl;
    STD_ASSERT(chip != NULL);

    *status = false;

    rc = sdi_smbus_read_byte(chip->bus_hdl, chip->addr.i2c_addr, MAX6620_FANFAULT,
                             &buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("max6620 read failure at addr: %d reg: %d rc: %d\n",
                              chip->addr.i2c_addr, MAX6620_FANFAULT, rc);
        return rc;
    }

    max6620_device_t *max6620_data  = (max6620_device_t *) chip->private_data;
    uint_t           m = 1 << (fan_id + 4);

    *status = (((max6620_data->fan_faults |= buf) & m) != 0);
    if (*status) {
        /* Fault detected => Re-set tach value to re-enable fault detection */

        uint16_t tach;

        rc = sdi_smbus_read_word(chip->bus_hdl, chip->addr.i2c_addr, MAX6620_FANTGTTACHCNT(fan_id),
                                 &tach, SDI_I2C_FLAG_NONE);
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("max6620 read failure at addr: %d reg: %d rc: %d\n",
                                  chip->addr.i2c_addr, MAX6620_FANTGTTACHCNT(fan_id), rc);
            return rc;
        }
        
        rc = sdi_smbus_write_word(chip->bus_hdl, chip->addr.i2c_addr, MAX6620_FANTGTTACHCNT(fan_id),
                                  tach, SDI_I2C_FLAG_NONE);
        
        if (rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("max6620 write failure at addr: %d reg: %d rc: %d\n",
                                  chip->addr, MAX6620_FANTGTTACHCNT(fan_id), rc);

            return (rc);
        }
        
        max6620_data->fan_faults &= ~m;
    }

    return rc;
}

fan_ctrl_t max6620_fan_resource = {
        sdi_max6620_resource_init,
        sdi_max6620_fan_speed_get,
        sdi_max6620_fan_speed_set,
        sdi_max6620_fan_status_get
};


/* Export the Driver table */
sdi_driver_t max6620_entry = {
        sdi_max6620_register,
        sdi_max6620_chip_init
};


/*
 * Update the database for a specific fan.
 * Parametrs:
 * [in] cur_node - temperature sensor node
 * [in] device_hdl - device handle of max6620 device
 * Return - none
 */
static void sdi_max6620_device_database_init(std_config_node_t cur_node, void* device_hdl)
{
    uint_t fan_id = 0;
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    max6620_device_t *max6620_data = NULL;
    size_t node_attr_len = 0;

    STD_ASSERT(device_hdl != NULL);

    chip = (sdi_device_hdl_t)device_hdl;
    STD_ASSERT(chip != NULL);

    max6620_data = (max6620_device_t*)chip->private_data;
    STD_ASSERT(max6620_data != NULL);

    if (strncmp(std_config_name_get(cur_node), SDI_DEV_NODE_FAN, strlen(SDI_DEV_NODE_FAN)) != 0)
    {
        return;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_INSTANCE);
    if(node_attr != NULL)
    {
        fan_id = (uint_t) strtoul(node_attr, NULL, 0);
    }

    STD_ASSERT(fan_id < MAX6620_MAX_FANS);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_FAN_NP);
    if(node_attr != NULL)
    {
        max6620_data->max6620_fan[fan_id].no_of_tach_pulse = (uint_t) strtoul(node_attr, NULL, 0);
    }
    else
    {
        max6620_data->max6620_fan[fan_id].no_of_tach_pulse = MAX6620_DEFAULT_TACH_PULSE_COUNT;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_FAN_SPEED);
    STD_ASSERT(node_attr != NULL);
    max6620_data->max6620_fan[fan_id].max_speed = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_ALIAS );
    if(node_attr == NULL)
    {
        max6620_data->max6620_fan[fan_id].alias = malloc(SDI_MAX_NAME_LEN);
        STD_ASSERT( max6620_data->max6620_fan[fan_id].alias != NULL);
        snprintf(max6620_data->max6620_fan[fan_id].alias, SDI_MAX_NAME_LEN, "max6620-%d-%d",
                 chip->instance, fan_id);
    }
    else
    {
        node_attr_len = strlen(node_attr)+1;
        max6620_data->max6620_fan[fan_id].alias = malloc(node_attr_len);
        STD_ASSERT( max6620_data->max6620_fan[fan_id].alias != NULL);
        memcpy(max6620_data->max6620_fan[fan_id].alias, node_attr, node_attr_len);
    }

    sdi_resource_add(SDI_RESOURCE_FAN,max6620_data->max6620_fan[fan_id].alias,
                        sdi_max6620_create_resource_hdl(chip,fan_id), &max6620_fan_resource);

}

/*
 * The configuration file format for the MAX6620 device node is as follows
 *<max6620 driver="max6620" instance="<chip_instance>" addr="<address of the chip>
 * enable_full_speed=<yes/no>"/>
 *<fan instance="<fan no>" alias="<fan alias>" tach_period_count="<tach count period>"/>
 *<fan instance="<fan no>" alias="<fan alias>" tach_period_count="<tach count period>"/>
 *</max6620>
 * Mandatory attributes    : instance and addr
 */

/* Registers the fan and it's resources with the SDI framework.
 * Parameters:
 * [in] node - max6620 device node from the configuration file
 * [in] bus_handle - Parent bus handle of the device
 * [out] device_hdl - Pointer to the max6620 device handle which will get filled by this function
 * Return - STD_ERR_OK, this function is kept as non void as the driver table function pointer
 * requires it
 */
t_std_error sdi_max6620_register(std_config_node_t node, void *bus_handle,
                                 sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    max6620_device_t *max6620_data = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_I2C_BUS);

    chip = calloc(sizeof(sdi_device_entry_t), 1);
    STD_ASSERT(chip != NULL);

    max6620_data = calloc(sizeof(max6620_device_t), 1);
    STD_ASSERT(max6620_data != NULL);

    chip->bus_hdl = bus_handle;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    chip->addr.i2c_addr = (i2c_addr_t)strtoul(node_attr, NULL, 16);

    chip->callbacks = &max6620_entry;
    chip->private_data = (void*)max6620_data;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_FAN_EN_FS);
    if((node_attr != NULL) && (strcmp(node_attr, "yes") == 0))
    {
        max6620_data->is_full_speed_on_fail = true;
    }
    else
    {
        max6620_data->is_full_speed_on_fail = false;
    }

    std_config_for_each_node(node, sdi_max6620_device_database_init, chip);

    *device_hdl = chip;

    return STD_ERR_OK;
}

/*
 * Does the chip level initialization for the max6620 device as specified in the config file
 * [in] device_hdl - device handle of the specific device
 * Return - STD_ERR_OK for success or the respective error code from i2c API in case of failure
 */
t_std_error sdi_max6620_chip_init(sdi_device_hdl_t device_hdl)
{
    max6620_device_t *max6620_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    max6620_data = (max6620_device_t*)device_hdl->private_data;
        STD_ASSERT(max6620_data != NULL);

    /* Disable auto speed change on failure */
    rc = sdi_max6620_onfail_fullspeed_enable(device_hdl, max6620_data->is_full_speed_on_fail);
    if ( rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("%s : %d - MAX6620DrvFanCtrlOnFailFullSpeedEnableSet failed \n",
                              __FUNCTION__, __LINE__);
        return rc;
    }

    /* Clear all fan fault mask bits */
    rc = sdi_max6620_clear_fan_fault(device_hdl);
    if ( rc != STD_ERR_OK)
    {
        SDI_DEVICE_ERRMSG_LOG("%s : %d - MAX6620DrvFanCtrlFanFaultMaskSet failed \n",
                              __FUNCTION__, __LINE__);
    }

    return rc;
}

