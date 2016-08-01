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
 * filename: sdi_emc2305.c
 */


/******************************************************************************
 * sdi_emc2305.c
 * Implements the driver for emc2305 fan controller chip. The EMC2305 chip can
 * control upto five fans.It supports two methods for fan control viz RPM mode
 * and Direct mode. This driver supports only the Direct mode.
 * The device provides the register interface for configuration and status
 * control of the fans.
 * For more information about the EMC2305, refer the data sheet in below link.
 * http://ww1.microchip.com/downloads/en/DeviceDoc/2305.pdf
 *
 * @todo: Rangex value has to be updated as per the speed.
 * Still discussions are in progress with hardware team.  Once get the input
 * will modify the rangex value.
 * Note: Default rangex and pole values are used.
 ******************************************************************************/

#include "sdi_emc2305.h"
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


typedef struct emc2305_fan_data {
    /* Maximum Speed(in RPM) for the fan*/
    uint_t max_speed;
    /* number of poles for the fan */
    uint8_t poles;
    /* number of edges for the fan */
    uint8_t edges;
    /* range tach value for the fan */
    uint8_t ranges;
    /*Alias name for the fan*/
    char *alias;
}emc2305_fan_data_t;

/*
 * EMC2305 device private data
 */
typedef struct emc2305_device {
    emc2305_fan_data_t emc2305_fan[EMC2305_MAX_FANS];
    sdi_emc2305_fan_control fan_control_type;
}emc2305_device_t;

typedef struct emc2305_resource_hdl {
    sdi_device_hdl_t emc2305_dev_hdl;
    uint_t fan_id;
}emc2305_resource_hdl_t;

/* Fault status registers */
static const uint8_t fault_status_reg[] = { EMC2305_FAN_STATUS_REG,
                                            EMC2305_FAN_STALL_STATUS_REG,
                                            EMC2305_FAN_SPIN_STATUS_REG,
                                            EMC2305_FAN_DRIVE_FAIL_STATUS_REG };

/* Status for individual fans */
static const uint8_t fan_fault_reg[] = { EMC2305_FAN_STALL_STATUS_REG,
                                         EMC2305_FAN_SPIN_STATUS_REG,
                                         EMC2305_FAN_DRIVE_FAIL_STATUS_REG };

/* Driver setting registers for individual fans */
static const uint8_t fan_driv_set_reg[] = { EMC2305_FAN0_SETTING_REG,
                                            EMC2305_FAN1_SETTING_REG,
                                            EMC2305_FAN2_SETTING_REG,
                                            EMC2305_FAN3_SETTING_REG,
                                            EMC2305_FAN4_SETTING_REG };

/* Fan Configuration registers for individual fans */
static const uint8_t fan_config1_reg[] = { EMC2305_FAN0_CFG1_REG,
                                           EMC2305_FAN1_CFG1_REG,
                                           EMC2305_FAN2_CFG1_REG,
                                           EMC2305_FAN3_CFG1_REG,
                                           EMC2305_FAN4_CFG1_REG };

static const uint8_t fan_config2_reg[] = { EMC2305_FAN0_CFG2_REG,
                                           EMC2305_FAN1_CFG2_REG,
                                           EMC2305_FAN2_CFG2_REG,
                                           EMC2305_FAN3_CFG2_REG,
                                           EMC2305_FAN4_CFG2_REG };

/* Pole Edge mapping */
static const uint8_t pole_edge_mapping[] = { EMC2305_FAN_POLE1_EDGE,
                                             EMC2305_FAN_POLE2_EDGE,
                                             EMC2305_FAN_POLE3_EDGE,
                                             EMC2305_FAN_POLE4_EDGE };

/* Range decode values.RPM ,Range mapping */
static const uint_t range_rpm_mapping[] = { EMC2305_RANGE0_MIN_RPM,
                                             EMC2305_RANGE1_MIN_RPM,
                                             EMC2305_RANGE2_MIN_RPM,
                                             EMC2305_RANGE3_MIN_RPM };

/* Fan Tach reading register for individual fans */
static const uint8_t fan_tach_reg[][2] = {
                         {EMC2305_FAN0_TACH_REG, EMC2305_FAN0_TACH_LBYTE_REG },
                         {EMC2305_FAN1_TACH_REG, EMC2305_FAN1_TACH_LBYTE_REG },
                         {EMC2305_FAN2_TACH_REG, EMC2305_FAN2_TACH_LBYTE_REG },
                         {EMC2305_FAN3_TACH_REG, EMC2305_FAN3_TACH_LBYTE_REG },
                         {EMC2305_FAN4_TACH_REG, EMC2305_FAN4_TACH_LBYTE_REG },
                         };

static const uint8_t fan_tach_target_reg[][2]= {
                              {EMC2305_FAN0_TACH_TARG_REG, EMC2305_FAN0_TACH_LTARG_REG},
                              {EMC2305_FAN1_TACH_TARG_REG, EMC2305_FAN1_TACH_LTARG_REG},
                              {EMC2305_FAN2_TACH_TARG_REG, EMC2305_FAN2_TACH_LTARG_REG},
                              {EMC2305_FAN3_TACH_TARG_REG, EMC2305_FAN3_TACH_LTARG_REG},
                              {EMC2305_FAN4_TACH_TARG_REG, EMC2305_FAN4_TACH_LTARG_REG},
                              };

/* Sets the speed of the fan referred by resource*/
static t_std_error sdi_emc2305_fan_speed_set(void *resource_hdl, uint_t speed);
/* This is the registration function for emc2305 driver.*/
static t_std_error sdi_emc2305_register(std_config_node_t node, void *bus_handle,
                                        sdi_device_hdl_t* device_hdl);
/*Does the Chip level initialization for emc2305*/
static t_std_error sdi_emc2305_chip_init(sdi_device_hdl_t device_hdl);

/*
 * Convert speed percentage value to volt.
 * percent[in] - voltage
 * Return      - corresponding voltage value.
 */
static inline uint8_t sdi_speed_percent_to_drv_volt_get(uint8_t percent)
{
    return ((percent * EMC2305_FAN_SETTING_MAX_VAL) / EMC2305_PERCENT_VAL);
}

/*
 * Convert RPM to speed percentage.
 * max_speed[in] - Max Fan speed.
 * rpm[in]       - RPM value.
 * Return        - corresponding speed_percent value.
 */
static inline uint8_t sdi_rpm_to_speed_percent_get(uint_t max_speed, uint_t rpm)
{
    return ((rpm * EMC2305_PERCENT_VAL) / max_speed);
}

/*
 * Get number of poles for the fan resource.
 * data[in]    - register value
 * Return      - corresponding pole value.
 */
static inline uint8_t sdi_fan_poles_get(uint8_t data)
{
   return (((data & EMC2305_FAN_POLE_MASK) >> EMC2305_FAN_POLE_SHIFT_BITS) + 1);
}

/*
 * Get TACH range value for the fan resource.
 * data[in]    - register value
 * Return      - corresponding range value.
 */
static inline uint8_t sdi_fan_ranges_get(uint8_t data)
{
   return(1 << (((data & EMC2305_FAN_RANGE_MASK) >> EMC2305_FAN_RANGE_SHIFT_BITS)));
}

/*
 * Creates the resource handle for a specific fan.
 * dev_hdl[in] - emc2305 device handle
 * fan_id[in]  - id for the specific fan
 * Return emc2305_resource_hdl_t - resource handle for the specific fan
 */
static emc2305_resource_hdl_t *sdi_emc2305_create_resource_hdl(
                               sdi_device_hdl_t dev_hdl, uint_t fan_id)
{
    emc2305_resource_hdl_t *emc2305_resource = NULL;

    STD_ASSERT(dev_hdl != NULL);

    emc2305_resource = calloc(1, sizeof(emc2305_resource_hdl_t));
    STD_ASSERT(emc2305_resource != NULL);

    emc2305_resource->emc2305_dev_hdl = dev_hdl;
    emc2305_resource->fan_id = fan_id;

    return emc2305_resource;
}

/* Clear the fan fault status in the EMC2305 Controller.
 * Parameters:
 * device_hdl[in] - device handle of the specific device
 * Return         - STD_ERR_OK for success or the respective error code from
 *                  i2c API in case of failure
 */
static t_std_error sdi_emc2305_clear_fan_fault(sdi_device_hdl_t device_hdl)
{
    uint8_t buf = 0, index = 0;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    for (index = 0; index < sizeof(fault_status_reg); index++) {
         /* All the status register are Read-On-Clear */
         rc = sdi_smbus_read_byte(device_hdl->bus_hdl, device_hdl->addr.i2c_addr,
                                  fault_status_reg[index], &buf, SDI_I2C_FLAG_NONE);
         if (rc != STD_ERR_OK) {
             SDI_DEVICE_ERRMSG_LOG("%s:read failure at addr:0x%x reg:%d rc:%d",
                                   __FUNCTION__, device_hdl->addr.i2c_addr,
                                   fault_status_reg[index], rc);
             return rc;
         }
    }
    return rc;
}

/*
 * Callback function to initialize the fan referred by resource.
 * resource_hdl[in] - callback data
 * max_rpm[in]      - Maximum speed of the fan referred by resource
 * Return           - STD_ERR_OK for success or the respective error code from
 *                    i2c API in case of failure
 */
static t_std_error sdi_emc2305_resource_init(void *resource_hdl, uint_t max_rpm)
{
    uint_t fan_id = 0;
    t_std_error rc = STD_ERR_OK;
    sdi_device_hdl_t chip = NULL;
    emc2305_device_t *emc2305_data = NULL;

    STD_ASSERT(resource_hdl != NULL);
    fan_id = ((emc2305_resource_hdl_t*)resource_hdl)->fan_id;

    chip = ((emc2305_resource_hdl_t*)resource_hdl)->emc2305_dev_hdl;
    STD_ASSERT(chip != NULL);

    emc2305_data = (emc2305_device_t*)chip->private_data;
    STD_ASSERT(emc2305_data != NULL);

    if((emc2305_data->emc2305_fan[fan_id].max_speed == 0) && (max_rpm != 0)) {
        emc2305_data->emc2305_fan[fan_id].max_speed = max_rpm;
    }
    rc = sdi_emc2305_fan_speed_set(sdi_emc2305_create_resource_hdl(chip,
                                   fan_id),
                                   emc2305_data->emc2305_fan[fan_id].max_speed);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("emc2305_fan_speed_set failed for fan-%d rc: %d",
                               fan_id, rc);
    }
    return rc;
}

/*
 * Callback function to retrieve the speed of the fan referred by resource
 * resource_hdl[in] - chip instance is passed as a callback data
 * speed[out]       - pointer to a buffer to get the fan speed
 * Return           - STD_ERR_OK for success or the respective error code from
 *                    i2c API in case of failure
 *
 * Formula to calculate the RPM
 * =============================
 * RPM =    1           (N - 1)
 *        -----   *    ---------- * FREQ * 60
 *        POLES        COUNT * 1
 *                            ---
 *                             M
 *
 *   where as
 *      POLES - Number of poles of the fan
 *      COUNT - TACH Reading register value
 *      FREQ  - Tachometer measurement freq
 *      N     - Number of edges
 *      M     - Multiplier defined by the Range bits
 */
static t_std_error sdi_emc2305_fan_speed_get(void *resource_hdl, uint_t *speed)
{
    uint8_t tach= 0, ltach = 0;
    uint16_t count = 0;
    uint_t fan_id = 0;
    sdi_device_hdl_t chip = NULL;
    emc2305_device_t *emc2305_data = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(speed != NULL);
    *speed = 0;

    fan_id = ((emc2305_resource_hdl_t*)resource_hdl)->fan_id;
    chip = ((emc2305_resource_hdl_t*)resource_hdl)->emc2305_dev_hdl;
    STD_ASSERT(chip != NULL);

    emc2305_data = (emc2305_device_t*)chip->private_data;
    STD_ASSERT(emc2305_data != NULL);

    rc = sdi_smbus_read_byte(chip->bus_hdl, chip->addr.i2c_addr,
                   fan_tach_reg[fan_id][EMC2305_INDEX0], &tach, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s: read failure at addr: 0x%x reg: %d rc: %d",
                              __FUNCTION__, chip->addr.i2c_addr,
                              fan_tach_reg[fan_id][EMC2305_INDEX0], rc);
        return rc;
    }
    rc = sdi_smbus_read_byte(chip->bus_hdl, chip->addr.i2c_addr,
                   fan_tach_reg[fan_id][EMC2305_INDEX1], &ltach, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s: read failure at addr: 0x%x reg: %d rc: %d",
                              __FUNCTION__, chip->addr.i2c_addr,
                              fan_tach_reg[fan_id][EMC2305_INDEX1], rc);
        return rc;
    }

    count = (((tach << BITS_PER_BYTE) | ltach) >> EMC2305_FAN_LTACH_SHIFT_BITS);
    *speed = (((emc2305_data->emc2305_fan[fan_id].edges - 1) * EMC2305_TACH_FREQ *
               emc2305_data->emc2305_fan[fan_id].ranges * EMC2305_RPM_CONST_VAL)
              / (count * emc2305_data->emc2305_fan[fan_id].poles));

    return rc;
}

static uint16_t sdi_rpm_to_tach_count (emc2305_fan_data_t *fan_data, uint_t speed)
{
    uint16_t tachval;

    tachval = ((((fan_data->edges - 1) * EMC2305_TACH_FREQ * EMC2305_RPM_CONST_VAL) / 
                fan_data->poles)* (fan_data->poles)) / speed;
    tachval = tachval << EMC2305_FAN_LTACH_SHIFT_BITS;

    return tachval;
}

/*
 * Callback function to set the speed of the fan referred by resource
 * Parameters:
 * resource_hdl[in] - callback data
 * speed[in]        - Speed to be set
 * Return           - STD_ERR_OK for success or the respective error code from
 *                    i2c API in case of failure
 */
static t_std_error sdi_emc2305_fan_speed_set(void *resource_hdl, uint_t speed)
{
    uint_t fan_id = 0;
    uint8_t setting= 0;
    uint8_t speed_percent = 0;
    t_std_error rc = STD_ERR_OK;
    sdi_device_hdl_t chip = NULL;
    emc2305_device_t *emc2305_data = NULL;

    fan_id = ((emc2305_resource_hdl_t*)resource_hdl)->fan_id;

    chip = ((emc2305_resource_hdl_t*)resource_hdl)->emc2305_dev_hdl;
    STD_ASSERT(chip != NULL);

    emc2305_data = (emc2305_device_t*)chip->private_data;
    STD_ASSERT(emc2305_data != NULL);

    if (speed > emc2305_data->emc2305_fan[fan_id].max_speed) {
        speed = emc2305_data->emc2305_fan[fan_id].max_speed;
    }

    if (emc2305_data->fan_control_type == EMC2305_FAN_CONTROL_RPM) {
        uint8_t tach[2];

        uint16_t tachval = sdi_rpm_to_tach_count(&emc2305_data->emc2305_fan[fan_id],
                                                 speed);
        tach[1] = (tachval & 0xff00) >> BITS_PER_BYTE;
        tach[0] = (tachval & 0xff );

        rc = sdi_smbus_write_byte(chip->bus_hdl, chip->addr.i2c_addr,
                       fan_tach_target_reg[fan_id][EMC2305_INDEX0], tach[1],
                       SDI_I2C_FLAG_NONE);
        if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s: write failure at addr: 0x%x reg: %d rc: %d",
                    __FUNCTION__, chip->addr.i2c_addr,
                    fan_tach_target_reg[fan_id][EMC2305_INDEX0], rc);
            return rc;
        }

        rc = sdi_smbus_write_byte(chip->bus_hdl, chip->addr.i2c_addr,
                       fan_tach_target_reg[fan_id][EMC2305_INDEX1], tach[0],
                       SDI_I2C_FLAG_NONE);

        if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s: write failure at addr: 0x%x reg: %d rc: %d",
                    __FUNCTION__, chip->addr.i2c_addr,
                    fan_tach_target_reg[fan_id][EMC2305_INDEX1], rc);
            return rc;
        }

        rc = sdi_smbus_write_byte(chip->bus_hdl, chip->addr.i2c_addr,
                fan_config1_reg[fan_id], emc2305_data->fan_control_type, 1);

        if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s: write failure at addr: 0x%x reg: %d rc: %d",
                    __FUNCTION__, chip->addr.i2c_addr, fan_config1_reg[fan_id], rc);
        }

    } else {

        /* Set the fan speed */
        speed_percent = sdi_rpm_to_speed_percent_get(
                emc2305_data->emc2305_fan[fan_id].max_speed, speed);
        setting = sdi_speed_percent_to_drv_volt_get(speed_percent);
        rc = sdi_smbus_write_byte(chip->bus_hdl, chip->addr.i2c_addr,
                fan_driv_set_reg[fan_id], setting, SDI_I2C_FLAG_NONE);

        if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s: write failure at addr: 0x%x reg: %d rc: %d",
                    __FUNCTION__, chip->addr.i2c_addr,
                    fan_driv_set_reg[fan_id], rc);
        }
    }
    return rc;
}

/*
 * Callback function to retrieve the fault status of the fan referred by resource
 * it will check the stall, spin, drive fail status
 * Parameters:
 * resource_hdl[in] - callback data for this function
 * status[out]      - pointer to a buffer to get the fault status of the fan.
 *                    will be true, if any fault.otherwise false.
 * Return           - STD_ERR_OK for success or the respective error code from
 *                    i2c API in case of failure
 */
static t_std_error sdi_emc2305_fan_status_get(void *resource_hdl, bool *status)
{
    uint_t fan_id = 0;
    uint8_t buf = 0, index = 0;
    sdi_device_hdl_t chip = NULL;
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(resource_hdl != NULL);
    STD_ASSERT(status != NULL);

    fan_id = ((emc2305_resource_hdl_t*)resource_hdl)->fan_id;

    chip = ((emc2305_resource_hdl_t*)resource_hdl)->emc2305_dev_hdl;
    STD_ASSERT(chip != NULL);

    *status = false;
    for (index = 0; index < sizeof(fan_fault_reg); index++) {
         rc = sdi_smbus_read_byte(chip->bus_hdl, chip->addr.i2c_addr,
                                  fan_fault_reg[index], &buf, SDI_I2C_FLAG_NONE);
         if(rc != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:read failure at addr:0x%x reg: %d rc: %d",
                                  __FUNCTION__, chip->addr.i2c_addr,
                                  fan_fault_reg[index], rc);
            return rc;
         }
         *status = (buf & (1 << fan_id)) ? true : false;
         if (*status == true) {
             SDI_DEVICE_TRACEMSG_LOG("Fan fault status addr=0x%x reg=%d buf=%d",
                                 chip->addr.i2c_addr, fan_fault_reg[index], buf);
             return rc;

         }
    }
    /* Check Watchdog fault status */
    rc = sdi_smbus_read_byte(chip->bus_hdl, chip->addr.i2c_addr,
                                  EMC2305_FAN_STATUS_REG, &buf, SDI_I2C_FLAG_NONE);
    if(rc != STD_ERR_OK) {
       SDI_DEVICE_ERRMSG_LOG("%s:read failure at addr:0x%x reg:%d rc:%d",
                             __FUNCTION__, chip->addr.i2c_addr,
                             EMC2305_FAN_STATUS_REG, rc);
       return rc;
    }
    *status = (buf & EMC2305_FAN_WATCHDOG_STATUS_MASK) ? true : false;
    if (*status == true) {
        SDI_DEVICE_TRACEMSG_LOG("Fan Watchdog status addr=0x%x reg=%d buf=%d",
                                 chip->addr.i2c_addr, EMC2305_FAN_STATUS_REG, buf);
    }
    return rc;
}

fan_ctrl_t emc2305_fan_resource = {
        sdi_emc2305_resource_init,
        sdi_emc2305_fan_speed_get,
        sdi_emc2305_fan_speed_set,
        sdi_emc2305_fan_status_get
};

/*
 * Every driver must export function with name sdi_<driver_name>_query_callbacks
 * so that the driver framework is able to look up and invoke it to get the callbacks
 */
const sdi_driver_t * sdi_emc2305_entry_callbacks(void)
{
    /* Export the Driver table */
    static const sdi_driver_t emc2305_entry = {
        sdi_emc2305_register,
        sdi_emc2305_chip_init
    };
    return &emc2305_entry;
}

/*
 * Update the database for a specific fan.
 * cur_node[in]   - node to be initialised.
 * device_hdl[in] - device handle of emc2305 device
 * Return - None
 */
static void sdi_emc2305_device_database_init(std_config_node_t cur_node,
                                             void* device_hdl)
{
    uint8_t buf = 0;
    uint_t fan_id = 0;
    char *node_attr = NULL;
    t_std_error rc = STD_ERR_OK;
    sdi_device_hdl_t chip = NULL;
    emc2305_device_t *emc2305_data = NULL;

    STD_ASSERT(device_hdl != NULL);

    chip = (sdi_device_hdl_t)device_hdl;
    STD_ASSERT(chip != NULL);

    emc2305_data = (emc2305_device_t*)chip->private_data;
    STD_ASSERT(emc2305_data != NULL);

    if (strncmp(std_config_name_get(cur_node), SDI_DEV_NODE_FAN,
                                               strlen(SDI_DEV_NODE_FAN)) != 0) {
        return;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_INSTANCE);
    if(node_attr != NULL) {
        fan_id = (uint_t) strtoul(node_attr, NULL, 0);
    }
    STD_ASSERT(fan_id < EMC2305_MAX_FANS);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_FAN_SPEED);
    STD_ASSERT(node_attr != NULL);
    emc2305_data->emc2305_fan[fan_id].max_speed = (uint_t)strtoul(node_attr,
                                                                  NULL, 0);

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_FAN_POLES);
    if (node_attr != NULL) {
        emc2305_data->emc2305_fan[fan_id].poles = (uint_t)strtoul(node_attr,
                                                                  NULL, 0);
    /*
     * @todo: Rangex value has to be updated as per the speed value.
     * Still discussions are in progress with hardware team.  Once get the
     * inputs will modify the rangex value.
     * Note: Default rangex value is used
     */
    emc2305_data->emc2305_fan[fan_id].ranges = EMC2305_FAN_RANGEX_DEFAULT;
    } else {
        /* Read the default pole from the register */
        rc = sdi_smbus_read_byte(chip->bus_hdl, chip->addr.i2c_addr,
                                 fan_config1_reg[fan_id], &buf, SDI_I2C_FLAG_NONE);
        if(rc != STD_ERR_OK) {
           SDI_DEVICE_ERRMSG_LOG("%s:read failure at addr:0x%x reg:%d rc:%d",
                                 __FUNCTION__, chip->addr.i2c_addr,
                                 fan_config1_reg[fan_id], rc);
           return;
        }
        emc2305_data->emc2305_fan[fan_id].poles = sdi_fan_poles_get(buf);
        emc2305_data->emc2305_fan[fan_id].ranges = EMC2305_FAN_RANGEX_DEFAULT;
    }
    /* Find the edges from the pole value */
    if ((emc2305_data->emc2305_fan[fan_id].poles >= EMC2305_FAN_POLE_MIN) &&
        (emc2305_data->emc2305_fan[fan_id].poles <= EMC2305_FAN_POLE_MAX)) {
         emc2305_data->emc2305_fan[fan_id].edges =
                  pole_edge_mapping[emc2305_data->emc2305_fan[fan_id].poles - 1];
    } else {
        SDI_DEVICE_ERRMSG_LOG("Fan-%d Invalid Pole value:%d", fan_id,
                              emc2305_data->emc2305_fan[fan_id].poles);
        return;
    }

    node_attr = std_config_attr_get(cur_node, SDI_DEV_ATTR_ALIAS);
    if(node_attr == NULL) {
        emc2305_data->emc2305_fan[fan_id].alias = calloc(1, SDI_MAX_NAME_LEN);
        STD_ASSERT(emc2305_data->emc2305_fan[fan_id].alias != NULL);
        snprintf(emc2305_data->emc2305_fan[fan_id].alias, SDI_MAX_NAME_LEN,
                                       "emc2305-%d-%d", chip->instance, fan_id);
    } else {
        emc2305_data->emc2305_fan[fan_id].alias = calloc(1, (strlen(node_attr)+1));
        STD_ASSERT(emc2305_data->emc2305_fan[fan_id].alias != NULL);
        safestrncpy(emc2305_data->emc2305_fan[fan_id].alias, node_attr,
                    (strlen(node_attr)+1));
    }

    sdi_resource_add(SDI_RESOURCE_FAN, emc2305_data->emc2305_fan[fan_id].alias,
                     sdi_emc2305_create_resource_hdl(chip, fan_id), &emc2305_fan_resource);
    return;
}

/*
 * The configuration file format for the EMC2305 device node is as follows
 *<emc2305 driver="emc2305" instance="<chip_instance>" addr="<address of the chip>"/>
 *<fan instance="<fan no>" alias="<fan alias>" fan_speed="<fan speed>" poles="<pole>" />
 *<fan instance="<fan no>" alias="<fan alias>" fan_speed="<fan speed>" poles="<pole>" />
 *</emc2305>
 * Mandatory attributes    : instance, addr and fan_speed
 */

/* Registers the fan and it's resources with the SDI framework.
 * Parameters:
 * node[in]        - emc2305 device node from the configuration file
 * bus_handle[in]  - Parent bus handle of the device
 * device_hdl[out] - Pointer to the emc2305 device handle which will get filled.
 * Return          - STD_ERR_OK, this function is kept as non void as the driver
 *                   table function pointer requires it
 */
static t_std_error sdi_emc2305_register(std_config_node_t node, void *bus_handle,
                                        sdi_device_hdl_t* device_hdl)
{
    char *node_attr = NULL;
    sdi_device_hdl_t chip = NULL;
    emc2305_device_t *emc2305_data = NULL;

    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(((sdi_bus_t*)bus_handle)->bus_type == SDI_I2C_BUS);

    chip = calloc(1, sizeof(sdi_device_entry_t));
    STD_ASSERT(chip != NULL);
    emc2305_data = calloc(1, sizeof(emc2305_device_t));
    STD_ASSERT(emc2305_data != NULL);

    chip->bus_hdl = bus_handle;

    /* Get all config attributes */
    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    chip->instance = (uint_t) strtoul(node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(node_attr != NULL);
    chip->addr.i2c_addr = (i2c_addr_t)strtoul(node_attr, NULL, 16);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_FAN_CONTROL_TYPE);
    if ((node_attr != NULL) && (strcmp(node_attr, "RPM") == 0)) {
        emc2305_data->fan_control_type = EMC2305_FAN_CONTROL_RPM;
    } else {
        emc2305_data->fan_control_type =  EMC2305_FAN_CONTROL_DIRECT;
    }

    chip->callbacks = sdi_emc2305_entry_callbacks();
    chip->private_data = (void*)emc2305_data;

    std_config_for_each_node(node, sdi_emc2305_device_database_init, chip);

    *device_hdl = chip;

    return STD_ERR_OK;
}

/*
 * Does the chip level initialization for the emc2305 device.
 * device_hdl[in] - device handle of the specific device
 * Return         - STD_ERR_OK for success or the respective error code from
 *                   i2c API in case of failure
 */
static t_std_error sdi_emc2305_chip_init(sdi_device_hdl_t device_hdl)
{
    t_std_error rc = STD_ERR_OK;

    STD_ASSERT(device_hdl != NULL);

    /* Clear fan fault status */
    rc = sdi_emc2305_clear_fan_fault(device_hdl);
    if (rc != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("%s:Fan Fault Status clear failed rc=%d",
                              __FUNCTION__, rc);
    }
    return rc;
}
