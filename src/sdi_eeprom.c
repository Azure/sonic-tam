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
 * filename: sdi_eeprom.c
 */


/******************************************************************************
 *  Driver implementation for EEPROM which includes System eeprom, PSU eeprom,
 *  FAN eeprom. Also Driver registration and initialisation.
 *  It supports different types of EEPROM formats as well.
 *
 *  This driver supports eeproms which are compatible with A24xx variants like
 *
 *  AT24C32 32K (4096 x 8bits)
 *  AT24C64 64K (8291 x 8bits)
 ******************************************************************************/

#include "sdi_driver_internal.h"
#include "sdi_resource_internal.h"
#include "sdi_device_common.h"
#include "sdi_common_attr.h"
#include "sdi_eeprom.h"
#include "sdi_onie_eeprom.h"
#include "sdi_dell_eeprom.h"
#include "sdi_i2c_bus_api.h"
#include "std_assert.h"
#include "sdi_bus_framework.h"
#include "sdi_entity_info_resource_attr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


static t_std_error sdi_eeprom_register(std_config_node_t node, void *bus_handle,
                                       sdi_device_hdl_t* device_hdl);
static t_std_error sdi_eeprom_device_init(sdi_device_hdl_t device_hdl);

entity_info_t eeprom_onie_fan_callbacks = {
        NULL, /**< eeprom init is done in the device itself */
        sdi_onie_fan_eeprom_data_get,
};

entity_info_t eeprom_onie_psu_callbacks = {
        NULL, /**< eeprom init is done in the device itself */
        sdi_onie_psu_eeprom_data_get,
};

entity_info_t eeprom_onie_syseeprom_callbacks = {
        NULL, /**< eeprom init is done in the device itself */
        sdi_onie_sys_eeprom_data_get,
};

entity_info_t eeprom_dell_legacy_fan_callbacks = {
        NULL, /**< eeprom init is done in the device itself */
        sdi_dell_legacy_fan_eeprom_data_get,
};

entity_info_t eeprom_dell_legacy_psu_callbacks = {
        NULL, /**< eeprom init is done in the device itself */
        sdi_dell_legacy_psu_eeprom_data_get,
};

entity_info_t eeprom_delta_psu_callbacks = {
        NULL, /**< eeprom init is done in the device itself */
        sdi_delta_psu_eeprom_data_get,
};

/* Export the Driver table */
sdi_driver_t eeprom_entry = {
        sdi_eeprom_register,
        sdi_eeprom_device_init
};


/**
 * Read from specific device offset for devices
 * hdl - Handle to the device on from which data has to be read
 * offset - Offset within the device from which data has to be read
 * data - The data stream read from the device would be returned
 *        in this.It should be pre-allocated by caller to hold len bytes.
 * len - Length of data to be read
 * flags - Miscellaneous flags based on device class. For current implementation
 *          this is ignored.
 * Returns - error code encoded in standard t_std_error format.
 */
static t_std_error sdi_eeprom_read(const struct sdi_device_entry *hdl, uint_t offset,
        uint8_t *data, uint_t len, uint flags)
{
    entity_info_device_t *eeprom_data = NULL;
    uint_t counter=0;
    t_std_error error=STD_ERR_OK;

    if ((hdl == NULL) || (data == NULL))
    {
        return SDI_ERRCODE(EINVAL);
    }

    eeprom_data = hdl->private_data;
    if (offset >= (eeprom_data->entity_size))
    {
        return SDI_ERRCODE(EINVAL);
    }

    switch (eeprom_data->entity_size)
    {
        case 8192: /* 64K bits*/
            /** These devices need 64bit offset which is not possible with
             * native SMBus API.
             *
             * However, AT24C64 datasheet states that this can be circumvented because
             * that chip supports.
             * (i) Dummy Write.  Dummy write is an operation in which the offset
             *     to write is specified but not the actual data.  This is an
             *     indication to instruct the chip to internally update itself
             *     to prepare to read/write from that address in subsequent command.
             * (ii) Sequential Read: Sequential read relies uses the chips internal
             *     offset(location) where the last operation was carried out and
             *     continues from there.
             *     So, if a previous command read from offset-x, then at end of the
             *     operation internally offset is at x+1.
             *     The next read/write would therefore be carried out for x+1 location.
             *
             * As per datasheet,
             * "A random read requires a “dummy” byte write sequence to load
             * in the data word address. Once the device address word and data
             * word address are clocked in and acknowledged by the EEPROM, the
             * microcontroller must generate another start condition. The
             * microcontroller now initiates a current address read by sending a
             * device address with the read/write select bit high. The EEPROM
             * acknowledges the device address and serially clocks out the data
             * word.  * The microcontroller does not respond with a zero but does
             * generate a following stop condition"
             *
             * Dummy Write two can be simulated by using SMBus API by translating
             * the byte sequence as follows since the waveform remains unaffected
             * thought the semantic interprettation is different. Device Command
             *
             * Sequence: ADDRESS, Offset_high, Offset_low
             * SMBus Simulation       : ADDRESS, Offset, Data (SMBus Write Byte Protocol)
             * where Offset=Offset_high
             *       Data = Offset_low
             *
             */

            for (counter=0; ((error == STD_ERR_OK) && (counter<len)) ; counter++)
            {
                /* Dummy Write Operation to set the desired Offset in EEPROM
                 * device
                 * TODO: As per datasheet it suffices to do this once per
                 *    cycle of iteration.  However this does not appear to work.
                 *    Hence writing the offset at start of every read cycle
                 *    till clarification is obtained from vendor.
                 */
                error=sdi_smbus_write_byte(hdl->bus_hdl, hdl->addr.i2c_addr,
                        0,(offset+counter)&0xff,SDI_I2C_FLAG_NONE);
                if (error)
                {
                    break;
                }

                /* Now do the actual read from Device, but remember not to
                 * send Offset, else chip will get confused */
                error=sdi_smbus_recv_byte(hdl->bus_hdl, hdl->addr.i2c_addr,
                        &data[counter], SDI_I2C_FLAG_NONE);
            }
            break;

        case 256:
            return sdi_smbus_read_multi_byte(hdl->bus_hdl, hdl->addr.i2c_addr,
                    offset, data, len, SDI_I2C_FLAG_NONE);

        default:
            return SDI_DEVICE_ERRCODE(EINVAL);
    }
    return error;
}

/**
 * The config file format will be as below for eeprom devices
 *
 *  <eeprom instance="<chip_instance>"
 *    alias="<Alias name for the particular device>">
 *     addr="<Address of the device>"
 *     size="<size of the eeprom device>"
 *   parser="<identify type of device and its format>"
 *  </eeprom>
 *
 * Note: parser and size is the mandatory attribute here.
 */
static t_std_error sdi_eeprom_register(std_config_node_t node, void *bus_handle,
                                       sdi_device_hdl_t* device_hdl)
{
    char *attr_value = NULL;
    sdi_device_hdl_t chip = NULL;
    entity_info_device_t *eeprom_data = NULL;

    /** Validate arguments */
    STD_ASSERT(node != NULL);
    STD_ASSERT(bus_handle != NULL);
    STD_ASSERT(device_hdl != NULL);
    STD_ASSERT(sdi_is_i2c_bus(bus_handle));

    chip = calloc(sizeof(sdi_device_entry_t),1);
    STD_ASSERT(chip != NULL);
    eeprom_data = calloc(sizeof(entity_info_device_t),1);
    STD_ASSERT(eeprom_data != NULL);

    chip->bus_hdl = bus_handle;

    /* Get all config attributes */
    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(attr_value != NULL);
    chip->instance = strtoul(attr_value, NULL, 0);

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_ADDRESS);
    STD_ASSERT(attr_value != NULL);
    chip->addr.i2c_addr = (i2c_addr_t) strtoul(attr_value, NULL, 16);

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_ALIAS);
    if (attr_value == NULL) {
        snprintf(chip->alias, SDI_MAX_NAME_LEN, "eeprom-%d", chip->instance );
    } else {
        strncpy(chip->alias, attr_value, SDI_MAX_NAME_LEN);
    }

    chip->callbacks = &eeprom_entry;
    chip->private_data = (void*)eeprom_data;

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_ENTITY_SIZE);
    if(attr_value) {
        eeprom_data->entity_size = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_NO_OF_FANS);
    if(attr_value) {
        eeprom_data->no_of_fans = strtoul(attr_value, NULL, 0);
    }

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_MAX_SPEED);
    if(attr_value) {
        eeprom_data->max_fan_speed = strtoul(attr_value, NULL, 0);
    }

    chip->sdi_device_read_fn=sdi_eeprom_read;

    attr_value = std_config_attr_get(node, SDI_DEV_ATTR_PARSER);
    STD_ASSERT(attr_value!=NULL);

    if (strncmp(attr_value, SDI_STR_ONIE_SYS_EEPROM,
                            strlen(SDI_STR_ONIE_SYS_EEPROM)) == 0) {
            sdi_resource_add(SDI_RESOURCE_ENTITY_INFO, chip->alias,(void*)chip,
                            &eeprom_onie_syseeprom_callbacks);
    } else if (strncmp(attr_value, SDI_STR_ONIE_PSU_EEPROM,
                            strlen(SDI_STR_ONIE_PSU_EEPROM)) == 0) {
            sdi_resource_add(SDI_RESOURCE_ENTITY_INFO, chip->alias,(void*)chip,
                            &eeprom_onie_psu_callbacks);

    } else if (strncmp(attr_value, SDI_STR_ONIE_FAN_EEPROM,
                            strlen(SDI_STR_ONIE_FAN_EEPROM)) == 0 ){
            sdi_resource_add(SDI_RESOURCE_ENTITY_INFO, chip->alias,(void*)chip,
                            &eeprom_onie_fan_callbacks);
    } else if (strncmp(attr_value, SDI_STR_DELL_LEGACY_PSU_EEPROM,
                            strlen(SDI_STR_DELL_LEGACY_PSU_EEPROM)) == 0) {
            sdi_resource_add(SDI_RESOURCE_ENTITY_INFO, chip->alias,(void*)chip,
                            &eeprom_dell_legacy_psu_callbacks);
    } else if (strncmp(attr_value, SDI_STR_DELTA_PSU_EEPROM,
                            strlen(SDI_STR_DELTA_PSU_EEPROM)) == 0 ){
            sdi_resource_add(SDI_RESOURCE_ENTITY_INFO, chip->alias,(void*)chip,
                            &eeprom_delta_psu_callbacks);
    } else if (strncmp(attr_value, SDI_STR_DELL_LEGACY_FAN_EEPROM,
                            strlen(SDI_STR_DELL_LEGACY_FAN_EEPROM)) == 0 ){
            sdi_resource_add(SDI_RESOURCE_ENTITY_INFO, chip->alias,(void*)chip,
                            &eeprom_dell_legacy_fan_callbacks);
    } else {
            /* Assert, when unsupported parser is received from config */
            STD_ASSERT(false);
    }

    *device_hdl = chip;

    return STD_ERR_OK;
}

/**
 * Does the initialization for the eeprom device
 * param[in] device_hdl - device handle of the specific device
 * return: STD_ERR_OK
 */
static t_std_error sdi_eeprom_device_init(sdi_device_hdl_t device_hdl)
{
    t_std_error rc = STD_ERR_OK;
    entity_info_device_t *eeprom_data = NULL;

    STD_ASSERT(device_hdl != NULL);

    eeprom_data = (entity_info_device_t*)device_hdl->private_data;
    STD_ASSERT(eeprom_data != NULL);

    /** todo: Need to do the EEPROM data caching, if require. Still in
     * discussion */

    return rc;
}

