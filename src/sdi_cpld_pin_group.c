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
 * filename: sdi_cpld_pin_group.c
 */


/******************************************************************************
 * Refer to sdi_cpld.c for documentation on cpld pin group
 *
 * xml file format:
 *      <sdi_cpld_pin_group instance="1" start_addr="0x7" start_offset="5"
 *            end_offset="8"  bus_name="board_revision" direction="in"
 *            polarity="normal">
 *      </sdi_cpld_pin_group>
 *        <!-- start_addr is cpld pin group's register's start address --!>
 *        <!- start_offset is cpld pin group's bit offset within start_addr --!>
 *        <!-- end_addr is cpld pin group's register's end address --!>
 *        <!- end_offset is cpld pin group's bit offset within end_addr --!>
 *****************************************************************************/

#include "sdi_cpld.h"
#include "sdi_device_common.h"
#include "sdi_bus_framework.h"
#include "sdi_bus_api.h"
#include "sdi_pin_bus_framework.h"
#include "sdi_pin_group_bus_framework.h"
#include "std_assert.h"
#include "sdi_pin_bus_attr.h"
#include "sdi_bus_attr.h"
#include "sdi_common_attr.h"
#include "std_mutex_lock.h"
#include "std_utils.h"
#include "sdi_cpld_attr.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Mask given 'bitfield' with bits from 'start_offset' to 'end_offset' */
#define SDI_CPLD_BIT_MASK(bitfield, start_offset, end_offset) \
    bitfield &= ((1 << ((end_offset - start_offset) + 1)) - 1);

/* Toggle 'dst_target' bitstream of given 'type' and mask with bits from
 * 'start_bit_offset' and 'end_bit_offset'
 */
#define SDI_CPLD_TOGGLE_AND_MASK(type, dst_target, start_bit_offset, end_bit_offset) \
do { \
    dst_target = (((type)~dst_target) & ((1 << ((end_bit_offset - start_bit_offset) + 1)) - 1)); \
} while (0)

/*
 * Read configured pin group level.
 * sequence of operation:
 * 1. set dec_counter_flag to 1 if start_addr of cpld pin group is greater than
 * end_addr. Set offset to start_addr
 * 2. Read cpld register at'offset'u'sing bus read api (starting from start_addr) onto buffer
 * 3. Read the register value with bits from : start_offset (applicable only while
 * reading start_addr, to end_offset (applicable only while reading end_addr or
 * while reading start_addr and the number of registers in pin group is 1) by masking
 * other bits
 * data = buffer with just bits from (start_offset to end offset)
 * 4. If the polarity is inverted, valid data is inverted.
 * 5. If the cpld register is not start_addr, left shift data by 8bits.
 * 5. Update level with data.
 * level |= data
 * 7. Increment offset if dec_counter_flag is not set, otherwise decrement offset.
 * 6. Repeat steps from 2 to 5 until end_addr of cpld pin group is read.
 * param[in] pin_group_hdl - pin group bus handle
 * param[in] value - configured pin group level
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO no failure
 */
static t_std_error sdi_cpld_pin_group_read_level (sdi_pin_group_bus_hdl_t pin_group_hdl,
                                                  uint_t *value)
{
    sdi_cpld_pin_group_t *cpld_pin_group = (sdi_cpld_pin_group_t *) pin_group_hdl;
    sdi_device_hdl_t dev_hdl = cpld_pin_group->cpld_hdl;
    sdi_bus_hdl_t bus_hdl = (sdi_bus_hdl_t) dev_hdl->bus_hdl;
    uint8_t buffer = 0;
    uint8_t data = 0;
    t_std_error error = STD_ERR_OK;
    uint_t level = 0;
    uint_t offset = 0;
    uint_t reg_count = 0;
    uint_t dec_counter_flag = 0;
    uint_t start_offset = cpld_pin_group->start_offset;
    uint_t end_offset = cpld_pin_group->end_offset;
    sdi_cpld_dev_hdl_t cpld_dev_hdl = (sdi_cpld_dev_hdl_t) dev_hdl->private_data;

    STD_ASSERT(cpld_dev_hdl->width == SDI_CPLD_DEFAULT_REGISTER_WIDTH);

    /* CPLD pin group can span across more than one cpld register in which case:
     * start_addr marks Least Significant Byte register address in pin group and
     * end_addr marks Most Significant Byte) register address in pin group.
     * If end_addr > start_addr, value read from cpld pin group will reflect below:
     *  ---------------------------------------------------------
     *  | end_addr | start_addr + 2 | start_addr + 1 | start_addr |
     *  |          |                |                |            |
     *  |   byte4  |     byte3      |     byte2      |   byte1    |
     *  ---------------------------------------------------------
     * or
     * If end_addr < start_addr, value read from cpld pin group will reflect below:
     *  ---------------------------------------------------------
     *  | end_addr | start_addr - 2 | start_addr - 1 | start_addr |
     *  |          |                |                |            |
     *  |   byte4  |     byte3      |     byte2      |   byte1    |
     *  ---------------------------------------------------------
     * Main Logic:
     * Read byte1 from cpld using bus read api and store it in data.
     * Update level = data.
     * Read byte2 from cpld using bus read api and store it in data.
     * Update level |= data << 8
     * Repeat above until all 4 bytes are read.
     */


    /* Set offset to start_addr. */
    offset = cpld_pin_group->start_addr;
    /* Depending on platform configuration, start_addr can be greater or lesser
     * than end_addr in pin group.
     * if end_addr > start_addr, above offset needs to be decremented in below loop
     * otherwise offset needs to be incremented
     */
    if (cpld_pin_group->start_addr >= cpld_pin_group->end_addr) {
        dec_counter_flag = 1;
    }

    /* Loop: until we read all cpld registers (indicated by length) in pin group */
    for (reg_count = 1; reg_count <= cpld_pin_group->length; reg_count++) {
        /* Read the cpld register (that's part of pin group) value to buffer */
        error = sdi_bus_read_byte(bus_hdl, dev_hdl->addr, offset, &buffer);
        if (error != STD_ERR_OK) {
            return error;
        }
        /* case a) For first and only cpld register(start_addr: byte1 as in figure) in pin group
         * (first and last are same), mask buffer with start_offset to end_offset and
         * store it in data. If polarity is inverted, toggle the data bits (only
         * between start_offset and end_offset */
        if (cpld_pin_group->length == 1) {
            data = (buffer >> (cpld_pin_group->start_offset));
            SDI_CPLD_BIT_MASK(data, start_offset, end_offset);
            if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                SDI_CPLD_TOGGLE_AND_MASK(uint_t, data, start_offset, end_offset);
            }
            /* Store the cpld register onto level*/
            level = data;
        } else {
            if (reg_count == cpld_pin_group->length) {
                /* case c) For last cpld register in pin group(end_addr: byte4),
                 * mask buffer with start_offset to cpld register bit length
                 * (BITS_PER_BYTE) and store it in data.If polarity is inverted,
                 * toggle the data bits (only between start_offset and
                 * BITS_PER_BYTE.*/
                data = buffer & ( (1 << (cpld_pin_group->end_offset + 1)) - 1 );
                if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                    SDI_CPLD_TOGGLE_AND_MASK(uint_t, data, 0, end_offset);
                }
                /* Left Shift the data to last byte position and Accumulate the
                 * result in level */
                level |= (data << ((reg_count - 1) * BITS_PER_BYTE));

                /* Shift the accumulated register value by start_offset */
                level = level >> start_offset;
            } else if (reg_count == 1) {
                /* case b) For first cpld register in pin group(start_addr:
                 * byte1), mask buffer with start_offset to cpld register bit
                 * length (BITS_PER_BYTE) and store it in data. If polarity is
                 * inverted, toggle the data bits (only between start_offset
                 * and BITS_PER_BYTE. */
                data = buffer;
                if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                    SDI_CPLD_TOGGLE_AND_MASK(uint_t, data, start_offset, (BITS_PER_BYTE -1));
                }
                /* Store data to Level */
                level = data;
            } else  {
                /* case d) For the cpld register (!first and !last) in pin
                 * group(byte2 and byte3), set data to buffer. If polarity is
                 * inverted, toggle the data bits. */
                data = buffer;
                if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                    SDI_CPLD_TOGGLE_AND_MASK(uint_t, data, 0, (BITS_PER_BYTE - 1));
                }
                 /*Left shift data based on byte position * 8 and accumulate it to level */
                level |= (data << ((reg_count -1) * BITS_PER_BYTE));
            }
        }
        /* if end_addr > start_addr, offset needs to be decremented
         * otherwise offset needs to be incremented
         */
        if (dec_counter_flag) {
            offset--;
        } else {
            offset++;
        }
    }
    /* Set the cpld pin group register value to value */
    *value = level;
    return error;
}

/*
 * Make 'src_bit_stream' part of 'dst_target' starting from start_bit_offset to
 * end_bit_offset
 * Ex: With dst_target 0xFFFF33FF, src_bit_stream 0x11, start_bit_offset 8,
 * end_bit_offset 16 as inputs, this function will modify dst_target to 0xFFFF11FF
 *
 * @param[in] dst_target : value that has to be modified
 * @param[in] src_bit_stream: value to be set in dst_target between
 * start_bit_offset and end_bit_offset
 * @param[in] start_bit_offset : start offset (counting from 0) at which
 *  src_bit_stream needs to be set in dst_target.
 * @param[in] end_bit_offset : end offset (counting from 0) till which
 * src_bit_stream needs to be set in dst_target.
 * @return dst_target modified with 'src_bit_stream' set between
 * 'start_bit_offset' and 'end_bit_offset'
 */
uint64_t sdi_cpld_bit_set_sub_bitstream(uint64_t dst_target, uint64_t src_bit_stream,
                                        uint64_t start_bit_offset,
                                        uint64_t end_bit_offset)
{
    uint64_t left = 0;
    uint64_t right = 0;
    uint64_t mask = 0;

    STD_ASSERT(start_bit_offset <= (sizeof(uint64_t) * BITS_PER_BYTE));
    STD_ASSERT(end_bit_offset <= (sizeof(uint64_t) * BITS_PER_BYTE));

    left = (((uint64_t)~0) - ((1 << end_bit_offset) - 1));
    right = ((1 << start_bit_offset) - 1);
    mask = (left | right);

    return (dst_target & mask) | (src_bit_stream << start_bit_offset);
}


/*
 * Update the cpld pin group level with given value
 * Sequence of operation is same as in sdi_cpld_pin_group_read_level, except
 * that value to be set on to pin group is additionlly masked as per start_offset
 * and end_offset and the cpld register is updated by using bus write api.
 * param[in] pin_group_hdl - cpld pin group handle
 * param[in] value - level to be configured on cpld pin group
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure,
 * SDI_DEVICE_ERRCODE(ENOTSUP) if un-supported
 */
static t_std_error sdi_cpld_pin_group_write_level (sdi_pin_group_bus_hdl_t pin_group_hdl,
                                                   uint_t value)
{
    sdi_cpld_pin_group_t *cpld_pin_group = (sdi_cpld_pin_group_t *) pin_group_hdl;
    sdi_device_hdl_t dev_hdl = cpld_pin_group->cpld_hdl;
    sdi_bus_hdl_t bus_hdl = (sdi_bus_hdl_t) dev_hdl->bus_hdl;
    uint8_t buffer = 0;
    uint8_t level = 0;
    uint_t data = value;
    t_std_error error = STD_ERR_OK;
    uint_t offset = 0;
    uint_t reg_count = 0;
    uint_t dec_counter_flag = 0;
    uint64_t start_offset = (uint64_t) cpld_pin_group->start_offset;
    uint64_t end_offset = (uint64_t) cpld_pin_group->end_offset;
    sdi_cpld_dev_hdl_t cpld_dev_hdl = (sdi_cpld_dev_hdl_t) dev_hdl->private_data;

    STD_ASSERT(cpld_dev_hdl->width == SDI_CPLD_DEFAULT_REGISTER_WIDTH);

    /* CPLD pin group can span across more than one cpld register in which case:
     * start_addr marks Least Significant Byte register address in pin group and
     * end_addr marks Most Significant Byte) register address in pin group.
     * If end_addr > start_addr, value(byte4 : byte3: byte2: byte1) Written to
     * cpld pin group will reflect below:
     *  ---------------------------------------------------------
     *  | end_addr | start_addr + 2 | start_addr + 1 | start_addr |
     *  |          |                |                |            |
     *  |   byte4  |     byte3      |     byte2      |   byte1    |
     *  ---------------------------------------------------------
     * or
     * If end_addr < start_addr, value read from cpld pin group will reflect below:
     *  ---------------------------------------------------------
     *  | end_addr | start_addr - 2 | start_addr - 1 | start_addr |
     *  |          |                |                |            |
     *  |   byte4  |     byte3      |     byte2      |   byte1    |
     *  ---------------------------------------------------------
     * Main Logic:
     * Set data = value (to be written to cpld pin group)
     * Read a byte from start_addr of cpld using bus read api and store it in
     * buffer. Mask last byte of data and bit-wise & with buffer, write it to
     * cpld start_addr.
     * Read next byte from cpld using bus read api and store it in buffer. Shift
     * data by a byte, mask last byte of data and bit-wise & with buffer, write
     * it to cpld next address.
     * Repeat above until all bytes are updated.
     */

    /* Set offset to start_addr.*/
    offset = cpld_pin_group->start_addr;
    if (cpld_pin_group->start_addr >= cpld_pin_group->end_addr) {
        /* Depending on platform configuration, start_addr can be greater or lesser
         * than end_addr in pin group.
         * if end_addr > start_addr, above offset needs to be decremented in below loop
         * otherwise offset needs to be incremented
         */
        dec_counter_flag = 1;
    }

    /* Loop: until all cpld registers of pin group are written */
    for (reg_count = cpld_pin_group->length; reg_count != 0; reg_count--) {
        /* Read the cpld register at offset into buffer. */
        error = sdi_bus_read_byte(bus_hdl, dev_hdl->addr, offset, &buffer);
        if (error != STD_ERR_OK) {
            return error;
        }
        /* Fetch the value to be written to the cpld register 'offset' */
        level = data & 0xFF;
        /* case a) For first and only cpld register (start_addr: byte1)
         * in pin group, mask last byte of data (level) and bit-wise & with buffer,
         * If polarity is inverted, toggle the last byte of data (level)
         * bits between start_offset and end_offset.
         */
        if (cpld_pin_group->length == 1) {
            if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                SDI_CPLD_TOGGLE_AND_MASK(uint8_t, level, start_offset, end_offset);
            }
            buffer = (uint8_t) sdi_cpld_bit_set_sub_bitstream((uint64_t)buffer, (uint64_t)level, start_offset, (end_offset + 1));
        } else {
            if (reg_count == cpld_pin_group->length) {
                /* case b) For first cpld register (start_addr: byte1)
                 * in pin group, mask last byte of data (level) and bit-wise & with buffer
                 * If polarity is inverted, toggle the last byte of data (level)
                 * bits between start_offset and BITS_PER_BYTE.
                 */
                if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                    SDI_CPLD_TOGGLE_AND_MASK(uint8_t, level, start_offset, (BITS_PER_BYTE - 1));
                }
                buffer = (uint8_t) sdi_cpld_bit_set_sub_bitstream((uint64_t)buffer, (uint64_t)level, start_offset, BITS_PER_BYTE);
            }
            else if (reg_count == 1) {
                /* case c) For last cpld register (end_addr: byte4)
                 * in pin group, mask MSB of data (level) and bit-wise & with buffer
                 * If polarity is inverted, toggle the level from first bit till
                 * end_offset
                 */
                if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                    SDI_CPLD_TOGGLE_AND_MASK(uint8_t, level, 0, end_offset);
                }
                buffer = (uint8_t)
                sdi_cpld_bit_set_sub_bitstream((uint64_t)buffer, (uint64_t)level, 0, (end_offset + 1));
            } else  {
                /* case d) For and !first !last cpld register (say byte2)
                 * in pin group, mask second byte of data (level) and bit-wise & with buffer
                 * If polarity is inverted, toggle all the bits of level
                 */
                if (pin_group_hdl->default_polarity == SDI_PIN_POLARITY_INVERTED) {
                    level = ~level;
                }
                buffer = (uint8_t) sdi_cpld_bit_set_sub_bitstream((uint64_t)buffer, (uint64_t)level, 0, BITS_PER_BYTE);
            }
        }
        /* Write modified value of buffer to cpld offset */
        error = sdi_bus_write_byte(bus_hdl, dev_hdl->addr, offset, buffer);
        if (error != STD_ERR_OK) {
            return error;
        }
        /* Right Shift data by 8bits to fetch next byte of value to be written */
        data = data >> BITS_PER_BYTE;

        /* if end_addr > start_addr, dec_counter_flag needs to be decremented
         * otherwise offset needs to be incremented */
        if (dec_counter_flag) {
            offset--;
        } else {
            offset++;
        }
    }
    return error;
}


/*
 * CPLD pin group direction can't be changed.
 * param[in] pin_group_hdl cpld pin group handle
 * param[in] direction pin group direction to be configured
 * return SDI_DEVICE_ERRCODE(ENOTSUP)
 */
static inline t_std_error sdi_cpld_pin_group_set_direction (sdi_pin_group_bus_hdl_t pin_group_hdl,
                                                     sdi_pin_bus_direction_t direction)
{
    return SDI_DEVICE_ERRCODE(ENOTSUP);
}

/*
 * Get cpld pin group configured direction
 * param[in] pin_group_hdl cpld pin group handle
 * param[out] direction configured pin group direction
 * return STD_ERR_OK on success.
 */
static inline t_std_error sdi_cpld_pin_group_get_direction (sdi_pin_group_bus_hdl_t pin_group_hdl,
                                                     sdi_pin_bus_direction_t *direction)
{
    *direction = pin_group_hdl->default_direction;
    return STD_ERR_OK;
}

/*
 * CPLD pin group polarity can't be changed.
 * param[in] pin_group_hdl cpld pin group handle
 * param[in] polarity pin group polarity to be configured
 * return SDI_DEVICE_ERRCODE(ENOTSUP)
 */
static inline t_std_error sdi_cpld_pin_group_set_polarity (sdi_pin_group_bus_hdl_t pin_group_hdl,
                                                    sdi_pin_bus_polarity_t polarity)
{
    return SDI_DEVICE_ERRCODE(ENOTSUP);
}


/*
 * Get cpld pin group configured polarity
 * param[in] pin_group_hdl cpld pin group handle
 * param[out] polarity configured pin group polarity
 * return STD_ERR_OK on success.
 */
static inline t_std_error sdi_cpld_pin_group_get_polarity (sdi_pin_group_bus_hdl_t pin_group_hdl,
                                                    sdi_pin_bus_polarity_t *polarity)
{
    *polarity = pin_group_hdl->default_polarity;
    return STD_ERR_OK;
}

/*
 * cpld pin group operations object
 */
static sdi_pin_group_bus_ops_t sdi_cpld_pin_group_ops = {
    .sdi_pin_group_bus_read_level = sdi_cpld_pin_group_read_level,
    .sdi_pin_group_bus_write_level = sdi_cpld_pin_group_write_level,
    .sdi_pin_group_bus_set_direction = sdi_cpld_pin_group_set_direction,
    .sdi_pin_group_bus_get_direction = sdi_cpld_pin_group_get_direction,
    .sdi_pin_group_bus_set_polarity = sdi_cpld_pin_group_set_polarity,
    .sdi_pin_group_bus_get_polarity = sdi_cpld_pin_group_get_polarity,
};

/*
 * configure default level for an output cpld pin group
 * param[in] bus_hdl cpld pin group handle
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_cpld_pin_group_driver_init(sdi_bus_hdl_t bus_hdl)
{
    t_std_error error = STD_ERR_OK;
    sdi_pin_group_bus_hdl_t pin_group_hdl = (sdi_pin_group_bus_hdl_t) bus_hdl;

    if (pin_group_hdl->default_direction == SDI_PIN_BUS_OUTPUT) {
        error = sdi_cpld_pin_group_write_level(pin_group_hdl, pin_group_hdl->default_level);
        if (error != STD_ERR_OK) {
            SDI_DEVICE_ERRMSG_LOG("%s:%d write pin group %s failed with error %d\n",
                                  __FUNCTION__, __LINE__,
                                  pin_group_hdl->bus.bus_name, error);
        }
    }
    return error;
}

/*
 * Create cpld pin group and register with pin group bus framework
 * param[in] node - cpld pin group node
 * param[out] bus_hdl - cpld pin group bus handle populated on successful registration
 * return STD_ERR_OK on success, SDI_DEVICE_ERRNO on failure
 */
static t_std_error sdi_cpld_pin_group_driver_register(std_config_node_t node,
                                                      sdi_bus_hdl_t *bus_hdl)
{
    char *node_attr = NULL;
    sdi_pin_group_bus_hdl_t pin_group_bus = NULL;
    t_std_error error = STD_ERR_OK;

    sdi_cpld_pin_group_t *cpld_pin_group = (sdi_cpld_pin_group_t *)
        calloc(sizeof(sdi_cpld_pin_group_t), 1);
    STD_ASSERT(cpld_pin_group != NULL);

    pin_group_bus = &(cpld_pin_group->bus);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_INSTANCE);
    STD_ASSERT(node_attr != NULL);
    pin_group_bus->bus.bus_id = (uint_t) strtoul (node_attr, NULL, 0);

    pin_group_bus->bus.bus_type = SDI_PIN_GROUP_BUS;
    pin_group_bus->bus.bus_init = sdi_cpld_pin_group_driver_init;

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_START_ADDR);
    STD_ASSERT(node_attr != NULL);
    cpld_pin_group->start_addr = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_END_ADDR);
    if (node_attr == NULL) {
        cpld_pin_group->end_addr = cpld_pin_group->start_addr;
        cpld_pin_group->length = 1;
    } else {
        cpld_pin_group->end_addr = (uint_t) strtoul (node_attr, NULL, 0);
        if (cpld_pin_group->start_addr > cpld_pin_group->end_addr) {
            cpld_pin_group->length = (cpld_pin_group->start_addr -
                    cpld_pin_group->end_addr) + 1;
        } else {
            cpld_pin_group->length = (cpld_pin_group->end_addr -
                    cpld_pin_group->start_addr) + 1;
        }
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_START_OFFSET);
    STD_ASSERT(node_attr != NULL);
    cpld_pin_group->start_offset = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_CPLD_END_OFFSET);
    STD_ASSERT(node_attr != NULL);
    cpld_pin_group->end_offset = (uint_t) strtoul (node_attr, NULL, 0);

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_DIRECTION);
    if (node_attr != NULL) {
        if ((strncmp(node_attr, SDI_DEV_ATTR_INPUT_PIN,
                SDI_DEV_ATTR_INPUT_PIN_LEN)) == 0) {
            pin_group_bus->default_direction = SDI_PIN_BUS_INPUT;
        } else if  ((strncmp(node_attr, SDI_DEV_ATTR_OUTPUT_PIN,
                    SDI_DEV_ATTR_OUTPUT_PIN_LEN)) == 0) {
             pin_group_bus->default_direction = SDI_PIN_BUS_OUTPUT;
        } else {
            SDI_DEVICE_ERRMSG_LOG("Pin %u invalid direction\n",
                                  pin_group_bus->bus.bus_id);
            STD_ASSERT(false);
        }
    } else {
        /* If pin direction is not configured, default it to input pin */
        pin_group_bus->default_direction = SDI_PIN_BUS_INPUT;
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_POLARITY);
    if (node_attr != NULL) {
        if ((strncmp(node_attr, SDI_DEV_ATTR_POLARITY_NORMAL,
                SDI_DEV_ATTR_POLARITY_NORMAL_LEN)) == 0) {
            pin_group_bus->default_polarity = SDI_PIN_POLARITY_NORMAL;
        } else if ((strncmp(node_attr, SDI_DEV_ATTR_POLARITY_INVERTED,
                SDI_DEV_ATTR_POLARITY_INVERTED_LEN)) == 0) {
            pin_group_bus->default_polarity = SDI_PIN_POLARITY_INVERTED;
        } else {
            SDI_DEVICE_ERRMSG_LOG("Pin %u invalid polarity\n",
                                  pin_group_bus->bus.bus_id);
            STD_ASSERT(false);
        }
    } else {
        /* If polarity is not configured, default it to normal */
        pin_group_bus->default_polarity = SDI_PIN_POLARITY_NORMAL;
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_PIN_LEVEL);
    if (node_attr != NULL) {
        pin_group_bus->default_level = (int) strtoul (node_attr, NULL, 0);
    } else {
        pin_group_bus->default_level = SDI_PIN_LEVEL_LOW;
    }

    node_attr = std_config_attr_get(node, SDI_DEV_ATTR_BUS_NAME);
    STD_ASSERT(node_attr != NULL);
    safestrncpy(pin_group_bus->bus.bus_name, node_attr, SDI_MAX_NAME_LEN);

    std_mutex_lock_init_non_recursive(&(pin_group_bus->lock));
    pin_group_bus->ops = &sdi_cpld_pin_group_ops;

    error = sdi_pin_group_bus_register((sdi_pin_group_bus_hdl_t )cpld_pin_group);
    if (error != STD_ERR_OK) {
        SDI_DEVICE_ERRMSG_LOG("cpld pin group %u registration failed, err: %d\n",
                              pin_group_bus->bus.bus_id, error);
        std_mutex_destroy(&(pin_group_bus->lock));
        free(cpld_pin_group);
        return error;
    }

    *bus_hdl = (sdi_bus_hdl_t ) cpld_pin_group;

    sdi_bus_register_device_list(node, (sdi_bus_hdl_t) cpld_pin_group);

    return error;
}

/**
 * SDI cpld pin group driver object to hold cpld group registration and
 * initialization function
 * Note:
 * Every bus driver must export function with name
 * sdi_<bus_driver_name>_entry_callbacks
 * so that the driver framework is able to look up and invoke it to get the
 * callbacks
 */
const sdi_bus_driver_t * sdi_cpld_pin_group_entry_callbacks(void)
{
     /*Export Bus Driver table*/
     static const sdi_bus_driver_t sdi_cpld_pin_group_entry = {
        .bus_register = sdi_cpld_pin_group_driver_register,
        .bus_init = sdi_cpld_pin_group_driver_init
     };
     return &sdi_cpld_pin_group_entry;
}
