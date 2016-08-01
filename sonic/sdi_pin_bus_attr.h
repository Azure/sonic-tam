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
 * filename: sdi_pin_bus_attr.h
 */


#ifndef __SDI_PIN_BUS_ATTR_H__
#define __SDI_PIN_BUS_ATTR_H__

#include <string.h>

/**
 * @file sdi_pin_bus_attr.h
 * @brief defines list of attributes supported for every pin bus node
 * configuration
 *
 * @defgroup sdi_config_pin_bus_attr SDI pin bus node configuration attributes
 * @ingroup sdi_config_attributes
 *
 * @{
 */

/**
 * @def Attribute used for representing the pin number
 */
#define SDI_DEV_ATTR_PIN_NUMBER            "pin"
/**
 * @def Attribute used for representing the pin group
 */
#define SDI_DEV_ATTR_PIN_GROUP             "pingroup"
/**
 * @def Attribute used for representing the pin direction
 */
#define SDI_DEV_ATTR_PIN_DIRECTION         "direction"
/**
 * @def Attribute used for representing the pin polarity
 */
#define SDI_DEV_ATTR_PIN_POLARITY          "polarity"
/**
 * @def Attribute used for representing the pin level
 */
#define SDI_DEV_ATTR_PIN_LEVEL             "level"
/**
 * @def Attribute used for representing the input pin
 */
#define SDI_DEV_ATTR_INPUT_PIN             "in"
/**
 * @def Attribute used for representing the input pin string length
 */
#define SDI_DEV_ATTR_INPUT_PIN_LEN         (strlen(SDI_DEV_ATTR_INPUT_PIN))
/**
 * @def Attribute used for representing the output pin
 */
#define SDI_DEV_ATTR_OUTPUT_PIN            "out"
/**
 * @def Attribute used for representing the output pin string length
 */
#define SDI_DEV_ATTR_OUTPUT_PIN_LEN        (strlen(SDI_DEV_ATTR_OUTPUT_PIN))
/**
 * @def Attribute used for representing the pin with normal polarity
 */
#define SDI_DEV_ATTR_POLARITY_NORMAL       "normal"
/**
 * @def Attribute used for representing the pin with normal polarity string
 * length
 */
#define SDI_DEV_ATTR_POLARITY_NORMAL_LEN    \
                (strlen(SDI_DEV_ATTR_POLARITY_NORMAL))
/**
 * @def Attribute used for representing the inverted pin representation
 */
#define SDI_DEV_ATTR_POLARITY_INVERTED     "inverted"
/**
 * @def Attribute used for representing the pin with inverted polarity string
 * length
 */
#define SDI_DEV_ATTR_POLARITY_INVERTED_LEN    \
                (strlen(SDI_DEV_ATTR_POLARITY_INVERTED))

/**
 * @}
 */

#endif   /* __SDI_PIN_BUS_ATTR_H__ */
