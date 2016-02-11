ROGRAMS=
SUBDIRS=
LIBRARIES=
ARCHIVES=
HEADERS=

# sdi_drivers_v2 - Library that implements SDI Driver as per new implementation plan
LIBRARIES+= sdi_drivers_v2
sdi_drivers_v2_SRCS= sdi_eeprom.c sdi_onie_eeprom.c sdi_dell_eeprom.c
sdi_drivers_v2_SRCS+=sys-interface-drivers/sdi_i2cdev.c
sdi_drivers_v2_SRCS+=sys-interface-drivers/sdi_gpio.c sys-interface-drivers/sdi_sysfs_gpio_helpers.c
sdi_drivers_v2_SRCS+=sdi_cpld.c sdi_cpld_pin.c sdi_cpld_pin_group.c
sdi_drivers_v2_SRCS+=sdi_io_bus.c
sdi_drivers_v2_SRCS+=sdi_i2cmux_pin.c
sdi_drivers_v2_SRCS+=sdi_tmp75.c
sdi_drivers_v2_SRCS+=sdi_emc142x.c sdi_max6699.c
sdi_drivers_v2_SRCS+=sdi_max6620.c
sdi_drivers_v2_SRCS+=sdi_emc2305.c
sdi_drivers_v2_SRCS+=sdi_mono_color_pin_led.c sdi_seven_segment_pin_led.c
sdi_drivers_v2_SRCS+=sdi_pmbus_dev.c sdi_s6k_psu.c
sdi_drivers_v2_SRCS+=sdi_pseudo_bus.c
sdi_drivers_v2_SRCS+=sdi_qsfp.c sdi_qsfp_eeprom.c
sdi_drivers_v2_SRCS+=sdi_sfp.c sdi_sfp_eeprom.c
sdi_drivers_v2_LDFLAGS= -z defs -lopen_common -lsdi_framework -levent_log -ldn_common
sdi_drivers_v2_CFLAGS= -Wall -Werror

include ${MAKE_INC}/workspace.mak
