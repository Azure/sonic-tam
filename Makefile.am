

#The sdi-device_drivers library
lib_LTLIBRARIES = libsonic_sdi_device_drivers.la

libsonic_sdi_device_drivers_la_SOURCES = src/sdi_eeprom.c src/sdi_onie_eeprom.c src/sdi_dell_eeprom.c \
                                         src/sdi_cpld.c src/sdi_cpld_pin.c src/sdi_cpld_pin_group.c \
                                         src/sdi_io_bus.c src/sdi_i2cmux_pin.c src/sdi_tmp75.c src/sdi_emc142x.c \
                                         src/sdi_max6699.c src/sdi_max6620.c src/sdi_emc2305.c src/sdi_mono_color_pin_led.c \
                                         src/sdi_seven_segment_pin_led.c src/sdi_pmbus_dev.c src/sdi_s6k_psu.c src/sdi_pseudo_bus.c \
                                         src/sdi_qsfp.c src/sdi_qsfp_eeprom.c src/sdi_sfp.c src/sdi_sfp_eeprom.c \
                                         src/sys-interface-drivers/sdi_i2cdev.c src/sys-interface-drivers/sdi_gpio.c \
                                         src/sys-interface-drivers/sdi_sysfs_gpio_helpers.c


libsonic_sdi_device_drivers_la_LDFLAGS = -lsonic_sdi_framework -lsonic_common -lsonic_logging -lpthread -lrt -version-info 1:1:0
libsonic_sdi_device_drivers_la_CFLAGS = -I$(top_srcdir)/sonic -I$(includedir)/sonic


