sonic-sdi-device-drivers
------------------------
This repo contains SDI device drivers for the SONiC project.

Description
-----------
This repo creates library libsonic_sdi_device_drivers.so that is loaded by the sonic-sdi-sys repo.  

This repository holds the drivers for various chips used on Dell hardware. The combination of the sonic-sdi-sys, sonic-sdi-framework and sonic-sdi-device-drivers provides a full implementation of the sonic-sdi-api.

Building
--------
Please see the instructions in the sonic-nas-manifest repo for more details on the common build tools.  [Sonic-nas-manifest](https://github.com/Azure/sonic-nas-manifest)

Development Dependencies:

 - sonic-logging
 - sonic-common-utils
 - sonic-sdi-api
 - sonic-sdi-framework
 - sonic-sdi-sys

Dependent Packages:

 - libsonic-logging1 libsonic-logging-dev libsonic-common1 libsonic-common-dev libsonic-sdi-framework1 libsonic-sdi-framework-dev libsonic-sdi-sys1 libsonic-sdi-sys-dev sonic-sdi-api-dev


BUILD CMD: sonic_build --dpkg libsonic-logging1 libsonic-logging-dev libsonic-common1 libsonic-common-dev libsonic-sdi-framework1 libsonic-sdi-framework-dev libsonic-sdi-sys1 libsonic-sdi-sys-dev sonic-sdi-api-dev -- clean binary


(c) Dell 2016

