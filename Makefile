PROGRAMS=
SUBDIRS=src utils
LIBRARIES=
ARCHIVES=
HEADERS=

SYSROOT_PREFIX=../platform-root/opt/ngos
DEVKIT_HEADER=${SYSROOT_PREFIX}/inc
INC_CFLAGS=-I${DEVKIT_HEADER}

DEVKIT_LIB=${SYSROOT_PREFIX}/lib
INSTALL_LIB=${DEVKIT_LIB}

all: install devkit

libsdi_drivers_v2.so: $(wildcard src/*.c)  $(wildcard inc/*.h)  $(wildcard src/sys-interface-drivers/*.c)
	gcc -shared -m64 -fPIC $(wildcard src/*.c)  $(wildcard src/sys-interface-drivers/*.c) $(INC_CFLAGS) -Iinc/ -L$(DEVKIT_LIB) -o $@ -lsdi_framework -levent_log -ldn_common  -z defs -lopen_common -lm

LIB_PATH=.

install: libsdi_drivers_v2.so
	cp $< $(INSTALL_LIB)
	cp conf/*.xml ${SYSROOT_PREFIX}/etc/sdi/
	chmod 777 ${SYSROOT_PREFIX}/etc/sdi/*.xml

devkit: libsdi_drivers_v2.so
	cp libsdi_drivers_v2.so $(DEVKIT_LIB)

.PHONY: all clean devkit install
