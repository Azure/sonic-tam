INCLUDES = -I $(top_srcdir)/tam 

noinst_LIBRARIES = libtamcommon.a

if DEBUG
DBGFLAGS = -ggdb -DDEBUG
else
DBGFLAGS = -g -DNDEBUG
endif

libtamcommon_a_SOURCES = tam_socket_interface.cpp tam_redis_adapter.cpp
libtamcommon_a_CPPFLAGS = $(DBGFLAGS) $(AM_CFLAGS) $(CFLAGS_COMMON) $(COV_CFLAGS)
