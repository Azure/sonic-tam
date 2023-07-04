INCLUDES = -I $(top_srcdir)/tam -I $(top_srcdir)/proto

noinst_LIBRARIES = libtambuffers.a

if DEBUG
DBGFLAGS = -ggdb -DDEBUG
else
DBGFLAGS = -g -DNDEBUG
endif

libtambuffers_a_SOURCES = tam_socket_interface.cpp tam_redis_adapter.cpp tam_proto_buffer_stats_decoder.cpp
libtambuffers_a_CPPFLAGS = $(DBGFLAGS) $(AM_CFLAGS) $(CFLAGS_COMMON) $(COV_CFLAGS) $(ASAN_CFLAGS)
