INCLUDES = -I $(top_srcdir) -I $(top_srcdir)/tam -I $(top_srcdir)/proto
CFLAGS_SWSS = -I /usr/include/swss

bin_PROGRAMS = thresholdmgr test_proto

if DEBUG
DBGFLAGS = -ggdb -DDEBUG
else
DBGFLAGS = -g
endif

thresholdmgr_SOURCES = thresholdmgr_main.cpp thresholdmgr.cpp $(top_srcdir)/proto/sai_tam_buffer_stats.pb.cc $(top_srcdir)/proto/sai_tam_event_threshold_breach.pb.cc $(top_srcdir)/proto/sai_tam_main.pb.cc $(top_srcdir)/proto/sai_tam_event.pb.cc $(top_srcdir)/proto/sai_tam_event_flow_learning.pb.cc
thresholdmgr_CFLAGS = $(DBGFLAGS) $(AM_CFLAGS) $(CFLAGS_COMMON) $(CFLAGS_SWSS) $(COV_CFLAGS)
thresholdmgr_CPPFLAGS = $(DBGFLAGS) $(AM_CFLAGS) $(CFLAGS_COMMON) $(CFLAGS_SWSS) $(COV_CFLAGS)
thresholdmgr_LDADD = -lprotobuf -lswsscommon -lnl-3 -lnl-route-3 -lhiredis $(top_srcdir)/tam/libtamcommon.a $(COV_LDFLAGS)

test_proto_SOURCES = test_proto.cpp $(top_srcdir)/proto/sai_tam_buffer_stats.pb.cc $(top_srcdir)/proto/sai_tam_event_threshold_breach.pb.cc $(top_srcdir)/proto/sai_tam_main.pb.cc $(top_srcdir)/proto/sai_tam_event.pb.cc $(top_srcdir)/proto/sai_tam_event_flow_learning.pb.cc
test_proto_CFLAGS = $(DBGFLAGS) $(AM_CFLAGS) $(CFLAGS_COMMON) $(CFLAGS_SWSS)
test_proto_CPPFLAGS = $(DBGFLAGS) $(AM_CFLAGS) $(CFLAGS_COMMON) $(CFLAGS_SWSS)
test_proto_LDADD = -lprotobuf 
