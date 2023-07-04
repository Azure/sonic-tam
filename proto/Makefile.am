
%.pb.cc %.pb.h: %.proto
	$(PROTOC) --proto_path=$(srcdir) -I /usr/include --cpp_out=$(srcdir) $^

dist_noinst_DATA = sai_tam_main.proto sai_tam_buffer_stats.proto sai_tam_event_threshold_breach.proto sai_tam_event.proto sai_tam_event_flow_learning.proto

nodist_proto_SOURCES = $(srcdir)/proto/sai_tam_main.pb.cc  $(srcdir)/proto/sai_tam_buffer_stats.pb.cc $(srcdir)/proto/sai_tam_event_threshold_breach.pb.cc $(srcdir)/proto/sai_tam_event.pb.cc $(srcdir)/proto/sai_tam_event_flow_learning.pb.cc

BUILT_SOURCES = sai_tam_main.pb.h sai_tam_buffer_stats.pb.h sai_tam_event_threshold_breach.pb.h sai_tam_event.pb.h sai_tam_event_flow_learning.pb.h

MOSTLYCLEANFILES = sai_tam_main.pb.h sai_tam_main.pb.cc sai_tam_buffer_stats.pb.h sai_tam_buffer_stats.pb.cc sai_tam_event_threshold_breach.pb.h sai_tam_event_threshold_breach.pb.cc sai_tam_event.pb.h sai_tam_event.pb.cc sai_tam_event_flow_learning.pb.h sai_tam_event_flow_learning.pb.cc
