#include <iostream>
#include <map>
#include <set>
#include <string>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sai_tam_event.pb.h"
#include "sai_tam_buffer_stats.pb.h"
#include "sai_tam_event_threshold_breach.pb.h"

#define TEST_PROTO_ARG_TYPE           1
#define TEST_PROTO_ARG_PRID           2
#define TEST_PROTO_ARG_PGQRID         3
#define TEST_PROTO_ARG_COUNT          4
#define TEST_PROTO_ARG_STATP          5
#define TEST_PROTO_ARG_STATB          6
#define TEST_PROTO_ARG_STATERRTYPE    7

using namespace std;

/*  Command line arguments
 *
 *  Mandatory arguments:
 *  argv[1] - type - posible values are "pgshared, pgheadroom, qunicast, qmulticast"
 *  argv[2] - port_rid - RID of port (long int)
 *  argv[3] - pg/q rid - RID of pg/queue (long int)
 *  argv[4] - count - Number of times protobuf is to be sent
 *  
 *  Optional arguments:  
 *  argv[5] - statpercent - Stat value in percentage (int) (Bufferstats proto is sent)
 *  argv[6] - stat - Stat value in bytes (long int) (Bufferstats proto is also sent)
 *  argv[7] - staterrtype - Type of erroneous buffer stat proto to send. Possible
 *            values are "porterr, buffererr"
 */    
int main (int argc, char* argv[])
{
  Event msg;
  ThresholdBreach *event = msg.mutable_threshold_event();
  ThresholdSource *source = event->mutable_breach_source();
  struct sockaddr_in servaddr, cliaddr;
  int sockfd;
  int size, i =10, statp;
  uint64_t port_oid = 0, statb;
  uint64_t buffer_oid = 0;
  char buffer[5000];
  InterfaceBufferStatistics *intfstats;
  BufferStatistics *bufstats;
  string errstat;

  //Check args, should be atleast 5
  if (argc < 5)
  {
      cout << "Invalid argument list passed."; 
  }	  
 
  for (i = 0; i < argc; i++)
  {
       cout<<argv[i];
  }

  string btype(argv[TEST_PROTO_ARG_TYPE]);

  /* Get port pg/q rid and set it. */
  buffer_oid = stol(argv[TEST_PROTO_ARG_PGQRID]);
  
  /* First parse the mandatory arguments  
   * for breach report.
   */
  if ((btype == "pgshared") || 
       (btype == "pgheadroom"))
  {
      source->set_type(THRESHOLD_BREACH_AT_IPG);
      if (btype == "pgshared")
      {
          source->set_ipg_type(IPG_SHARED);
      }
      else
      {
          source->set_ipg_type(IPG_XOFF);
      }
      source->set_ipg_oid(buffer_oid);
  }
  else if ((btype == "qunicast") || 
            (btype == "qmulticast"))
  {
      source->set_type(THRESHOLD_BREACH_AT_QUEUE);
      if (btype == "qunicast")
      {
          source->set_queue_type(QUEUE_UNICAST);
      }
      else
      {
          source->set_queue_type(QUEUE_MULTICAST);
      }
      source->set_queue_oid(buffer_oid);
  }
  else
  {
      cout << "Invalid breach type argument provided.";
      cout << "Valid values are pgshared, pgheadroom, qunicast, qmulticast";
  }

  /* Get port oid and set it. */
  port_oid = stol(argv[TEST_PROTO_ARG_PRID]);
  /* Fill up the event data */
  source->set_port_oid(port_oid);
  if (argc > 5)
  {
      /* Buffer stats to be added */ 
      statp = stoi(argv[TEST_PROTO_ARG_STATP]);
      statb = stol(argv[TEST_PROTO_ARG_STATB]);
      
      if (argc == 8)
      {
          /* errneous report to be sent */
          errstat = argv[TEST_PROTO_ARG_STATERRTYPE];
      }

      intfstats = event->mutable_buffer_stats()->add_intf_buffer_stats();
      if (errstat == "porterr")
      {
          /* Send erroneous port oid. */
          intfstats->set_port_oid(0x000001000);
      }
      else 
      {
          intfstats->set_port_oid(port_oid);
      }

      if (btype == "pgshared" || btype == "pgheadroom")
      {
          /* PG stats */
          IPGStatistics *ipgstats = intfstats->add_ipg_stats();

          if (errstat == "buffererr")
          {
              /* Send erroneous pg oid. */
              ipgstats->set_ipg_oid(0x0000002);
          }
          else
          {
              ipgstats->set_ipg_oid(buffer_oid);
          }
            
          if (btype == "pgshared")
          {
              ipgstats->set_ipg_type(IPG_SHARED);
          }
          else
          {
              ipgstats->set_ipg_type(IPG_XOFF);
          }
          /* Set stat value. */
          bufstats = ipgstats->mutable_stats();
          bufstats->set_peak_buffer_occupancy_bytes(statb);
          bufstats->set_peak_buffer_occupancy_percent(statp);
      }
      else
      {
          /* Queue buffer stats. */
          /* queue stats */
          QueueStatistics *queuestats = intfstats->add_queue_stats();

          if (errstat == "buffererr")
          {
              /* Send erroneous pg oid. */
              queuestats->set_queue_oid(0x0000002);
          }
          else
          {
              queuestats->set_queue_oid(buffer_oid);
          }
            
          if (btype == "qunicast")
          {
              queuestats->set_queue_type(QUEUE_UNICAST);
          }
          else
          {
              queuestats->set_queue_type(QUEUE_MULTICAST);
          }

          /* Set stat value. */
          bufstats = queuestats->mutable_stats();
          bufstats->set_peak_buffer_occupancy_bytes(statb);
          bufstats->set_peak_buffer_occupancy_percent(statp);
      }
  }

  cout <<event->DebugString();

  // Creating socket file descriptor 
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
     cout << "socket creation failed"; 
     return -1;
  } 
      
  memset(&servaddr, 0, sizeof(servaddr)); 
  memset(&cliaddr, 0, sizeof(cliaddr)); 
      
  // Filling server information 
  servaddr.sin_family    = AF_INET; // IPv4 
  servaddr.sin_addr.s_addr = INADDR_ANY; 
  servaddr.sin_port = htons(9072); 

  //Client info
  cliaddr.sin_family    = AF_INET; // IPv4
  cliaddr.sin_addr.s_addr = INADDR_ANY;
  cliaddr.sin_port = htons(9171);

      
  // Bind the socket with the server address 
  if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
          sizeof(servaddr)) < 0 ) 
  { 
    cout <<"bind failed"; 
    return -1;
  } 

  /* Set timestamp */
  msg.set_timestamp(std::time(nullptr));
  size = msg.ByteSize();

  /* Serialize protobuf to array */ 
  msg.SerializeToArray(buffer, size);

  i = stoi(argv[TEST_PROTO_ARG_COUNT]);

  /* Setup a socket and send */
  while (i-- != 0)
  {
     sendto(sockfd, (char *)buffer, size, MSG_CONFIRM, 
                (const struct sockaddr *)&cliaddr, sizeof(cliaddr));

//     cout <<"Protobuf message sent. %d" << size;
//     sleep (1);
  }

  cout << i;

  /* Clean-up */
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
