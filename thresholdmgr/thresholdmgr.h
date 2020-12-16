/*
 * Copyright 2019 Broadcom Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _THRESHOLDMGR_H_
#define _THRESHOLDMGR_H_

#include <swss/dbconnector.h>
#include <swss/schema.h>
#include <swss/table.h>
#include <swss/macaddress.h>
#include <swss/notificationproducer.h>
#include <swss/producerstatetable.h>
#include <swss/table.h>
#include <swss/select.h>
#include <swss/timestamp.h>

#include "redisapi.h"
#include "tam_redis_adapter.h"
#include "tam_socket_interface.h"
#include "tam_proto_buffer_stats_decoder.h"
#include "sai_tam_event.pb.h"
#include "sai_tam_buffer_stats.pb.h"
#include "sai_tam_event_threshold_breach.pb.h"

#define COUNTER_DB_TABLE_DELIMITER                  ":"
#define THRESHOLD_MGR_BREACH_EVENT_PORT             9171
/* Address to loopback in software to local host.  */
#define ADDR_LOOPBACK                               0x7f000001      /* 127.0.0.1   */

#define PG_WATERMARK_PLUGIN_NAME                    "watermark_pg.lua"
#define QUEUE_WATERMARK_PLUGIN_NAME                 "watermark_queue.lua"
#define BUFFER_POOL_WATERMARK_PLUGIN_NAME           "watermark_pool.lua"



static const std::map<std::string, std::string> thresSaiCounter =
{
    { "shared", "SAI_INGRESS_PRIORITY_GROUP_STAT_SHARED_WATERMARK_BYTES" },
    { "headroom", "SAI_INGRESS_PRIORITY_GROUP_STAT_XOFF_ROOM_WATERMARK_BYTES" },
    { "unicast", "SAI_QUEUE_STAT_SHARED_WATERMARK_BYTES"  },
    { "multicast", "SAI_QUEUE_STAT_SHARED_WATERMARK_BYTES" },
    { "ingress", "SAI_BUFFER_POOL_STAT_WATERMARK_BYTES" },
    { "egress", "SAI_BUFFER_POOL_STAT_WATERMARK_BYTES" }

};

using namespace swss;
using namespace std;

class ThresholdMgr
{ 
public:
    ThresholdMgr(DBConnector *appDb, DBConnector *stateDb, DBConnector *counterDb, DBConnector *asicDb);
    ~ThresholdMgr()
    {
    }

    /* Routine to read and process the threshold protobuf data. */
    bool readProcessProtobuf();
    TamRedisAdapter m_redisAdapter;
    int getFd();
    void resetPlugins();

private:
    unique_ptr<TamSocketInterface> m_protoInterface;
    Table m_counterTable;
    Table m_counterStatTable;
    string m_pgWmSha;
    string m_queueWmSha;
    string m_poolWmSha;
    bool m_runAllPlugins;

    /* Get the buffer statistics from the buffer stats proto class */
    bool getBufferStat(SwitchBufferStats bufferStats, uint64_t if_oid, 
                            string buffer_type, uint64_t buffer_type_oid,
                                    uint64_t *stat, uint64_t *statPercent);

    /* Generate event-id from DB. */
    int generateBreachEventIndex();

    /* Process the threshold breach protobuf data. */
    bool processThresholdBreachData(char *protoBuffer, int len);
   
    /* Process the threshold breach protobuf data and write stats to Db. */
    bool processThresholdStatsData(char *protoBuffer, int len);

    bool initializeCounters(string timestamp);

    void runPlugins(vector<string> poolList, vector<string> pgList, vector<string> queueList);

    /* Run lua plugins for all PG's, queue's and buffer pools. */
    void runAllPlugins();

};

#endif // _THRESHOLDMGR_H_
