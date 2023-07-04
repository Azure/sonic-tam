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

#include <vector>
#include <string>
#include <netinet/in.h>

#include "thresholdmgr.h"

ThresholdMgr::ThresholdMgr(DBConnector *appDb, DBConnector *stateDb,
                            DBConnector *counterDb, DBConnector *asicDb) :
                           m_counterTable(counterDb, COUNTERS_THRESHOLD_BREACH_TABLE),
                           m_counterStatTable(new RedisPipeline(counterDb, 16000), COUNTERS_TABLE, true),
                           m_redisAdapter(appDb, stateDb, counterDb, asicDb)
{
    uint16_t port = THRESHOLD_MGR_BREACH_EVENT_PORT;
    uint32_t addr = INADDR_LOOPBACK;

    /* Initialize the socket interface. */
    m_protoInterface = unique_ptr<TamSocketInterface>
                         (new TamSocketInterface(port, addr));

    /* Setup plugins to be run on COUNTERS_DB. */
    string pgLuaScript = loadLuaScript(PG_WATERMARK_PLUGIN_NAME);
    string queueLuaScript = loadLuaScript(QUEUE_WATERMARK_PLUGIN_NAME);
    string poolLuaScript = loadLuaScript(BUFFER_POOL_WATERMARK_PLUGIN_NAME);

    m_pgWmSha = loadRedisScript(counterDb, pgLuaScript);
    m_queueWmSha = loadRedisScript(counterDb, queueLuaScript);
    m_poolWmSha = loadRedisScript(counterDb, poolLuaScript);
    m_runAllPlugins = false;

}

int ThresholdMgr::generateBreachEventIndex()
{
    int lastEventid = 0;
    string key ("event-id");
    vector<FieldValueTuple> fvs;

    /* Read COUNTERS_DB THRESHOLD_BREACH_ENTRY
     * for getting the last written event-id.
     */
    m_counterTable.get(key, fvs);
    if (fvs.empty() != true)
    {
        /* Pick last event-id and start from there. */
        /* We have only 1 entry in this table. */
        for (auto fv: fvs)
        {
            if (fvField(fv) == "id")
            {
                lastEventid = stoi(fvValue(fv));
            }
        }
    }

    return lastEventid;
}


bool ThresholdMgr::getBufferStat(SwitchBufferStats bufferStats, uint64_t if_oid, 
                                string buffer_type, uint64_t buffer_type_oid,
                                  uint64_t *stat, uint64_t *statPercent)
{
    TamProtoBufferStatsDecoder stats(&bufferStats);
    statData data = std::make_pair(0, 0); 

    /* Check if buffer stats proto was decoded successfully. */
    if (stats.getProtoDecoded() != true)
    {
        SWSS_LOG_ERROR("Unable to parse/decode buffer stats proto report.");
        return false;
    }

    /* Retrieve stats according to the buffer */
    if ((buffer_type == "shared") || (buffer_type == "headroom"))
    {
        /* Get priority-group stats. */
        bufferStatsPg pgStats;
        IPGType type;

        if (buffer_type == "shared")
        {
            type = IPG_SHARED;
        }
        else 
        {
            type = IPG_XOFF;
        }

        pgStats = stats.getPriorityGroupBufferStats();
        pgType key = std::make_pair(buffer_type_oid, type);
        bufferStatsPg::iterator it;

        /* Find the entry. If not found, return false. */
        it = pgStats.find(key);
        if (it == pgStats.end()) 
        {
            SWSS_LOG_ERROR("PG 0x%x not found in buffer stats report", buffer_type_oid);
            return false;
        }
        data = it->second;
    }
    else if ((buffer_type == "unicast" || buffer_type == "multicast"))
    {
        /* Get queue stats. */
        bufferStatsQueue queueStats;
        QueueType type;

        if (buffer_type == "unicast")
        {
            type = QUEUE_UNICAST;
        }
        else 
        {
            type = QUEUE_MULTICAST;
        }

        queueStats = stats.getQueueBufferStats();
        queueType key = std::make_pair(buffer_type_oid, type);
        bufferStatsQueue::iterator it;

        /* Find the entry. If not found, return false. */
        it = queueStats.find(key);
        if (it == queueStats.end()) 
        {
            SWSS_LOG_ERROR("Queue 0x%x not found in buffer stats report", buffer_type_oid);
            return false;
        }
        data = it->second;
    }
    else if ((buffer_type == "ingress") || (buffer_type == "egress"))
    {
        /* Get queue stats. */
        bufferStatsGlobalPool poolStats;
        BufferPoolType type;

        if (buffer_type == "ingress")
        {
            type = BUFFERPOOL_INGRESS;
        }
        else 
        {
            type = BUFFERPOOL_EGRESS;
        }

        poolStats = stats.getBufferPoolBufferStats();
        poolType key = std::make_pair(buffer_type_oid, type);
        bufferStatsGlobalPool::iterator it;

        /* Find the entry. If not found, return false. */
        it = poolStats.find(key);
        if (it == poolStats.end()) 
        {
            SWSS_LOG_ERROR("Pool 0x%x not found in buffer stats report", buffer_type_oid);
            return false;
        }
        data = it->second;
    }

    *stat = data.first;
    *statPercent = data.second;

    return true;
}

bool ThresholdMgr::processThresholdBreachData(char *protoBuffer, int len)
{
    ThresholdBreach   thresBreachEvent;
    ThresholdSource   breachSource;
    SwitchBufferStats bufferStats;
    Event             event;
    string            if_name, timestamp, oid_string;
    uint64_t          if_oid = 0, buffer_type_oid = 0;
    uint64_t          stat = 0, statPercent = 0;
    time_t            time;
    string            buffer, buffer_type;
    int               index = 0;
    char              tmp[64];
 
    if (protoBuffer == NULL) 
    {
        return false;
    }

    /* Convert the message buffer to protobuf message.
     * Serialize APIs convert the message buffer to
     * protobuf format. m_messageBuffer has the array.
     * len needs to be the exact size of protobuf data sent.
     * Errors could be seen if len is not accurate.
     */
    if (event.ParseFromArray(protoBuffer, len) != true)
    {
        SWSS_LOG_ERROR( "Unable to parse buffer to event proto format.");
        return false;
    }

    /* Get the timestamp and format it. */
    memset(tmp, 0, sizeof(tmp));
    time = static_cast<time_t> (event.timestamp());
    size_t size = strftime(tmp, 32 ,"%Y-%m-%d.%T", localtime(&time));
    if (size != 0)
    {
        timestamp = string(tmp);
    }
    else
    {
        SWSS_LOG_DEBUG("Generating software timestamp, proto timestamp is invalid.");
        /* Generate SW timestamp. */
        timestamp = getTimestamp();
    }

    thresBreachEvent = event.threshold_event();
 
    SWSS_LOG_DEBUG( "Successfully decoded threshold proto of length %d", len);
    SWSS_LOG_DEBUG("%s", thresBreachEvent.DebugString().c_str());
 
    breachSource = thresBreachEvent.breach_source();
 
 
    /* Check if threshold breach proto has buffer stats. */
    if (thresBreachEvent.has_buffer_stats())
    {
        /* Decode buffer statistics. */
        bufferStats = thresBreachEvent.buffer_stats();
    }

    /* Check if system map is ready. */ 
    if (m_redisAdapter.getSystemMapReady() != true)
    {
       SWSS_LOG_NOTICE("System Maps are not ready. Ignore processing protobufs.");
       return false;
    }

    /* Check if port, queue and pg RID maps are generated. 
     * Generate them if not generated yet.
     */
    if (m_redisAdapter.getMapsInitialized() != true)
    {
        if (m_redisAdapter.generateRedisOidMaps() != true)
        {
            SWSS_LOG_ERROR("Aborting protobuf processing, couldn't generate RID maps");
            return false;
        }
    }
    vector<string>      buffer_pool = m_redisAdapter.getPoolOidList();
 
    /* thresBreachEvent now has the breach event data. */
    /* Get port */
    if (breachSource.has_port_oid() == true)
    {
        if_oid = breachSource.port_oid();
        oid_string = m_redisAdapter.serializeOid(if_oid);
        if (m_redisAdapter.getPortNameFromRid(oid_string, &if_name)
                                 != true)
        {
            SWSS_LOG_ERROR( "Invalid proto message port oid %lld.", if_oid);
            return false;
        }
    }
 
    /* Get breach type. */
    switch (breachSource.type())
    {
        case ThresholdBreachSourceType::THRESHOLD_BREACH_AT_QUEUE:
            buffer = "queue";
            /* UCQ/MCQ/CPUQ trigger */
            if (breachSource.has_queue_type() == true)
            {
                if (breachSource.queue_type() == QueueType::QUEUE_UNICAST) 
                {
                    buffer_type = "unicast";
                }
                else if (breachSource.queue_type() == QueueType::QUEUE_MULTICAST)
                {
                    buffer_type = "multicast";
                }
            }

            /* queue number - index */
            if (breachSource.has_queue_oid() == true)
            {
                buffer_type_oid = breachSource.queue_oid();
                oid_string = m_redisAdapter.serializeOid(buffer_type_oid);
                if (m_redisAdapter.getQueueNumFromRid(oid_string, &index)
                             != true)
                {
                    SWSS_LOG_ERROR( "Invalid proto message queue oid %lld.", buffer_type_oid);
                    return false;
                }
            }

            /* For multicast queues, we show relative index. 
             * mc index = index - numUcQueues.
             */
            if (breachSource.queue_type() == QueueType::QUEUE_MULTICAST)
            {
              if(if_name != "CPU")
              {
                int numUcQueues;
                if (m_redisAdapter.getNumUcQueues(if_name, &numUcQueues) != true)           
                {
                    SWSS_LOG_ERROR("Unable to get numUcQueues for port %s", if_name.c_str());
                    return false;
                }  

                SWSS_LOG_DEBUG("numUcQueues %d for port %s", numUcQueues, if_name.c_str());
                index = index - numUcQueues;
              }
            }

 
            break;

        case ThresholdBreachSourceType::THRESHOLD_BREACH_AT_IPG:
            /* priority-group trigger */
            buffer = "priority-group";

            switch (breachSource.ipg_type())
            {
                case IPGType::IPG_SHARED:
                    buffer_type = "shared";
                    break;

                case IPGType::IPG_XOFF:
                    buffer_type = "headroom";
                    break;

                default:
                    break;
            }

            /* PG index */
            if (breachSource.has_ipg_oid() == true)
            {
                buffer_type_oid = breachSource.ipg_oid();
                oid_string = m_redisAdapter.serializeOid(buffer_type_oid);
                if (m_redisAdapter.getPgNumFromRid(oid_string, &index)
                             != true)
                {
                     SWSS_LOG_ERROR( "Invalid proto message pg oid %lld.", buffer_type_oid);
                     return false;
                }
            }
            break;

        case ThresholdBreachSourceType::THRESHOLD_BREACH_AT_GLOBAL_POOLS:
            switch (breachSource.pool_type())
            {
                case BufferPoolType::BUFFERPOOL_INGRESS:
                    buffer_type = "ingress";
                    break;

                case BufferPoolType::BUFFERPOOL_EGRESS:
                    buffer_type = "egress";
                    break;

                default:
                    break;
            }

            /* pool oid */
            if (breachSource.has_pool_oid() == true)
            {
                buffer_type_oid = breachSource.pool_oid();
                oid_string = m_redisAdapter.serializeOid(buffer_type_oid);
                string vid;

                if (m_redisAdapter.getVidFromRid(oid_string, &vid)
                             != true)
                {
                  if(!(buffer_pool.empty()))
                  {
                     SWSS_LOG_ERROR( "Invalid buffer pool OID %lld in report, no corresponding VID found", buffer_type_oid);
                     return false;
                  }
                }
                /*Get buffer pool name for corresponding VID*/
                if(m_redisAdapter.getBufferPoolName(vid, &buffer) != true)
                {
                   SWSS_LOG_ERROR( "Invalid buffer pool VID %lld in report, no corresponding Buffer pool name found", buffer_type_oid);
                   return false;
                }
            }

        default:
            break;
    }

    if (thresBreachEvent.has_buffer_stats())
    {
        /* Get the buffer statistics for breach source. */
        if (getBufferStat(bufferStats, if_oid, buffer_type, buffer_type_oid,
                                    &stat, &statPercent) != true)
        {
            SWSS_LOG_ERROR("Invalid buffer stats report received for breach event %s oid:%lld",
                                  thresSaiCounter.at(buffer_type).c_str(), buffer_type_oid);
            return false;
        }
    }


    SWSS_LOG_DEBUG("Successfully processed breach event protobuf.");
    SWSS_LOG_DEBUG("Breach event buffer: %s, type: %s, port %s.", buffer.c_str(), buffer_type.c_str(), if_name.c_str());
    SWSS_LOG_DEBUG("index: %d, breach_value: %d, %s: %lld, time-stamp %s.", index, statPercent,
                                        thresSaiCounter.at(buffer_type).c_str(), stat, timestamp.c_str());

    /* Create the threshold breach table entry 
     * with the data from protobuf 
     * processing and write it to COUNTER_DB.
     */
    /* Create the THRESHOLD_BREACH table entry. */
    int eventid = generateBreachEventIndex();
    string key = string("breach-report") + COUNTER_DB_TABLE_DELIMITER + 
                                               to_string(eventid);
    vector<FieldValueTuple> fvs;

    /* Populate the THRESHOLD_BREACH entry. */
    fvs.emplace_back("buffer", buffer);
    fvs.emplace_back("type", buffer_type);
    fvs.emplace_back("port", if_name);
    fvs.emplace_back("index", to_string(index));
    fvs.emplace_back("breach_value", to_string(statPercent));
    fvs.emplace_back(thresSaiCounter.at(buffer_type), to_string(stat));
    fvs.emplace_back("time-stamp", timestamp);

    /* Write THRESHOLD_BREACH entry to COUNTERS_DB. */
    m_counterTable.set(key, fvs);

    /* Populate the next event-id to be used into COUNTERS_DB. */
    string eventkey ("event-id");
    vector<FieldValueTuple> eventfvs;

    /* Write next breach event id to use in case of warmboot,
     * docker restart etc. into COUNTERS_DB.
     */
    eventfvs.emplace_back("id", to_string(++eventid));
    m_counterTable.set(eventkey, eventfvs);

    return true;
}

void ThresholdMgr::runAllPlugins()
{
    vector<string> pgList = m_redisAdapter.getPgOidList();
    vector<string> queueList = m_redisAdapter.getQueueOidList();
    vector<string> poolList = m_redisAdapter.getPoolOidList();
    runPlugins(poolList, pgList, queueList);
}

void ThresholdMgr::runPlugins(vector<string> poolList, vector<string> pgList, vector<string> queueList)
{
    swss::DBConnector db("COUNTERS_DB", 0);

    /* Lua script expects argv to be set.
     * Most of this data is not used.
     * Only argv[0] is used by the lua script
     */
    const vector<string> argv =
    {
        to_string(COUNTERS_DB),
        COUNTERS_TABLE,
        to_string(1 * 1000)
    };

    /* Run the Lua plugins for pool, pg and queue counters. */
    /* Alternatively, run the plugin for all PGs, queues and pools present in COUNTERS_DB MAPS. */

    /* Pool plugin */
    runRedisScript(db, m_poolWmSha, poolList, argv);

    /* PG plugin */
    runRedisScript(db, m_pgWmSha, pgList, argv);

    /* Queue plugin */
    runRedisScript(db, m_queueWmSha, queueList, argv);
}

bool ThresholdMgr::processThresholdStatsData(char *protoBuffer, int len)
{
    ThresholdBreach   thresBreachEvent;
    SwitchBufferStats   bufferStats;
    Event               event;
    string              timestamp;
    char                tmp[64];
    statData            data = std::make_pair(0, 0);
    vector<string>      pgList;
    vector<string>      queueList;
    vector<string>      poolList;
    time_t              time;

    if (protoBuffer == NULL)
    {
        return false;
    }

    /* Convert the message buffer to protobuf message.
     * Serialize APIs convert the message buffer to
     * protobuf format. m_messageBuffer has the array.
     * len needs to be the exact size of protobuf data sent.
     * Errors could be seen if len is not accurate.
     */
    if (event.ParseFromArray(protoBuffer, len) != true)
    {
        SWSS_LOG_ERROR( "Unable to parse buffer to event proto format.");
        return false;
    }

    thresBreachEvent = event.threshold_event();

    /* Check if threshold breach proto has buffer stats. */
    if (!(thresBreachEvent.has_buffer_stats()))
    {
        SWSS_LOG_INFO( "No Stats Present.");
        return false;
    }
    /* Decode buffer statistics. */
    bufferStats = thresBreachEvent.buffer_stats();

    /* Populate timestamp of report. */
    time = static_cast<time_t> (event.timestamp());

    if (time != 0)
    {
        size_t size = strftime(tmp, 32 ,"%Y-%m-%d.%T", localtime(&time));

        if (size != 0)
        {
            timestamp = string(tmp);
        }
    }
    else
    {
        SWSS_LOG_DEBUG("Generating software timestamp, proto timestamp is invalid.");
        /* Generate SW timestamp. */
        timestamp = getTimestamp();
    }


    /* Decode the buffer stats protobuf. */
    TamProtoBufferStatsDecoder   protoDecoder(&bufferStats);

    if (protoDecoder.getProtoDecoded() != true)
    {
        SWSS_LOG_ERROR("Unable to decode buffer stats proto.");
        return false;
    }

    /* Check if system map is ready. */
    if (m_redisAdapter.getSystemMapReady() != true)
    {
       SWSS_LOG_NOTICE("System Maps are not ready. Ignore processing protobufs.");
       return false;
    }

    /* Check if port, queue and pg RID maps are generated.
     * Generate them if not generated yet.
     */
    if (m_redisAdapter.getMapsInitialized() != true)
    {
        if (m_redisAdapter.generateRedisOidMaps() != true)
        {
            SWSS_LOG_ERROR("Aborting protobuf processing, couldn't generate RID maps");
            return false;
        }
    }

    /* Buffer stats event proto was parsed and decoded successfully. Get the stats and write to COUNTERS_DB. */
    SWSS_LOG_INFO("Proto received %s", event.DebugString().c_str());

    vector<string>      buffer_pool = m_redisAdapter.getPoolOidList();

    /* Get and process all global buffer pool stats. */
    bufferStatsGlobalPool poolStats = protoDecoder.getBufferPoolBufferStats();
    bufferStatsGlobalPool::iterator it;

    for (it = poolStats.begin(); it  != poolStats.end(); it++)
    {
        /* Get the entry. Convert the buffer pool
         * RID to VID and populate COUNTERS_DB.
         */
        poolType pool = it->first;
        string rid, vid, counterName;
        vector<FieldValueTuple> values;

        data = it->second;
        /* Serialize RID (uint64_t) */
        rid = m_redisAdapter.serializeOid(pool.first);
        /* Convert RID to VID */
        if (m_redisAdapter.getVidFromRid(rid, &vid) != true)
        {
           /*When VID is received in stats and it is not                                                                                                          present in non empty buffer pool table then throw error*/
           if(!(buffer_pool.empty()))
           {
             SWSS_LOG_ERROR( "Invalid buffer pool OID %lld in report, no corresponding VID found", pool.first);
           }

           continue;
        }

        /* Update counters. Check if buffer pool type matters here.
         * We are only interested in stat in bytes right now.
         */
        counterName = "SAI_BUFFER_POOL_STAT_WATERMARK_BYTES";
        values.emplace_back(counterName, to_string(data.first));
        m_counterStatTable.set(vid, values);

        /* Update counter percent. */
        counterName = "SAI_BUFFER_POOL_PERCENT_STAT_WATERMARK";
        values.emplace_back(counterName, to_string(data.second));
        m_counterStatTable.set(vid, values);

        /* Update timestamp in DB. */
        counterName = "SAI_BUFFER_POOL_STAT_TIMESTAMP";
        values.emplace_back(counterName, timestamp);
        m_counterStatTable.set(vid, values);

        /* Add key to pool list to run plugin on. */
        poolList.push_back(vid);
    }

    /* Decode PG stats and write to COUNTERS_DB */
    bufferStatsPg pgStats = protoDecoder.getPriorityGroupBufferStats();
    bufferStatsPg::iterator pgit;

    /* Loop through all PGs and populate the counters. */
    for (pgit = pgStats.begin(); pgit != pgStats.end(); pgit++)
    {
        /* Get the entry. Convert PG
         * RID to VID and populate COUNTERS_DB.
         */
        pgType priorityGroup = pgit->first;
        data = pgit->second;

        string rid, vid, counterName;
        vector<FieldValueTuple> values;

        /* Serialize RID (uint64_t) */
        rid = m_redisAdapter.serializeOid(priorityGroup.first);

        /* Convert RID to VID */
        if (m_redisAdapter.getVidFromRid(rid, &vid) != true)
        {
            SWSS_LOG_ERROR("Invalid priority-group OID %lld in report, no corresponding VID found", priorityGroup.first);
            continue;
        }

        /* Update counters. */
        if (priorityGroup.second == IPG_SHARED)
        {
            counterName = "SAI_INGRESS_PRIORITY_GROUP_STAT_SHARED_WATERMARK_BYTES";
        }
        else
        {
            counterName = "SAI_INGRESS_PRIORITY_GROUP_STAT_XOFF_ROOM_WATERMARK_BYTES";
        }

        values.emplace_back(counterName, to_string(data.first));

        /* Update counter percent. */
        if (priorityGroup.second == IPG_SHARED)
        {
            counterName = "SAI_INGRESS_PRIORITY_GROUP_PERCENT_STAT_SHARED_WATERMARK";
        }
        else
        {
            counterName = "SAI_INGRESS_PRIORITY_GROUP_PERCENT_STAT_XOFF_ROOM_WATERMARK";
        }

        values.emplace_back(counterName, to_string(data.second));

        /* Update timestamp in DB. */
        counterName = "SAI_PG_STAT_TIMESTAMP";

        values.emplace_back(counterName, timestamp);
        m_counterStatTable.set(vid, values);


        /* Add key to pg list to run plugin on. */
        pgList.push_back(vid);
    }

    /* Decode queue stats and populate in COUNTERS_DB. */
    bufferStatsQueue queueStats = protoDecoder.getQueueBufferStats();
    bufferStatsQueue::iterator qit;

    /* Loop through all queues and populate the counters. */
    for (qit = queueStats.begin(); qit != queueStats.end(); qit++)
    {
        /* Get the entry. Convert Queue
         * RID to VID and populate COUNTERS_DB.
         */
        queueType queue = qit->first;
        data = qit->second;

        string rid, vid, counterName;
        vector<FieldValueTuple> values;

        /* Serialize RID (uint64_t) */
        rid = m_redisAdapter.serializeOid(queue.first);

        /* Convert RID to VID */
        if (m_redisAdapter.getVidFromRid(rid, &vid) != true)
        {
            SWSS_LOG_ERROR("Invalid queue OID %lld in report, no corresponding VID found", queue.first);
            continue;
        }

        /* Update counters. */
        counterName = "SAI_QUEUE_STAT_SHARED_WATERMARK_BYTES";

        values.emplace_back(counterName, to_string(data.first));

        /* Update counter percent. */
        counterName = "SAI_QUEUE_PERCENT_STAT_SHARED_WATERMARK";

        values.emplace_back(counterName, to_string(data.second));

		        /* Update timestamp in DB. */
        counterName = "SAI_QUEUE_STAT_TIMESTAMP";

        values.emplace_back(counterName, timestamp);
        m_counterStatTable.set(vid, values);

        /* Add key to queue  list to run plugin on. */
        queueList.push_back(vid);
    }


    m_counterStatTable.flush();

    /* For the first report, run through all lists for initialising WM tables. */
    if (m_runAllPlugins == false)
    {
        runAllPlugins();
        m_runAllPlugins = true;
    }
    else
    {
        /* Run plugins to generate watermark table data. */
        runPlugins(poolList, pgList, queueList);
    }
    return true;
}


bool ThresholdMgr::readProcessProtobuf()
{
    int protoDataRead = 0;

    /* Read the protobuf data from the protobuf
     * socket interface and process the protobuf.
     */
    protoDataRead = (int)m_protoInterface->readData();
    if (protoDataRead <= 0)
    {
        SWSS_LOG_ERROR("Error while receiving threshold protobuf data.");
        return false;
    }

    SWSS_LOG_DEBUG("Protobuf message received of length %d", protoDataRead);

    /* Read data is saved in protoInterface->m_messageBuffer.
     * Extract data into protobuf and process it.
     */
    if (processThresholdStatsData(m_protoInterface->m_messageBuffer, protoDataRead)
                                                                              != true )
    {
        SWSS_LOG_ERROR("Error while processing threshold protobuf stats data.");
        return false;
    }


    if (processThresholdBreachData(m_protoInterface->m_messageBuffer, protoDataRead)
                                                                              != true )
    {
        SWSS_LOG_ERROR("Error while processing threshold protobuf data.");
        return false;
    }

    return true;
}

int ThresholdMgr::getFd()
{
    return m_protoInterface->getFd();
}

void ThresholdMgr::resetPlugins()
{
    m_runAllPlugins = false;
}
