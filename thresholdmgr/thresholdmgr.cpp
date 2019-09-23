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
                           m_redisAdapter(appDb, stateDb, counterDb, asicDb) 
{
    uint16_t port = THRESHOLD_MGR_BREACH_EVENT_PORT;
    uint32_t addr = INADDR_LOOPBACK;

    /* Initialize the socket interface. */
    m_protoInterface = unique_ptr<TamSocketInterface>
                         (new TamSocketInterface(port, addr));
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


string ThresholdMgr::serializeOid(uint64_t oid)
{
    char buf[32];
 
    snprintf(buf, sizeof(buf), "oid:0x%lx", oid);
 
    return buf;
}

bool ThresholdMgr::getBufferStat(SwitchBufferStats bufferStats, uint64_t if_oid, 
                                string buffer_type, uint64_t buffer_type_oid,
                                  uint64_t *stat, uint64_t *statPercent)
{
    int i = 0;
    InterfaceBufferStatistics intfStats;
    BufferStatistics stats;
    IPGType ipgType;
    QueueType queueType;

    /* Parse the bufferStats data and get the stat. */
    /* First parse for the port. */
    for (i = 0; i < bufferStats.intf_buffer_stats_size(); i++)
    {
        intfStats = bufferStats.intf_buffer_stats(i);
 
        /* Though port need not be validated for PG/queue OID, we do it
         * however to handle the case of an invalid protobuf report.
         */
        if (if_oid != intfStats.port_oid())   
        {
            continue;
        }

        /* Based upon buffer_type, determine the counter. */
        if ((buffer_type == "shared") || (buffer_type == "headroom"))
        {
            if (buffer_type == "shared")
            {
                ipgType = IPG_SHARED;
            }
            else
            {
                ipgType = IPG_XOFF;
            }
            
            /* Priority group stats */   
            int j = 0;
            for (j = 0; j < intfStats.ipg_stats_size(); j++)
            {
                IPGStatistics ipgStats = intfStats.ipg_stats(j);
                stats = ipgStats.stats();
                if ((buffer_type_oid == ipgStats.ipg_oid()) &&
                                  (ipgType == ipgStats.ipg_type()))
                {
                    /* Found the PG. Get the data. */
                    *stat = stats.peak_buffer_occupancy_bytes();              
                    *statPercent = stats.peak_buffer_occupancy_percent();
                    return true;
                }
            }
        }
        else if ((buffer_type == "unicast") || (buffer_type == "multicast"))
        {
            /* Queue stats */
            if (buffer_type == "unicast")
            {
                queueType = QUEUE_UNICAST;
            }
            else
            {
                queueType = QUEUE_MULTICAST;
            }
            
            /* Queue stats */   
            int j = 0;
            for (j = 0; j < intfStats.queue_stats_size(); j++)
            {
                QueueStatistics queueStats = intfStats.queue_stats(j);
                stats = queueStats.stats();
                if ((buffer_type_oid == queueStats.queue_oid()) &&
                                  (queueType == queueStats.queue_type()))
                {
                    /* Found the queue. Get the data. */
                    *stat = stats.peak_buffer_occupancy_bytes();              
                    *statPercent = stats.peak_buffer_occupancy_percent();
                    return true;
                }
            }
        }
    }

    return false;
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
 
    /* thresBreachEvent now has the breach event data. */
    /* Get port */
    if (breachSource.has_port_oid() == true)
    {
        if_oid = breachSource.port_oid();
        oid_string = serializeOid(if_oid);
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
                oid_string = serializeOid(buffer_type_oid);
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
                int numUcQueues;
                if (m_redisAdapter.getNumUcQueues(if_name, &numUcQueues) != true)           
                {
                    SWSS_LOG_ERROR("Unable to get numUcQueues for port %s", if_name.c_str());
                    return false;
                }  

                SWSS_LOG_DEBUG("numUcQueues %d for port %s", numUcQueues, if_name.c_str());
                index = index - numUcQueues;
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
                oid_string = serializeOid(buffer_type_oid);
                if (m_redisAdapter.getPgNumFromRid(oid_string, &index)
                             != true)
                {
                     SWSS_LOG_ERROR( "Invalid proto message pg oid %lld.", buffer_type_oid);
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
    if (processThresholdBreachData(m_protoInterface->m_messageBuffer, protoDataRead)
                                                                              != true )
    {
        SWSS_LOG_ERROR("Error while processing threshold protobuf data.");
        return false;
    }

   
    return true;
}

