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

#include <iostream>
#include <vector>
#include <string>

#include "tam_proto_buffer_stats_decoder.h"

TamProtoBufferStatsDecoder::TamProtoBufferStatsDecoder(SwitchBufferStats *bufferStats)
{
    m_protoDecoded = false;
    m_bufferStatsTimestamp = 0;

    /* bufferStats in the incoming proto to be decoded. */
    if (bufferStats == NULL)
    {
        SWSS_LOG_ERROR("Invalid buffer stats proto provided to TamProtoBufferStatsDecoder.");
        return;
    }

    /* Decode and buffer stats proto and populate the decoded data. */
    m_protoDecoded = decodeTamBufferStatsProto(bufferStats);
    if (m_protoDecoded != true)
    {
        SWSS_LOG_ERROR("Unable to decode Tam buffer stats protobuf.");
    }
}

bool TamProtoBufferStatsDecoder::decodeTamBufferStatsProto(SwitchBufferStats *bufferStats)
{
    int i = 0, j = 0;
    BufferPoolStatistics poolStats;
    BufferStatistics stats;
    InterfaceBufferStatistics intfStats;
    IPGStatistics ipgStats;
    QueueStatistics queueStats;

    if (bufferStats == NULL)
    {
        SWSS_LOG_ERROR("Invalid buffer stats proto provided to decodeTamBufferStatsProto.");
        return false;
    }

    /* Decode global buffer pool stats */
    for (i = 0; i < bufferStats->pool_stats_size(); i++)
    {
        poolStats = bufferStats->pool_stats(i);
        stats = poolStats.stats();
        poolType key = std::make_pair(poolStats.pool_oid(), poolStats.pool_type());
        statData data = std::make_pair(stats.peak_buffer_occupancy_bytes(), stats.peak_buffer_occupancy_percent());
        
        /* Insert the global pool stats into the map. */
        m_globalPoolBufStats[key] = data;
    }
    
    /* Decode per interface stats. */
    for (i = 0; i < bufferStats->intf_buffer_stats_size(); i++)
    {
        intfStats = bufferStats->intf_buffer_stats(i);

        /* Retrieve PG stats */
        for (j = 0; j < intfStats.ipg_stats_size(); j++)
        {
            ipgStats = intfStats.ipg_stats(j); 
            stats = ipgStats.stats();
            pgType key = std::make_pair(ipgStats.ipg_oid(), ipgStats.ipg_type());
            statData data = std::make_pair(stats.peak_buffer_occupancy_bytes(), stats.peak_buffer_occupancy_percent());

            /* Insert entry to pg stats. */
            m_pgBufStats[key] = data;
        }

        /* Retrieve queue stats */
        for (j = 0; j < intfStats.queue_stats_size(); j++)
        {
            queueStats = intfStats.queue_stats(j);
            stats = queueStats.stats();
            queueType key = std::make_pair(queueStats.queue_oid(), queueStats.queue_type());
            statData data = std::make_pair(stats.peak_buffer_occupancy_bytes(), stats.peak_buffer_occupancy_percent());

            /* Insert entry to queue stats. */
            m_queueBufStats[key] = data;
        }

        /* Retrieve interface buffer pool stats */
        for (j = 0; j < intfStats.pool_stats_size(); j++)
        {
            poolStats = intfStats.pool_stats(j);
            stats = poolStats.stats();
            interfacePoolType key = std::make_pair(poolStats.pool_oid(), poolStats.pool_type());
            statData data = std::make_pair(stats.peak_buffer_occupancy_bytes(), stats.peak_buffer_occupancy_percent());

            /* Insert entry to interface pool stats. */
            m_interfacePoolBufStats[key] = data;
        }
    }

    /* Retrieve buffer stats proto timestamp */
    m_bufferStatsTimestamp = 0;

    return true;
}

bufferStatsPg TamProtoBufferStatsDecoder::getPriorityGroupBufferStats()
{
    return m_pgBufStats;
}

bufferStatsQueue TamProtoBufferStatsDecoder::getQueueBufferStats()
{
    return m_queueBufStats;
}

bufferStatsGlobalPool TamProtoBufferStatsDecoder::getBufferPoolBufferStats()
{
    return m_globalPoolBufStats;
}

bufferStatsInterfacePool TamProtoBufferStatsDecoder::getInterfaceBufferPoolBufferStats()
{
    return m_interfacePoolBufStats;
}

uint64_t TamProtoBufferStatsDecoder::getBufferStatsTimestamp()
{
    return m_bufferStatsTimestamp;
}

bool TamProtoBufferStatsDecoder::getProtoDecoded()
{
    return m_protoDecoded;
}
