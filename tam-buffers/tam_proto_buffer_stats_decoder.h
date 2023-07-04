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

#ifndef _TAM_PROTO_BUFFER_STATS_DECODER_H_
#define _TAM_PROTO_BUFFER_STATS_DECODER_H_

#include <swss/dbconnector.h>
#include <swss/table.h>
#include <swss/producerstatetable.h>
#include <swss/notificationconsumer.h>
#include <swss/notificationproducer.h>
#include <swss/schema.h>

#include <map>
#include <set>
#include <string>

#include "sai_tam_buffer_stats.pb.h"

using namespace std;

typedef enum _bufferStatType {
    SNAPSHOTMGR_BUFFER_TYPE_PG,
    SNAPSHOTMGR_BUFFER_TYPE_QUEUE,
    SNAPSHOTMGR_BUFFER_TYPE_POOL
} bufferStatType;

typedef std::pair<uint64_t, IPGType> pgType; 

typedef std::pair<uint64_t, QueueType> queueType;

typedef std::pair<uint64_t, BufferPoolType> poolType;

typedef std::pair<uint64_t, BufferPoolType> interfacePoolType;

typedef std::pair<uint64_t, uint64_t> statData;

/* PG stats map: Key - (PG RID, PG type), Data - (64-bit stat in bytes, 64-bit stat in percent) */
typedef std::map<pgType, statData> bufferStatsPg;

/* Queue stats map: Key - (Queue RID, Queue type), Data - (64-bit stat in bytes, 64-bit stat in percent) */
typedef std::map<queueType, statData> bufferStatsQueue;

/* Global Buffer pool stats map: Key - (buffer pool RID, Pool Type), Data - (64-bit stat in bytes, 64-bit stat in percent) */
typedef std::map<poolType, statData> bufferStatsGlobalPool;

/* Per interface buffer pool stats map: Key - (buffer pool RID, Interface Pool Type), Data - (64-bit stat in bytes, 64-bit stat in percent) */
typedef std::map<interfacePoolType, statData> bufferStatsInterfacePool;

class TamProtoBufferStatsDecoder
{
public:
    TamProtoBufferStatsDecoder(SwitchBufferStats *bufferStats);
   
    ~TamProtoBufferStatsDecoder()
    {
    }

    /* -------------- Routines to decode and retrieve class data with snapshot information -------------- */

    /* Routine to get PG buffer stats. */
    bufferStatsPg getPriorityGroupBufferStats();

    /* Routine to get queue buffer stats. */
    bufferStatsQueue getQueueBufferStats();

    /* Routine to get global buffer pool buffer stats. */
    bufferStatsGlobalPool getBufferPoolBufferStats();

    /* Routine to get interface buffer pool buffer stats. */
    bufferStatsInterfacePool getInterfaceBufferPoolBufferStats();
    
    /* Routine to get buffer stats timestamp. */
    uint64_t getBufferStatsTimestamp();

    /* check if proto was decoded successfully. */
    bool getProtoDecoded();

private:
    bool                        m_protoDecoded;                   /* True if proto decoded successfully. */
    bufferStatsPg               m_pgBufStats;                     /* PG buffer stats */
    bufferStatsQueue            m_queueBufStats;                  /* Queue buffer stats */
    bufferStatsGlobalPool       m_globalPoolBufStats;             /* Global buffer pool buffer stats */
    bufferStatsInterfacePool    m_interfacePoolBufStats;          /* Interface buffer pool buffer stats */
    uint64_t                    m_bufferStatsTimestamp;           /* Buffer stats proto timestamp. */

    /* Routine to decode the snapshot protobuf data and populate the class data. */
    bool decodeTamBufferStatsProto(SwitchBufferStats *bufferStats);
};

#endif /* _TAM_PROTO_BUFFER_STATS_DECODER_H_ */
