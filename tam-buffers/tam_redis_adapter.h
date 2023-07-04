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

#ifndef _TAM_REDIS_ADAPTER_H
#define _TAM_REDIS_ADAPTER_H

#include <swss/dbconnector.h>
#include <swss/table.h>
#include <swss/producerstatetable.h>
#include <swss/notificationconsumer.h>
#include <swss/notificationproducer.h>
#include <swss/schema.h>

#include <map>
#include <set>
#include <string>

const char delimiter           = ':';

using namespace std;

namespace swss { 

/* RID-Port Map: Key - RID, Data - Port name*/
typedef map<string, string> ridPortNameMap;

/* RID-Queue Map: Key - RID, Data - Queue number */
typedef map<string, int> ridQueueNumMap;

/* RID-PG Map: Key - RID, Data - priority group number */
typedef map<string, int> ridPriorityGroupNumMap;

/* Portname-numUcQueues Map: Key - Port name, Data - numUcQueues */
typedef map<string, int> portNumUcQueuesMap;

/* Cache the RIDTOVID table since this is referenced a lot for snapshot. */
/* RID-VID Nap: Key - Object RID, Data - Object VID */
typedef map<string, string> ridVidMap;

typedef map<string, string> bufferPoolNameMap;

class TamRedisAdapter
{
public:
    TamRedisAdapter(DBConnector *appDb, DBConnector *stateDb, DBConnector *counterDb, DBConnector *asicDb);
    ~TamRedisAdapter()
    {
    }

    /* API to get port name corresponding to RID */
    bool getPortNameFromRid(string rid, string *portName);

    /* API to get queue number corresponding to RID */
    bool getQueueNumFromRid(string rid, int *queueNum);

    /* API to get priority group corresponding to RID */
    bool getPgNumFromRid(string rid, int *pgNum);

    /* API to get VID for RID of an object. */
    bool getVidFromRid(string rid, string  *vid);

    /* API to validate RID and update maps if needed. */
    bool updateGetVidFromRid(string rid, string *vid);

    /* API to get buffer pool name from VID. */
    bool getBufferPoolName(string vid, string *buffer_name);

    /* API to get numUcQueues corresponding to portName */
    bool getNumUcQueues(string port, int *numUcQueues);

    /* API to generate PG, queue and port OID maps */
    bool generateRedisOidMaps();

    /* API to find if maps are initialized. Return boolean. */
    bool getMapsInitialized();

    /* API to find if all system maps are ready. */
    bool getSystemMapReady();

    /* API to get list of all PGs. */
    vector<string> getPgOidList();  

    /* API to get list of all queues. */
    vector<string> getQueueOidList();  

    /* API to get list of all pools. */
    vector<string> getPoolOidList();  

    /* API to generate the OID string from uint64_t SAI object ID */
    string serializeOid(uint64_t oid);

private:
    Table                  m_queue_name_table;       /* Table to store queue name entries */
    Table                  m_queue_type_table;       /* Table to store queue type entries */
    Table                  m_queue_port_table;       /* Table to store queue port entries */
    Table                  m_queue_map_table;        /* Table to store queue map entries */
    Table                  m_port_name_table;        /* Table to store port name entries */
    Table                  m_pg_name_table;          /* Table to store priority group name entries */
    Table                  m_vid_rid_table;          /* Table to store VID RID entries */
    Table                  m_pool_name_table;        /* Table to store buffer pool entries */

    ridPortNameMap         m_rid_portname_map;        /* RID-Port Map */
    ridQueueNumMap         m_rid_queuenum_map;       /* RID-Queue Map */
    ridPriorityGroupNumMap m_rid_pgnum_map;          /* RID- PG Map */
    ridVidMap              m_rid_vid_map;            /* RID-VID Map */
    bufferPoolNameMap      m_pool_name_map;          /* VID-POOL name map*/
    portNumUcQueuesMap     m_port_num_ucqueues_map;  /* Port-NumUcQueues Map */
    bool                   m_mapsInitialized;        /* Are RID maps initialized. */
    DBConnector            *m_asicDb;                /* ASIC DB Connector */

    /* Initializes RID-Port map */
    bool init_rid_portname_map();

    /* Initializes RID-Queue  map */
    bool init_rid_queuenum_map();

    /* Initializes RID-PG  map */
    bool init_rid_pgnum_map();

    /* Initializes RID-VID  map */
    bool init_rid_vid_map();

    /* Initializes VID-POOL name  map */
    bool init_buffer_pool_map();

    /* Validate RID from DB */
    bool validateRID(string rid);
};

}

#endif /* _TAM_REDIS_ADAPTER_H */
