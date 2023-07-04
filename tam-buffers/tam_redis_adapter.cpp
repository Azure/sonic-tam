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

#include <swss/dbconnector.h>
#include <swss/schema.h>
#include <swss/table.h>
#include <swss/macaddress.h>
#include <swss/notificationproducer.h>
#include <swss/producerstatetable.h>
#include <swss/table.h>
#include <swss/tokenize.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <string>
#include <algorithm>
#include <cstring>
#include <cstdint>

#include "tam_redis_adapter.h"

using namespace std;
using namespace swss;

/* Constructor function for Redis interface.
 * The redis interface is a placeholder for general routines
 * used to convert data in REDIS DBs.
 */
TamRedisAdapter::TamRedisAdapter(DBConnector *appDb, DBConnector *stateDb, 
                                  DBConnector *counterDb, DBConnector *asicDb) :
       m_queue_name_table(counterDb, COUNTERS_QUEUE_NAME_MAP),
       m_queue_type_table(counterDb, COUNTERS_QUEUE_TYPE_MAP),
       m_queue_port_table(counterDb, COUNTERS_QUEUE_PORT_MAP),
       m_queue_map_table(counterDb, COUNTERS_QUEUE_MAP),
       m_port_name_table(counterDb, COUNTERS_PORT_NAME_MAP),
       m_pool_name_table(counterDb, COUNTERS_BUFFER_POOL_NAME_MAP),
       m_pg_name_table(counterDb, COUNTERS_PG_NAME_MAP),
       m_vid_rid_table(asicDb, "VIDTORID")
{
//    /* Initialize all system related maps */
//    init_rid_portname_map();
//    init_rid_queuenum_map();
//    init_rid_pgnum_map();

    m_asicDb = asicDb;

    m_mapsInitialized = false;
}

/* Validate if RID exists on the
 * system.
 */
bool TamRedisAdapter::validateRID(string rid)
{
    Table tbl = Table(m_asicDb, "RIDTOVID");
    string val;

    tbl.hget("", rid, val);
    if (val != "")
    {
        return true;
    }

    return false;
}


/* Generate the RID to string maps for ports, PGs, queues.
 *
 */
bool TamRedisAdapter::generateRedisOidMaps()
{
    bool rc = false;

    /* Initialize all system related maps */
    rc = init_rid_portname_map();
    if (rc != true)
    {
        SWSS_LOG_ERROR("Unable to generate RID-portname map.");
        return false;
    }

    rc = init_rid_queuenum_map();
    if (rc != true)
    {
        SWSS_LOG_ERROR("Unable to generate RID-queuenum map.");
        return false;
    }

    rc = init_rid_pgnum_map();
    if (rc != true)
    {
        SWSS_LOG_ERROR("Unable to generate RID-pgnum map.");
        return false;
    }

    rc = init_rid_vid_map();
    if (rc != true)
    {
        SWSS_LOG_ERROR("Unable to generate RID-VID map.");
        return false;
    }

    /*initialize buffer pool from VID to buffer pool name*/
    if(init_buffer_pool_map() != true)
    {
        /* buffer pools will be created only when user explicitly configures it. ignore this error */
        SWSS_LOG_INFO("Unable to create pool map,Buffer pools are not created");
    }
    m_mapsInitialized = true;
    SWSS_LOG_DEBUG("Initialized redis RID maps for proto processing.");

    return rc;
}

/* Initialize RID (SAI object id) to port name map.
 * 
 */
bool TamRedisAdapter::init_rid_portname_map()
{
    SWSS_LOG_ENTER();
    vector<FieldValueTuple> vid_rid_values;
    vector<FieldValueTuple> port_name_values;

    m_vid_rid_table.get("", vid_rid_values);
    m_port_name_table.get("", port_name_values);

    if ((vid_rid_values.empty() == true) ||
        (port_name_values.empty() == true))
    {
        SWSS_LOG_DEBUG("vid_rid_values size %d, port_name_values size %d", 
                                      vid_rid_values.size(), port_name_values.size());
        return false;
    }

    /* Clear the MAP before inserting */
    m_rid_portname_map.clear();

    for (auto fv: port_name_values)
    {
        /* Extract port name field.
         * Port name field format - "Ethernet<Port>"
         * Port name value format - "oid:0xAAAAAAAABBBBBBBB" 
         */
        string portStr = fvField(fv);

        if (portStr.empty())
        {
            continue;
        }
  
        /* Search through vid-rid table entries and get the rid
         * corresponding to port name value(vid) and insert entry
         * RID-Port map 
         */

        for (auto fv1: vid_rid_values)
        {
            if (fvField(fv1) == fvValue(fv))
            {
                string ridStr = fvValue(fv1);
                if (!ridStr.empty())
                {
                    SWSS_LOG_DEBUG("Inserting entry in RID-Port with key %s and port %s", ridStr.c_str(), portStr.c_str());
                    m_rid_portname_map.insert(pair<string, string>(ridStr, portStr));
                    break;
                }
            }
        }
    }
    return true;
}

/* Initialize RID (SAI object id) to queue num map.
 * 
 */
bool TamRedisAdapter::init_rid_queuenum_map()
{
    SWSS_LOG_ENTER();
    vector<FieldValueTuple> vid_rid_values;
    vector<FieldValueTuple> queue_name_values;
    vector<FieldValueTuple> queue_type_values;

    m_vid_rid_table.get("", vid_rid_values);
    m_queue_name_table.get("", queue_name_values);
    m_queue_type_table.get("", queue_type_values);

    if ((vid_rid_values.empty() == true) ||
        (queue_type_values.empty() == true) ||
        (queue_name_values.empty() == true))
    {
        SWSS_LOG_DEBUG("vid_rid_values size %d, queue_name_values size %d", 
                                      vid_rid_values.size(), queue_name_values.size(), 
                                      queue_type_values.size());
        return false;
    }

    /* Clear the MAP before inserting */
    m_rid_queuenum_map.clear();
    m_port_num_ucqueues_map.clear();

    for (auto fv: queue_name_values)
    {
        /* Extract queue from queue name field.
         * Queue name field format - "Ethernet<Port>:queue"
         * Queue name value format - "oid:0xAAAAAAAABBBBBBBB" 
         */

        int queue;
        string queueStr = fvField(fv);

        /* Tokenize the string to extract the queue number. */
        vector<string> fields = tokenize(queueStr, delimiter);

        /* fields[0] has "alias", fields[1] has the queue number. */
        if (!fields[1].empty())
        {
            queue = stoi(fields[1]);
        }
        else
        {
            continue;
        }
  
        /* Search through vid-rid table entries and get the rid
         * corresponding to queue name value(vid) and insert entry
         * RID-queue map 
         */

        for (auto fv1: vid_rid_values)
        {
            if (fvField(fv1) == fvValue(fv))
            {
                string ridStr = fvValue(fv1);
                if (!ridStr.empty())
                {
                    SWSS_LOG_DEBUG("Inserting entry in RID-Queue with key %s and queue %d", ridStr.c_str(), queue);
                    m_rid_queuenum_map.insert(pair<string, int>(ridStr, queue));
                    break;
                }
            }
        }

        /* Populate port-numUcQueues map. 
         * fields[0] has portname, fvValue(fv) has the queue OID. 
         * Find queue type and populate the map. 
         */
        if (fields[0].empty())
        {
            continue;
        }

        for (auto fv2: queue_type_values)
        {
            if (fvValue(fv) != fvField(fv2))
            {
                /* Not the queue we are looking at. */
                continue;
            }
              
            /* Check if queue is unicast. */
            if (fvValue(fv2) == "SAI_QUEUE_TYPE_UNICAST")
            {
                m_port_num_ucqueues_map[fields[0]]++;
                break;
            }
        }
    }

    return true;
}

/* Initialize RID (SAI object id) to pg num map.
 * 
 */
bool TamRedisAdapter::init_rid_pgnum_map()
{
    SWSS_LOG_ENTER();
    vector<FieldValueTuple> vid_rid_values;
    vector<FieldValueTuple> pg_name_values;

    m_vid_rid_table.get("", vid_rid_values);
    m_pg_name_table.get("", pg_name_values);

    if ((vid_rid_values.empty() == true) ||
        (pg_name_values.empty() == true))
    {
        SWSS_LOG_DEBUG("vid_rid_values size %d, pg_name_values size %d", 
                                      vid_rid_values.size(), pg_name_values.size());
        return false;
    }

    /* Clear the MAP before inserting */
    m_rid_pgnum_map.clear();

    for (auto fv: pg_name_values)
    {
        /* Extract queue from queue name field.
         * PG name field format - "Ethernet<Port>:pg"
         * PG name value format - "oid:0xAAAAAAAABBBBBBBB" 
         */

        int pg;
        string pgStr = fvField(fv);

        /* Tokenize the string to extract the queue number. */
        vector<string> fields = tokenize(pgStr, delimiter);

        /* fields[0] has "alias", fields[1] has the queue number. */
        if (!fields[1].empty())
        {
            pg = stoi(fields[1]);
        }
        else
        {
            continue;
        }
  
        /* Search through vid-rid table entries and get the rid
         * corresponding to priority group name value(vid) and
         * insert entry RID-PG map 
         */

        for (auto fv1: vid_rid_values)
        {
            if (fvField(fv1) == fvValue(fv))
            {
                string ridStr = fvValue(fv1);
                if (!ridStr.empty())
                {
                    SWSS_LOG_DEBUG("Inserting entry in RID-PG with key %s and pg %d", ridStr.c_str(), pg);
                    m_rid_pgnum_map.insert(pair<string, int>(ridStr, pg));
                    break;
                }
            }
        }
    }
    return true;
}

/* Initialize RID (SAI object id) to VID.
 *
 */
bool TamRedisAdapter::init_rid_vid_map()
{
    SWSS_LOG_ENTER();
    vector<FieldValueTuple> vid_rid_values;

    m_vid_rid_table.get("", vid_rid_values);

    if (vid_rid_values.empty() == true) 
    {
        SWSS_LOG_DEBUG("vid_rid_values size %d",
                                      vid_rid_values.size());
        return false;
    }

    /* Clear the MAP before inserting */
    m_rid_vid_map.clear();

    /* Populate RID-VID map. */
    for (auto fv: vid_rid_values)
    {
        if (!fvField(fv).empty())
        {
            SWSS_LOG_DEBUG("Inserting entry in RID-VID map with key %s and val %s", fvValue(fv).c_str(), fvField(fv).c_str());
            m_rid_vid_map.insert(pair<string, string>(fvValue(fv), fvField(fv)));
        }
    }

    return true;
}

/* Initialize buffer pool from VID to buffer pool name*/
bool TamRedisAdapter::init_buffer_pool_map()
{
    SWSS_LOG_ENTER();
    vector<FieldValueTuple> pool_name_values;

    m_pool_name_table.get("", pool_name_values);

    if (pool_name_values.empty() == true)
    {
        SWSS_LOG_DEBUG("pool_name_values size %d",
                                      pool_name_values.size());
        return false;
    }

    /* Clear the MAP before inserting */
    m_pool_name_map.clear();

    for (auto fv: pool_name_values)
    {
         if(!fvField(fv).empty())
         {
            m_pool_name_map.insert(pair<string, string>(fvField(fv), fvValue(fv)));
         }
    }

    return true;
}
/* Get the port name given an RID string.
 * 
 */
bool TamRedisAdapter::getPortNameFromRid(string rid, string *portName)
{
    SWSS_LOG_ENTER();
    ridPortNameMap::iterator it;

    it = m_rid_portname_map.find(rid);
    if (it != m_rid_portname_map.end())
    {
        *portName = it->second;
        SWSS_LOG_DEBUG("Found port name %s corresponding to RID %s", portName->c_str(), rid.c_str());
        return true;
    }

    SWSS_LOG_DEBUG("Port number corresponding to RID %s does not exist", rid.c_str());
    return false;
}

/* Get the queue num given an RID string.
 * 
 */
bool TamRedisAdapter::getQueueNumFromRid(string rid, int *queueNum)
{
    SWSS_LOG_ENTER();
    ridQueueNumMap::iterator it;

    it = m_rid_queuenum_map.find(rid);
    if (it != m_rid_queuenum_map.end())
    {
        *queueNum = it->second;
        SWSS_LOG_DEBUG("Found queue number %d corresponding to RID %s", *queueNum, rid.c_str());
        return true;
    }

    SWSS_LOG_DEBUG("Queue number corresponding to RID %s does not exist", rid.c_str());
    return false;
}

/* Get the priority-group num given an RID string.
 * 
 */
bool TamRedisAdapter::getPgNumFromRid(string rid, int *pgNum)
{
    SWSS_LOG_ENTER();
    ridPriorityGroupNumMap::iterator it;

    it = m_rid_pgnum_map.find(rid);
    if (it != m_rid_pgnum_map.end())
    {
        *pgNum = it->second;
        SWSS_LOG_DEBUG("Found priority group %d corresponding to RID %s", *pgNum, rid.c_str());
        return true;
    }

    SWSS_LOG_DEBUG("Priority group number corresponding to RID %s does not exist", rid.c_str());
    return false;
}

/* Get the VID for an object RID.
 *
 */
bool TamRedisAdapter::getVidFromRid(string rid, string *vid)
{
    SWSS_LOG_ENTER();
    ridVidMap::iterator it;

    it = m_rid_vid_map.find(rid);
    if (it != m_rid_vid_map.end())
    {
        *vid = it->second;
        SWSS_LOG_DEBUG("Found VID %s corresponding to RID %s", vid->c_str(), rid.c_str());
        return true;
    } 

    SWSS_LOG_DEBUG("VID entry corresponding to RID %s not found.", rid.c_str());
    return false;
}

/* Validate RID and see if we need to update maps.
 * If so, update and re-attempt converting RID to VID.
 */
bool TamRedisAdapter::updateGetVidFromRid(string rid, string *vid)
{
    ridVidMap::iterator it;

    /* This API is called by caller when we couldn't find a mapping
     * for the given RID. Validate if such an RID exists in the
     * system. If it does, our maps may be stale. Refresh them.
     */
    if (validateRID(rid) == true)
    {
        SWSS_LOG_NOTICE("Updating snapshot maps, RID %s found in system but not in local map.", rid.c_str());
        if (generateRedisOidMaps() != true)
        {
            SWSS_LOG_NOTICE("Unable to generate Redis maps, will re-attempt on next invocation.");
            return false;
        }

        /* Maps are re-generated. Attempt to find RID now in local map. */
        it = m_rid_vid_map.find(rid);
        if (it != m_rid_vid_map.end())
        {
            *vid = it->second;
            SWSS_LOG_DEBUG("Found VID %s corresponding to RID %s", vid->c_str(), rid.c_str());
            return true;
        }

        /* If we got here, there is an issue with our map creation technique.
         * Log an error mentioning, a restart of snapshotmgr may be required to recover.
         */
        SWSS_LOG_ERROR("Erroneous map generated on refresh, restart snapshotmgr to recover.");
    }
    else
    {
        SWSS_LOG_NOTICE("Received an erroneous RID in protobuf. No mapping for RID %s found on system.", rid.c_str());
    }

    SWSS_LOG_DEBUG("VID entry corresponding to RID %s not found.", rid.c_str());
    return false;
}

/* Get the priority-group num given an RID string.
 *
 */
bool TamRedisAdapter::getNumUcQueues(string if_name, int *numUcQueues)
{
    SWSS_LOG_ENTER();
    portNumUcQueuesMap::iterator it;

    it = m_port_num_ucqueues_map.find(if_name);
    if (it != m_port_num_ucqueues_map.end())
    {
        *numUcQueues = it->second;
        SWSS_LOG_DEBUG("Found numUcQueues %d corresponding to port %s", *numUcQueues, if_name.c_str());
        return true;
    }

    SWSS_LOG_DEBUG("numUcQueues for port %s does not exist", if_name.c_str());
    return false;
}

/* Check if maps are generated.
 *
 */
bool TamRedisAdapter::getMapsInitialized()
{
    return m_mapsInitialized;  
}

/* Check if maps are generated.
 *
 */
bool TamRedisAdapter::getSystemMapReady()
{
	bool systemReady = false;
	string mapDone = "false";

	m_queue_map_table.hget("queuMapInitDone", "Done", mapDone);

	if(mapDone == "true")
	{
		systemReady = true;
	}

        SWSS_LOG_DEBUG("System Ready is %d", systemReady);
	return systemReady;
}

/* Convert uint64_t sai object to an OID string 
 * for REDIS_DB format.
 */
string TamRedisAdapter::serializeOid(uint64_t oid)
{
    char buf[32];

    snprintf(buf, sizeof(buf), "oid:0x%lx", oid);

    return buf;
}

/* Get list of all PG OIDs(VID) in 
 * system.
 */
vector<string> TamRedisAdapter::getPgOidList()
{
    vector<string> pgList;
    vector<FieldValueTuple> pg_name_values;

    m_pg_name_table.get("", pg_name_values);
    
    for (auto fv: pg_name_values)
    {
        /* fvValue(fv) returns the PG VID. 
         * Simply insert into pgList.
         */
        pgList.push_back(fvValue(fv));      
    }

    return pgList;
}

/* Get list of all queue OIDs(VID) in
 * system.
 */
vector<string> TamRedisAdapter::getQueueOidList()
{
    vector<string> queueList;
    vector<FieldValueTuple> queue_name_values;

    m_queue_name_table.get("", queue_name_values);

    for (auto fv: queue_name_values)
    {
        /* fvValue(fv) returns the Queue VID.
         * Simply insert into queueList.
         */
        queueList.push_back(fvValue(fv));
    }

    return queueList;
}

/* Get pool list OIDs(VID) in
 * system.
 */
vector<string> TamRedisAdapter::getPoolOidList()
{
    vector<string> poolList;
    vector<FieldValueTuple> pool_name_values;

    m_pool_name_table.get("", pool_name_values);

    for (auto fv: pool_name_values)
    {
        /* fvValue(fv) returns the Pool VID.
         * Simply insert into poolList.
         */
        poolList.push_back(fvValue(fv));
    }

    return poolList;
}

/* Get pool name map in
 * system.
 */
bool TamRedisAdapter::getBufferPoolName(string vid, string *buffer_name)
{
    SWSS_LOG_ENTER();
    bufferPoolNameMap::iterator it;

    it = m_pool_name_map.begin();
    for (;it != m_pool_name_map.end();it++)
    {
        if(it->second == vid)
        {
          *buffer_name = it->first;
          return true;
        }
    }
    return false;
}
