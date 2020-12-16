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
#include <swss/subscriberstatetable.h>
#include <swss/select.h>
#include "thresholdmgr.h"

bool sonic_tam_threshold_port_queue_map_init(ThresholdMgr *thresholdmgr);
int main(int argc, char **argv)
{
    swss::Logger::linkToDbNative("thresholdmgr");
    map<string, KeyOpFieldsValuesTuple> queue_map;

    try
    {
        SWSS_LOG_NOTICE("-----Starting ThresholdMgr-----");

        swss::DBConnector appDb("APPL_DB", 0);
        swss::DBConnector stateDb("STATE_DB", 0);
        swss::DBConnector counterDb("COUNTERS_DB", 0);
        swss::DBConnector asicDb("ASIC_DB", 0);
    
        /* Initialize the threshold manager. */
        ThresholdMgr *thresholdMgr = new ThresholdMgr(&appDb, &stateDb, &counterDb, &asicDb);
        Select s;
        SubscriberStateTable mapInit(&counterDb, COUNTERS_QUEUE_MAP);
        s.addSelectable(&mapInit);

        int sock_fd = thresholdMgr->getFd();
        SelectableFd fd(sock_fd);
        s.addSelectable(&fd);

        while (true)
        {
          Selectable *sel = NULL;
          s.select(&sel);

          if (sel == (Selectable *)&mapInit)
          {
            std::deque<KeyOpFieldsValuesTuple> entries;
            mapInit.pops(entries);

            for (auto entry: entries)
            {
                    string key = kfvKey(entry);
                    queue_map[key] = entry;
            }
            /* When changes occur in qMapInitDone
            * get port and queue maps and update
            * system maps with recent mappings
            * */
               SWSS_LOG_DEBUG("DPB Event occured calling Qmap reinit in Thresholdmgr.");
            thresholdMgr->m_redisAdapter.generateRedisOidMaps();
            thresholdMgr->resetPlugins();
          }

          if (sel == (Selectable *)&fd)
          {
            /* ThresholdMgr listens to protobuf data,
             * processes the protobuf data and writes
             * it to COUNTER_DB.
             */ 
            if (thresholdMgr->readProcessProtobuf() != true)
            {
                SWSS_LOG_DEBUG("Unable to read and process threshold protobuf data.");
                continue;
            }
          }
        }
    }
    catch (const exception &e)
    {
        SWSS_LOG_ERROR("Runtime error: %s", e.what());
    }

    return -1;
}

