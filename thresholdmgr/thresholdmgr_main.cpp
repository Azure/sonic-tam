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
#include "thresholdmgr.h"

int main(int argc, char **argv)
{
    swss::Logger::linkToDbNative("thresholdmgr");

    try
    {
        SWSS_LOG_NOTICE("-----Starting ThresholdMgr-----");

        swss::DBConnector appDb(APPL_DB, DBConnector::DEFAULT_UNIXSOCKET, 0);
        swss::DBConnector stateDb(STATE_DB, DBConnector::DEFAULT_UNIXSOCKET, 0);
        swss::DBConnector counterDb(COUNTERS_DB, DBConnector::DEFAULT_UNIXSOCKET, 0);
        swss::DBConnector asicDb(ASIC_DB, DBConnector::DEFAULT_UNIXSOCKET, 0);
    
        /* Initialize the threshold manager. */
        ThresholdMgr *thresholdMgr = new ThresholdMgr(&appDb, &stateDb, &counterDb, &asicDb);

        while (true)
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
    catch (const exception &e)
    {
        SWSS_LOG_ERROR("Runtime error: %s", e.what());
    }

    return -1;
}
