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

#ifndef _TAM_SOCKET_INTERFACE_H
#define _TAM_SOCKET_INTERFACE__H

#include <map>
#include <set>
#include <string>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

/* Maximum size of proto buffer */
#define TAM_MAX_MSG_LEN                     50000
#define TAM_SOCKET_TIMEOUT                  3               /* 3 seconds */

class TamSocketInterface
{
public:
    TamSocketInterface(uint16_t port, uint32_t saddr);      
    ~TamSocketInterface();
    
    /* Raw data arriving from socket. */
    char *m_messageBuffer;

    /* Flag to check if socket is up. */
    bool m_tam_socket_up;

    /* Read data from socket and save in buffer; */
    bool fdSelect();

    /* Get socket descriptor. */
    int getFd();

    /* Read from socket. */
    ssize_t readData();

private:
    unsigned int   m_bufSize;
    int            m_tam_socket;
    int            m_recv_timeout;
};

#endif /* _TAM_SOCKET_INTERFACE_H */
