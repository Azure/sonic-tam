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

#include <errno.h>
#include <string.h>
#include <system_error>

#include <swss/logger.h>
#include "tam_socket_interface.h"

using namespace std;

TamSocketInterface::TamSocketInterface(uint16_t port, uint32_t saddr) :
    m_bufSize(TAM_MAX_MSG_LEN),
    m_messageBuffer(NULL),
    m_recv_timeout(TAM_SOCKET_TIMEOUT)
{
    struct sockaddr_in addr;
    int val = 1;

    /* Create a UDP socket. */
    m_tam_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_tam_socket < 0)
        throw system_error(errno, system_category());

    if (setsockopt(m_tam_socket, SOL_SOCKET, SO_REUSEADDR, &val,
                   sizeof(val)) < 0)
    {
        close(m_tam_socket);
        throw system_error(errno, system_category());
    }

    /* Bind socket to server address. */
    memset (&addr, 0, sizeof (addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(saddr);

    if (bind(m_tam_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(m_tam_socket);
        throw system_error(errno, system_category());
    }

    m_tam_socket_up = true;
    m_messageBuffer = new char[m_bufSize];
}

TamSocketInterface::~TamSocketInterface()
{
    m_tam_socket_up = false;

    /* Close the socket and return. */
    delete m_messageBuffer;
    close(m_tam_socket);
}

int TamSocketInterface::getFd()
{
    return m_tam_socket;
}

/* Setup select to timeout if data is not 
 * available on socket after m_recv_timeout. 
 * Return false on timeout or select error.
 */
bool TamSocketInterface::fdSelect()
{
    struct timeval timeout;
    fd_set readfds;
    int maxFd;

    /* Use select to timeout after pre-configured timeout. */
    timeout.tv_sec = m_recv_timeout;
    timeout.tv_usec = 0; 
    FD_ZERO(&readfds);
    FD_SET(m_tam_socket, &readfds);
    maxFd = m_tam_socket + 1;

    /* Wait on data. */
    if (select(maxFd, &readfds, NULL, 
                   NULL, &timeout) < 0)
    {
        return false;
    }

    if (!FD_ISSET(m_tam_socket, &readfds))
    {
        return false;
    }

    return true;
}

/* Receive processing thread. Read data from socket.
 * Returns size of data read.
 */
ssize_t TamSocketInterface::readData()
{
    socklen_t len;
    ssize_t dataRead;
    struct sockaddr_in cliaddr;
 
    /* Clean up buffer. */
    memset(m_messageBuffer, 0, m_bufSize);   
 
    /* At this point, data should be available on socket.
     * Read the data. 
     */
    dataRead = recvfrom(m_tam_socket, m_messageBuffer, m_bufSize,
                               MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
    if (dataRead <= 0)
    {
        SWSS_LOG_ERROR("recvfrom returned %d on the protobuf socket.", dataRead);
        return -1;
    }     
   
    return dataRead;
}
