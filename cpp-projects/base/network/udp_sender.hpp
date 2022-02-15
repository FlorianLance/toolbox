
/*******************************************************************************
** Toolbox-base                                                               **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/

#pragma once

// std
#include <memory>

// signals
#include "lsignal.h"

// local
#include "udp_header.hpp"

namespace tool::network{

class UdpSender {

public:

    UdpSender();
    ~UdpSender();

    bool is_opened() const;

    // socket
    bool init_socket(std::string tagetName, std::string writingPort);//, size_t readingInterface);//, int readingPort);
    void clean_socket();

    // send
    size_t send_packet_data(std::int8_t *packetData, size_t nbBytes);
    size_t send_packet(Header &header);
    size_t send_packets(Header &header, size_t allPacketsNbBytes);

    void update_size_packets(size_t newUdpPacketSize);

protected:

    size_t sizeUdpPacket = 9000;
    std::vector<std::int8_t> packetBuffer;
    std::vector<std::int8_t> bufferToSend;

private:

    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};
}
