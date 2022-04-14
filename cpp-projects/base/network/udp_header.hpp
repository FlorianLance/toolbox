
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
#include <vector>

namespace tool::network{

struct Header{

    Header() = default;
    Header(std::int8_t* data){
        std::copy(data, data + sizeof(Header), reinterpret_cast<std::int8_t*>(this));
    }

    std::int8_t type = 0;
    std::uint32_t totalSizeBytes = 0;
    std::uint16_t totalNumberPackets = 0;
    std::uint16_t currentPacketId = 0;
    std::uint16_t currentPacketSizeBytes = 0;
    std::int64_t currentPacketTime = 0;
    std::uint32_t dataOffset = 0;
    std::int32_t idMessage = -1;

    static Header generate_mono_packet(int8_t type, size_t messageNbBytes);
};

struct UdpMonoPacketMessage{
    void init_packet_from_data(std::int8_t *data, std::uint32_t messageNbBytes);
    void copy_packet_to_data(const Header &header, std::vector<int8_t> &data) const;
};

struct UdpMultiPacketsMessage{

    bool receivingFrame = false;
    size_t timeoutMs = 15;
    std::int64_t firstPacketTimestamp;
    size_t totalBytesReceived = 0;    
    size_t nbPacketsReceived = 0;

    int idLastMultiPacketsMessageReceived = -1;
    int idLastMutliPacketsMessageSent = -1;

    bool copy_packet_to_data(const Header &header, size_t nbBytes, std::int8_t *packetData, std::vector<int8_t> &data);
    bool all_received(const Header &header);
};

}
