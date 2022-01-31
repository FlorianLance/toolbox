
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
#include <array>
#include <string>

// local
#include "kinect4_types.hpp"

namespace tool::camera::K4 {

enum MessageType : std::int8_t {
    manager_id = 0,
    start_cameras,
    stop_cameras,
    settings,
    ask_frame,
    quit,
    quit_and_shutdown,
    feedback,
    data
};

enum Feedback : std::int8_t{
    valid,
    timeout
};

struct Header{
    MessageType type;
    std::uint32_t totalSizeBytes = 0;
    std::uint16_t totalNumberPackets = 0;
    std::uint16_t currentPacketId = 0;
    std::uint16_t currentPacketSizeBytes = 0;
    std::uint32_t currentPacketTime = 0;
    std::uint32_t dataOffset = 0;
};

struct UdpMessage{
};

struct UdpInitFromManager : UdpMessage{

    UdpInitFromManager() = default;
    UdpInitFromManager(std::string ipAdressStr, uint16_t port, uint16_t maxSizeUdpPacket) : port(port), maxSizeUdpPacket(maxSizeUdpPacket){
        std::fill(ipAdress.begin(), ipAdress.end(), ' ');
        if(ipAdressStr.size() <= 45){
            std::copy(std::begin(ipAdressStr), std::end(ipAdressStr), ipAdress.begin());
        }
    }

    std::array<char, 45> ipAdress;
    std::uint16_t port;
    std::uint16_t maxSizeUdpPacket;
};

struct UdpStartCameras : UdpMessage{
    Mode mode;
};

struct UdpStopCameras : UdpMessage{
};

struct UdpAskForFrame : UdpMessage{
};



struct UdpFeedbackMessage : UdpMessage{
    MessageType receivedMessageType;
    Feedback feedback;
};

struct DataMessage : UdpMessage{
    //    std::vector<Header> headers;
    //    std::vector<std::vector<std::int16_t>> data;
};

}
