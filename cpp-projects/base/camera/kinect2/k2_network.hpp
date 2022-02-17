
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

// local
#include "k2_types.hpp"

namespace tool::camera {

// tcp
enum class K2TcpCommand : uint8_t{
    Init,
    OpenCamera,
    CloseCamera,
    UpdateParameters,
    AskNewFrame,
    ShutdownComputer,
    RestartComputer,
    AskToSaveSettingsFile
};

struct K2TcpPacket{
    std::uint8_t idGrabber; // 8
    K2TcpCommand cmd; // 8
    K2FrameRequest frameMode; // 8
    K2Settings p;
    int64_t t; // 64
    uint16_t sizeUdpPackets; // 16
    int writingPort; // 32
    std_a1<char, 45>  writingIpv6Address; // 45*8
};

struct K2UdpHeader{

    std::int64_t  timeStamp = 0; // 64
    std::uint64_t frameId : 32, offset : 32; // 64
    std::uint64_t sizeFullData : 32, size1: 32; // 64
    std::uint64_t size2 : 32, totalNbPackets : 16, sizePacketData : 16; // 64
    std::float_t intrinsics1; // 32
    std::float_t intrinsics2; // 32
    std::float_t intrinsics3; // 32
    std::float_t intrinsics4; // 32
    uint16_t idPacket    = 0; // 16
    uint8_t idGrabber      = 0; // 8
    K2FrameRequest frameMode = K2FrameRequest::compressed_color_512x424; // 8
};

//struct K2UdpFrameData2{

//    K2UdpHeader firstHeader;
//    size_t sizePts  = 0;
//    size_t sizeTris = 0;
//    // more infos
//    // ...
//};

struct K2UdpFrameData{

    K2UdpFrameData() : data(std::make_unique<K2FrameData>()), bufferData(std::make_unique<K2FrameData>()){}

    std::mutex swapLock;
    K2FrameDataUP data = nullptr;
    K2FrameDataUP bufferData = nullptr;

    size_t sizePts  = 0;
    size_t sizeTris = 0;

    void swap(){
        swapLock.lock();
        std::swap(data, bufferData);
        swapLock.unlock();
    }

    //    enum class LastUpdated {normal,colored,mesh,none};
    //    LastUpdated lastUpdate = LastUpdated::none;

    K2UdpHeader firstHeader;
};

struct K2GrabberTargetInfo{
    std::string name;
    int targetPreferredInterface;
    int targetReadingPort;
    int targetWritingPort;
};


}
