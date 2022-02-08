

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

#include "kinect4_network.hpp"

// local
#include "utility/logger.hpp"
#include "utility/format.hpp"

using namespace tool::network;
using namespace tool::camera::K4;
using namespace std::chrono;

bool MultiPacketsMessageData::process(const Header &header, int8_t *data, size_t nbBytes){

    if(header.idMessage != idLastMultiPacketsMessageReceived){
        //            Logger::message(std::format("New frame {}\n", header.idMessage));
        // new frame
        receivingFrame = true;
        idLastMultiPacketsMessageReceived = header.idMessage;
        totalBytesReceived  = 0;
        nbPacketsReceived   = 0;
        firstPacketTimestamp   = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch());
    }

    if(!receivingFrame){
        // Logger::message(std::format("not receiving\n"));
        return false;
    }

    size_t totalPacketSizeBytes = header.totalNumberPackets * sizeof(Header);
    size_t totalDataSizeBytes   = header.totalSizeBytes - totalPacketSizeBytes;

    // resize data
    if(frameData.size() < totalDataSizeBytes){
        frameData.resize(totalDataSizeBytes);
    }

    // copy packet
    std::copy(
        data,
        data + header.currentPacketSizeBytes - sizeof (Header),
        frameData.begin() + header.dataOffset
    );

    totalBytesReceived += nbBytes;
    nbPacketsReceived++;

    // check integrity
    // # nb bytes received
    if(header.currentPacketSizeBytes != nbBytes){
        // drop packet
        Logger::error(fmt("MutliPacketsMessageData::process Invalid packet size {}/{}\n", header.currentPacketSizeBytes,nbBytes));
        receivingFrame = false;
        return false;
    }

    // # packet timeout
    size_t ts = duration_cast<milliseconds>((duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch())-firstPacketTimestamp)).count();
    if(ts > timeoutMs){
        // drop packet
        Logger::error(fmt("MutliPacketsMessageData::process Timeout packet {}ms\n", ts));
        receivingFrame = false;
        return false;
    }

    return true;
}

bool MultiPacketsMessageData::all_received(const Header &header){
    if(nbPacketsReceived == header.totalNumberPackets && totalBytesReceived == header.totalSizeBytes){
        receivingFrame = false;
        return true;
    }
    return false;
}

std::shared_ptr<CompressedCloudFrame> MultiPacketsMessageData::generate_compressed_cloud_frame(){

    auto ccFrame = std::make_shared<CompressedCloudFrame>();

    size_t offset = 0;

    // copy infos
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + 8,
        reinterpret_cast<std::int8_t*>(&ccFrame->colorWidth));
    offset += 8;
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + 8,
        reinterpret_cast<std::int8_t*>(&ccFrame->colorHeight));
    offset += 8;
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + 8,
        reinterpret_cast<std::int8_t*>(&ccFrame->validVerticesCount));
    offset += 8;

    size_t nbAudioFrames;
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + 8,
        reinterpret_cast<std::int8_t*>(&nbAudioFrames));
    offset += 8;
    ccFrame->audioFrames.resize(nbAudioFrames);

    size_t colorDataSize;
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + 8,
        reinterpret_cast<std::int8_t*>(&colorDataSize));
    offset += 8;
    ccFrame->encodedColorData.resize(colorDataSize);

    size_t cloudDataSize;
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + 8,
        reinterpret_cast<std::int8_t*>(&cloudDataSize));
    offset += 8;
    ccFrame->encodedCloudData.resize(cloudDataSize);

    // copy imusample
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + sizeof(ImuSample),
        reinterpret_cast<std::int8_t*>(&ccFrame->imuSample));
    offset += sizeof(ImuSample);

    // copy audio
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + 28 * nbAudioFrames,
        reinterpret_cast<std::int8_t*>(ccFrame->audioFrames.data()));
    offset += 28 * nbAudioFrames;

    // copy color
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + colorDataSize,
        reinterpret_cast<std::int8_t*>(ccFrame->encodedColorData.data()));
    offset += colorDataSize;

    // copy cloud
    std::copy(
        frameData.data() + offset,
        frameData.data() + offset + cloudDataSize,
        reinterpret_cast<std::int8_t*>(ccFrame->encodedCloudData.data()));
    offset += cloudDataSize;

    return ccFrame;
}

std::shared_ptr<CompressedFullFrame> MultiPacketsMessageData::generate_compressed_full_frame(){

    auto cfFrame = std::make_shared<CompressedFullFrame>();
    // ...

    return cfFrame;
}

Header Header::generate_mono_packet(MessageType type, size_t messageNbBytes){
    Header header;
    header.type = type;
    header.totalSizeBytes = sizeof(Header) + messageNbBytes;
    header.totalNumberPackets = 1;
    header.currentPacketId = 0;
    header.currentPacketSizeBytes = header.totalSizeBytes;
    header.currentPacketTime = 0;
    header.dataOffset = 0;
    return header;
}

void UdpMessage::copy_to_data(const Header &header, size_t messageNbBytes, std::vector<int8_t> &data) const{
    auto messageData = reinterpret_cast<const int8_t *>(this);
    if(data.size() < header.totalSizeBytes){
        data.resize(header.totalSizeBytes);
    }
    auto headerD = reinterpret_cast<const std::int8_t*>(&header);
    std::copy(headerD, headerD + sizeof(Header), data.begin());
    std::copy(messageData, messageData + messageNbBytes, data.begin() + sizeof(Header));
}

void UdpMessage::read_from_data(int8_t *data, uint32_t messageNbBytes){
    std::copy(data, data + messageNbBytes, reinterpret_cast<std::int8_t*>(this));
}

UdpInitFromManager::UdpInitFromManager(std::string ipAdressStr, uint16_t port, uint16_t maxSizeUdpPacket, std::int64_t timestamp) :
      port(port), maxSizeUdpPacket(maxSizeUdpPacket), timestamp(timestamp){

    std::fill(ipAdress.begin(), ipAdress.end(), ' ');
    if(ipAdressStr.size() <= 45){
        std::copy(std::begin(ipAdressStr), std::end(ipAdressStr), ipAdress.begin());
    }
}

UdpInitFromManager::UdpInitFromManager(int8_t *data, uint32_t messageNbBytes){
    read_from_data(data, messageNbBytes);
}

UdpFeedbackMessage::UdpFeedbackMessage(int8_t *data, uint32_t messageNbBytes){
    read_from_data(data, messageNbBytes);
}

UdpUpdateFiltersSettings::UdpUpdateFiltersSettings(int8_t *data, uint32_t messageNbBytes){
    read_from_data(data, messageNbBytes);
}

UdpUpdateDeviceSettings::UdpUpdateDeviceSettings(int8_t *data, uint32_t messageNbBytes){
    read_from_data(data, messageNbBytes);
}

UdpCommand::UdpCommand(int8_t *data, uint32_t messageNbBytes){
    read_from_data(data, messageNbBytes);
}
