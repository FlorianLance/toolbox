

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


UdpInitFromManager::UdpInitFromManager(std::string ipAdressStr, uint16_t port, uint16_t maxSizeUdpPacket) :
      port(port), maxSizeUdpPacket(maxSizeUdpPacket){

    std::fill(ipAdress.begin(), ipAdress.end(), ' ');
    if(ipAdressStr.size() <= 45){
        std::copy(std::begin(ipAdressStr), std::end(ipAdressStr), ipAdress.begin());
    }
}

UdpInitFromManager::UdpInitFromManager(int8_t *data){
    init_packet_from_data(data, sizeof (UdpInitFromManager));
}

UdpFeedbackMessage::UdpFeedbackMessage(int8_t *data){
    init_packet_from_data(data, sizeof (UdpFeedbackMessage));
}

UdpUpdateFiltersSettings::UdpUpdateFiltersSettings(int8_t *data){
    init_packet_from_data(data, sizeof (UdpUpdateFiltersSettings));
}

UdpUpdateDeviceSettings::UdpUpdateDeviceSettings(int8_t *data){
    init_packet_from_data(data, sizeof (UdpUpdateDeviceSettings));
}

UdpCommand::UdpCommand(int8_t *data){
    init_packet_from_data(data, sizeof(UdpCommand));
}


std::shared_ptr<CompressedCloudFrame> UdpCompresedCloudFrameMessage::generate_frame(){

    auto frameDataPtr = frameData.data();
    auto ccFrame = std::make_shared<CompressedCloudFrame>();

    size_t offset = 0;

    // copy infos
    read(ccFrame->idCapture, offset);
    read(ccFrame->afterCaptureTS, offset);

    read(ccFrame->colorWidth, offset);
    read(ccFrame->colorHeight, offset);
    read(ccFrame->validVerticesCount, offset);

    size_t nbAudioFrames;
    read(nbAudioFrames, offset);
    ccFrame->audioFrames.resize(nbAudioFrames);

    size_t colorDataSize;
    read(colorDataSize, offset);
    ccFrame->encodedColorData.resize(colorDataSize);

    size_t cloudDataSize;
    read(cloudDataSize, offset);
    ccFrame->encodedCloudData.resize(cloudDataSize);

    // copy imusample
    read(ccFrame->imuSample, offset);

    // copy audio
    std::copy(
        frameDataPtr + offset,
        frameDataPtr + offset + 28 * nbAudioFrames,
        reinterpret_cast<std::int8_t*>(ccFrame->audioFrames.data()));
    offset += 28 * nbAudioFrames;

    // copy color
    std::copy(
        frameDataPtr + offset,
        frameDataPtr + offset + colorDataSize,
        reinterpret_cast<std::int8_t*>(ccFrame->encodedColorData.data()));
    offset += colorDataSize;

    // copy cloud
    std::copy(
        frameDataPtr + offset,
        frameDataPtr + offset + cloudDataSize,
        reinterpret_cast<std::int8_t*>(ccFrame->encodedCloudData.data()));
    offset += cloudDataSize;

    return ccFrame;
}

std::shared_ptr<CompressedFullFrame> UdpCompresedFullFrameMessage::generate_frame(){

    auto frameDataPtr = frameData.data();
    auto cfFrame = std::make_shared<CompressedFullFrame>();
    // ...

    return cfFrame;
}

