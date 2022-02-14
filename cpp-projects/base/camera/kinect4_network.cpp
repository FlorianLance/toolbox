

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


std::shared_ptr<CompressedCloudFrame> UdpCompresedCloudFrameMessage::generate_frame(std::int8_t *data){

    auto ccFrame = std::make_shared<CompressedCloudFrame>();

    size_t offset = 0, nbAudioFrames, colorDataSize, cloudDataSize;

    // copy infos
    read(ccFrame->idCapture, data, offset);
    read(ccFrame->afterCaptureTS, data, offset);
    read(ccFrame->colorWidth, data, offset);
    read(ccFrame->colorHeight, data, offset);
    read(ccFrame->validVerticesCount, data, offset);
    read(nbAudioFrames, data, offset);   
    read(colorDataSize, data, offset);  
    read(cloudDataSize, data, offset);

    // resize arrays
    ccFrame->audioFrames.resize(nbAudioFrames);
    ccFrame->encodedColorData.resize(colorDataSize);
    ccFrame->encodedCloudData.resize(cloudDataSize);

    // copy imusample
    read(ccFrame->imuSample, data, offset);
    // copy audio
    read_array(ccFrame->audioFrames.data()->data(), data, nbAudioFrames*7, offset);
    // copy color
    read_array(ccFrame->encodedColorData.data(), data, colorDataSize, offset);
    // copy cloud
    read_array(ccFrame->encodedCloudData.data(), data, cloudDataSize, offset);

    return ccFrame;
}

size_t UdpCompresedCloudFrameMessage::initialize_data(std::shared_ptr<camera::K4::CompressedCloudFrame> frame, std::vector<std::int8_t> &data){

    size_t nbAudioFrames = frame->audioFrames.size();
    size_t encColorDataSize = frame->encodedColorData.size();
    size_t encCloudDataSize = frame->encodedCloudData.size();

    size_t totalDataSize =
        sizeof(frame->idCapture)  + sizeof(frame->afterCaptureTS) +
        sizeof(frame->colorWidth) + sizeof(frame->colorHeight) +
        sizeof(frame->validVerticesCount) +
        sizeof(nbAudioFrames) +
        sizeof(encColorDataSize) +
        sizeof(encCloudDataSize) +
        sizeof(ImuSample) +
        nbAudioFrames * 28+
        encColorDataSize +
        encCloudDataSize;

    size_t offset = 0;
    if(data.size() < totalDataSize){
        data.resize(totalDataSize);
    }

    auto dataP = data.data();

    // copy infos
    write(frame->idCapture, dataP, offset);
    write(frame->afterCaptureTS, dataP, offset);
    write(frame->colorWidth, dataP, offset);
    write(frame->colorHeight, dataP, offset);
    write(frame->validVerticesCount, dataP, offset);
    write(nbAudioFrames, dataP, offset);
    write(encColorDataSize, dataP, offset);
    write(encCloudDataSize, dataP, offset);
    // copy imusample
    write(frame->imuSample, dataP, offset);
    // copy audio
    write_array(frame->audioFrames.data()->data(), dataP, nbAudioFrames*7, offset);
    // copy color
    write_array(frame->encodedColorData.data(),  dataP, encColorDataSize,offset);
    // copy cloud
    write_array(frame->encodedCloudData.data(), dataP, encCloudDataSize, offset);

    return totalDataSize;
}

std::shared_ptr<CompressedFullFrame> UdpCompresedFullFrameMessage::generate_frame(std::int8_t *data){

    auto cfFrame = std::make_shared<CompressedFullFrame>();


    size_t offset = 0, nbAudioFrames, colorDataSize, depthDataSize, infraDataSize;

    // copy infos
    read(cfFrame->idCapture, data, offset);
    read(cfFrame->afterCaptureTS, data, offset);
    read(cfFrame->colorWidth, data, offset);
    read(cfFrame->colorHeight, data, offset);
    read(cfFrame->depthWidth, data, offset);
    read(cfFrame->depthHeight, data, offset);
    read(cfFrame->infraWidth, data, offset);
    read(cfFrame->infraHeight, data, offset);
    read(cfFrame->validVerticesCount, data, offset);
    read(nbAudioFrames, data, offset);
    read(colorDataSize, data, offset);
    read(depthDataSize, data, offset);
    read(infraDataSize, data, offset);

    // resize arrays
    cfFrame->audioFrames.resize(nbAudioFrames);
    cfFrame->encodedColorData.resize(colorDataSize);
    cfFrame->encodedDepthData.resize(depthDataSize);
    cfFrame->encodedInfraData.resize(infraDataSize);

    // copy imusample
    read(cfFrame->imuSample, data, offset);
    // copy audio
    read_array(cfFrame->audioFrames.data()->data(), data, nbAudioFrames*7, offset);
    // copy color
    read_array(cfFrame->encodedColorData.data(), data, colorDataSize, offset);
    // copy depth
    read_array(cfFrame->encodedDepthData.data(), data, depthDataSize, offset);
    // copy infra
    read_array(cfFrame->encodedInfraData.data(), data, infraDataSize, offset);

    return cfFrame;
}

size_t UdpCompresedFullFrameMessage::initialize_data(std::shared_ptr<camera::K4::CompressedFullFrame> frame, std::vector<int8_t> &data){

    size_t nbAudioFrames    = frame->audioFrames.size();
    size_t encColorDataSize = frame->encodedColorData.size();
    size_t encDepthDataSize = frame->encodedDepthData.size();
    size_t encInfraDataSize = frame->encodedInfraData.size();

    size_t totalDataSize =
        sizeof(frame->idCapture)  + sizeof(frame->afterCaptureTS) +
        sizeof(frame->colorWidth) + sizeof(frame->colorHeight) +
        sizeof(frame->depthWidth) + sizeof(frame->depthHeight) +
        sizeof(frame->infraWidth) + sizeof(frame->infraHeight) +
        sizeof(frame->validVerticesCount) +
        sizeof(nbAudioFrames) +
        sizeof(encColorDataSize) +
        sizeof(encDepthDataSize) +
        sizeof(encInfraDataSize) +
        sizeof(ImuSample) +
        nbAudioFrames * sizeof(float)*7+
        encColorDataSize +
        encDepthDataSize +
        encInfraDataSize;

    size_t offset = 0;
    if(data.size() < totalDataSize){
        data.resize(totalDataSize);
    }

    auto dataP = data.data();


    // copy infos
    write(frame->idCapture, dataP, offset);
    write(frame->afterCaptureTS, dataP, offset);
    write(frame->colorWidth, dataP, offset);
    write(frame->colorHeight, dataP, offset);
    write(frame->depthWidth, dataP, offset);
    write(frame->depthHeight, dataP, offset);
    write(frame->infraWidth, dataP, offset);
    write(frame->infraHeight, dataP, offset);
    write(frame->validVerticesCount, dataP, offset);
    write(nbAudioFrames, dataP, offset);
    write(encColorDataSize, dataP, offset);
    write(encDepthDataSize, dataP, offset);
    write(encInfraDataSize, dataP, offset);
    write(frame->imuSample, dataP, offset);

    // copy audio
    write_array(frame->audioFrames.data(), dataP, nbAudioFrames*7, offset);
    // copy color
    write_array(frame->encodedColorData.data(), dataP, encColorDataSize, offset);
    // copy depth
    write_array(frame->encodedDepthData.data(), dataP, encDepthDataSize, offset);
//    // copy infra
//    write_array(frame->encodedInfraData.data(), dataP, encInfraDataSize, offset);

    return totalDataSize;
}

