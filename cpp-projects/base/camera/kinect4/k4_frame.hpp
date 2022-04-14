
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
#include "camera/frame.hpp"
#include "k4_types.hpp"
#include "geometry/matrix4.hpp"
#include "utility/io_data.hpp"

namespace tool::camera{


/**
 * @brief Compressed kinect4 base frame
 */
struct K4CompressedFrame : Frame{

    size_t validVerticesCount = 0;
    size_t colorWidth = 0;
    size_t colorHeight = 0;
    std::vector<std::uint8_t> encodedColorData;

    //        std::vector<std::uint8_t> encodedAudioData; // todo
    std::vector<std::array<float, 7>> audioFrames; // to be removed
    K4ImuSample imuSample;

    virtual size_t total_data_size() const{
        // contains also audioFrames.size() and encodedColorData.size()
        return
            sizeof(size_t)*6+sizeof(std::int64_t)+ encodedColorData.size()+
            audioFrames.size()*7*sizeof(float) + sizeof(K4ImuSample);
    }
};

/**
 * @brief Compressed kinect4 cloud frame
 */
struct K4CompressedCloudFrame : public K4CompressedFrame{

    std::vector<std::uint8_t> encodedCloudData;

    // contains also encodedCloudData.size()
    size_t total_data_size() const override{
        return K4CompressedFrame::total_data_size() + encodedCloudData.size() + sizeof(size_t);
    }

    K4CompressedCloudFrame() = default;
    K4CompressedCloudFrame(std::int8_t *data){
        init_from_data(data);
    }

    void init_from_data(std::int8_t *data){

        size_t offset = 0, nbAudioFrames, encodedColorDataSize, encodedcloudDataSize;

        // copy infos
        read(idCapture, data, offset);
        read(afterCaptureTS, data, offset);
        read(colorWidth, data, offset);
        read(colorHeight, data, offset);
        read(validVerticesCount, data, offset);
        read(nbAudioFrames, data, offset);
        read(encodedColorDataSize, data, offset);
        read(encodedcloudDataSize, data, offset);

        // resize arrays
        audioFrames.resize(nbAudioFrames);
        encodedColorData.resize(encodedColorDataSize);
        encodedCloudData.resize(encodedcloudDataSize);

        // copy imusample
        read(imuSample, data, offset);

        // copy audio
        if(nbAudioFrames > 0){
            read_array(audioFrames.data()->data(), data, nbAudioFrames*7, offset);
        }
        // copy color
        if(encodedColorDataSize > 0){
            read_array(encodedColorData.data(), data, encodedColorDataSize, offset);
        }
        // copy cloud
        if(encodedcloudDataSize > 0){
            read_array(encodedCloudData.data(), data, encodedcloudDataSize, offset);
        }
    }

    size_t convert_to_data(std::vector<std::int8_t> &data){

        size_t totalDataSize = total_data_size();
        size_t offset = 0;
        if(data.size() < totalDataSize){
            data.resize(totalDataSize);
        }

        auto dataP = data.data();

        // copy infos
        write(idCapture, dataP, offset);
        write(afterCaptureTS, dataP, offset);
        write(colorWidth, dataP, offset);
        write(colorHeight, dataP, offset);
        write(validVerticesCount, dataP, offset);
        size_t nbAudioFrames    = audioFrames.size();
        write(nbAudioFrames, dataP, offset);
        size_t encColorDataSize = encodedColorData.size();
        write(encColorDataSize, dataP, offset);
        size_t encCloudDataSize = encodedCloudData.size();
        write(encCloudDataSize, dataP, offset);

        // copy imusample
        write(imuSample, dataP, offset);

        // copy audio
        if(nbAudioFrames > 0){
            write_array(audioFrames.data()->data(), dataP, nbAudioFrames*7, offset);
        }
        // copy color
        if(encColorDataSize > 0){
            write_array(encodedColorData.data(),  dataP, encColorDataSize,offset);
        }
        // copy cloud
        if(encCloudDataSize > 0){
            write_array(encodedCloudData.data(), dataP, encCloudDataSize, offset);
        }

        return totalDataSize;
    }
};

/**
 * @brief Compressed kinect4 full frame
 */
struct K4CompressedFullFrame : public K4CompressedFrame{

    K4Mode mode;
    k4a_calibration_t calibration;

    size_t depthWidth = 0;
    size_t depthHeight = 0;
    std::vector<std::uint8_t> encodedDepthData;

    size_t infraWidth = 0;
    size_t infraHeight = 0;
    std::vector<std::uint8_t> encodedInfraData;

    size_t total_data_size() const override{
        // contains also encodedDepthData.size() and encodedInfraData.size()
        return K4CompressedFrame::total_data_size() + sizeof(mode) + sizeof(calibration) +
               sizeof(size_t)*6 + encodedDepthData.size() + encodedInfraData.size();
    }

    K4CompressedFullFrame() = default;
    K4CompressedFullFrame(std::int8_t *data){
        init_from_data(data);
    }

    void init_from_data(std::int8_t *data){

        size_t offset = 0, nbAudioFrames, encodedColorDataSize, encodedDepthDataSize, encodedInfraDataSize;

        // copy infos
        read(idCapture, data, offset);
        read(afterCaptureTS, data, offset);
        read(mode, data, offset);
        read(calibration, data, offset);
        read(colorWidth, data, offset);
        read(colorHeight, data, offset);
        read(depthWidth, data, offset);
        read(depthHeight, data, offset);
        read(infraWidth, data, offset);
        read(infraHeight, data, offset);
        read(validVerticesCount, data, offset);
        read(nbAudioFrames, data, offset);
        read(encodedColorDataSize, data, offset);
        read(encodedDepthDataSize, data, offset);
        read(encodedInfraDataSize, data, offset);

        // resize arrays
        audioFrames.resize(nbAudioFrames);
        encodedColorData.resize(encodedColorDataSize);
        encodedDepthData.resize(encodedDepthDataSize);
        encodedInfraData.resize(encodedInfraDataSize);

        // copy imusample
        read(imuSample, data, offset);

        // copy audio
        if(nbAudioFrames > 0){
            read_array(audioFrames.data()->data(), data, nbAudioFrames*7, offset);
        }
        // copy color
        if(encodedColorDataSize > 0){
            read_array(encodedColorData.data(), data, encodedColorDataSize, offset);
        }
        // copy depth
        if(encodedDepthDataSize > 0){
            read_array(encodedDepthData.data(), data, encodedDepthDataSize, offset);
        }
        // copy infra
        if(encodedInfraDataSize > 0){
            read_array(encodedInfraData.data(), data, encodedInfraDataSize, offset);
        }
    }


    size_t convert_to_data(std::vector<std::int8_t> &data){

        size_t nbAudioFrames    = audioFrames.size();
        size_t encColorDataSize = encodedColorData.size();
        size_t encDepthDataSize = encodedDepthData.size();
        size_t encInfraDataSize = encodedInfraData.size();

        size_t totalDataSize = total_data_size();

        size_t offset = 0;
        if(data.size() < totalDataSize){
            data.resize(totalDataSize);
        }

        auto dataP = data.data();

        // copy infos
        write(idCapture, dataP, offset);
        write(afterCaptureTS, dataP, offset);
        write(mode, dataP, offset);
        write(calibration, dataP, offset);
        write(colorWidth, dataP, offset);
        write(colorHeight, dataP, offset);
        write(depthWidth, dataP, offset);
        write(depthHeight, dataP, offset);
        write(infraWidth, dataP, offset);
        write(infraHeight, dataP, offset);
        write(validVerticesCount, dataP, offset);
        write(nbAudioFrames, dataP, offset);
        write(encColorDataSize, dataP, offset);
        write(encDepthDataSize, dataP, offset);
        write(encInfraDataSize, dataP, offset);
        write(imuSample, dataP, offset);

        // copy audio
        if(nbAudioFrames > 0){
            write_array(audioFrames.data()->data(), dataP, nbAudioFrames*7, offset);
        }
        // copy color
        if(encColorDataSize > 0){
            write_array(encodedColorData.data(), dataP, encColorDataSize, offset);
        }
        // copy depth
        if(encDepthDataSize > 0){
            write_array(encodedDepthData.data(), dataP, encDepthDataSize, offset);
        }
        // copy infra
        if(encInfraDataSize > 0){
            write_array(encodedInfraData.data(), dataP, encInfraDataSize, offset);
        }

        return totalDataSize;
    }

};

/**
 * @brief Display kinect4 frame to be used in clients
 */
struct K4DisplayFrame : Frame{
    PixelsFrame colorFrame;
    PixelsFrame depthFrame;
    PixelsFrame infraredFrame;
    ColoredCloudFrame cloud;
    std::vector<std::array<float, 7>> audioFrames;
    K4ImuSample imuSample;
};

/**
 * @brief Uncompressed kinect4 cloud frame
 */
struct K4CloudFrame : Frame{
    ColoredCloudFrame cloud;
};

/**
 * @brief Uncompressed kinect4 full frame
 */
struct K4FullFrame : Frame{
    // sizes
    size_t colorWidth = 0;
    size_t colorHeight = 0;
    size_t depthWidth = 0;
    size_t depthHeight = 0;
    size_t infraWidth = 0;
    size_t infraHeight = 0;
    // raw
    std::vector<std::uint16_t> rawDepthData;
    std::vector<std::uint16_t> rawInfraData;
    // image
    std::vector<geo::Pt3<std::uint8_t>> imageColorData;
    std::vector<geo::Pt3<std::uint8_t>> imageDepthData;
    std::vector<geo::Pt3<std::uint8_t>> imageInfraData;
    // # cloud data
    ColoredCloudFrame cloud;
};


}
