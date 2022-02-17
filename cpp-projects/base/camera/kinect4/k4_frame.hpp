
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

namespace tool::camera{

// # display data frame (to be displayed in a client)
struct K4DisplayDataFrame{
    PixelsFrame colorFrame;
    PixelsFrame depthFrame;
    PixelsFrame infraredFrame;
    ColoredCloudFrame cloud;
    std::vector<std::array<float, 7>> audioFrames;
    K4ImuSample imuSample;

};

// compressed common frame
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

// compressed cloud frame (to be sended throught network or saved)
struct K4CompressedCloudFrame : public K4CompressedFrame{
    std::vector<std::uint8_t> encodedCloudData;
    // contains also encodedCloudData.size()
    size_t total_data_size() const override{
        return K4CompressedFrame::total_data_size() + encodedCloudData.size() + sizeof(size_t);
    }
};

// compressed full frame (to be saved)
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
};


// uncompressed
struct K4CloudFrame : Frame{
    tool::camera::ColoredCloudFrame cloud;
};

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
    tool::camera::ColoredCloudFrame cloud;
};




}
