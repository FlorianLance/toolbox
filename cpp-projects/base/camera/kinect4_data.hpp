
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
#include "kinect4_types.hpp"
#include "geometry/point3.hpp"

namespace tool::camera::K4{

    struct VoxelData{
        std::int64_t idX : 13, idY : 13, idZ : 14, r : 8, g : 8, b : 8;
    };

    struct PackedVoxel{
        static std::tuple<std::uint32_t,std::uint32_t> pack(const geo::Pt3<std::int16_t> &pos, const geo::Pt4<std::uint8_t> &col) noexcept;
        static void unpack(std::uint32_t p1, std::uint32_t p2, geo::Pt3<std::int16_t> &pos, geo::Pt3<std::uint8_t> &col) noexcept;
        static std::uint64_t pack64(const geo::Pt3<std::int16_t> &pos, const geo::Pt4<std::uint8_t> &col);
        static void unpack64(std::uint64_t p, geo::Pt3<std::int16_t> &pos, geo::Pt4<std::uint8_t> &col);
        static geo::Pt4<std::uint8_t> pack_xy(std::int16_t x, std::int16_t y);
    };

    struct VertexMeshData{
        geo::Pt3f pos;
        geo::Pt4<std::uint8_t> col;
    };

    // display
    // # image display data (color,depth,infrared)
    struct PixelsFrame{
        size_t width = 0;
        size_t height = 0;
        std::vector<geo::Pt3<std::uint8_t>> pixels;
    };

    // colored cloud display data
    struct ColoredCloudFrame{
        size_t validVerticesCount = 0;
        std::vector<geo::Pt3f> vertices;
        std::vector<geo::Pt3f> colors;
        // std::vector<geo::Pt3f> normals;
    };

    // # display data frame (to be displayed in a client)
    struct DisplayDataFrame{
        PixelsFrame colorFrame;
        PixelsFrame depthFrame;
        PixelsFrame infraredFrame;
        ColoredCloudFrame cloud;
        std::vector<std::array<float, 7>> audioFrames;
        ImuSample imuSample;
    };

    // compressed common frame
    struct CompressedFrame{
        size_t idCapture = 0;
        std::int64_t afterCaptureTS = 0;
        size_t validVerticesCount = 0;
        size_t colorWidth = 0;
        size_t colorHeight = 0;
        std::vector<std::uint8_t> encodedColorData;

//        std::vector<std::uint8_t> encodedAudioData; // todo
        std::vector<std::array<float, 7>> audioFrames; // to be removed
        ImuSample imuSample;
    };

    // compressed cloud frame (to be sended throught network or saved)
    struct CompressedCloudFrame : public CompressedFrame{
        std::vector<std::uint8_t> encodedCloudData;
    };

    // compressed full frame (to be saved)
    struct CompressedFullFrame : public CompressedFrame{

        Mode mode;
        k4a_calibration_t calibration;

        size_t depthWidth = 0;
        size_t depthHeight = 0;
        std::vector<std::uint8_t> encodedDepthData;

        size_t infraWidth = 0;
        size_t infraHeight = 0;
        std::vector<std::uint8_t> encodedInfraData;
    };

    struct CloudFrame{        
        tool::camera::K4::ColoredCloudFrame cloud;
    };

    // uncompressed
    struct FullFrame{
        // raw
        std::vector<std::uint16_t> rawDepthData;
        std::vector<std::uint16_t> rawInfraData;
        // image
        std::vector<geo::Pt3<std::uint8_t>> imageColorData;
        std::vector<geo::Pt3<std::uint8_t>> imageDepthData;
        std::vector<geo::Pt3<std::uint8_t>> imageInfraData;
        // # cloud data
        tool::camera::K4::ColoredCloudFrame cloud;
    };
}









