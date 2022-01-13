
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
#include "kinect4_data.hpp"

namespace tool::camera::K4{

struct FrameUncompressor{

    FrameUncompressor();
    ~FrameUncompressor();

    // uncompress
    bool uncompress_color(
        size_t colorWidth, size_t colorHeight,
        const std::vector<std::uint8_t> &encoded,
        std::vector<geo::Pt3<uint8_t>> &uncompressedColor);

    // convert
    void convert_to_depth_image(Mode mode, size_t depthWidth, size_t depthHeight,
        const std::vector<uint16_t> &uncompressedDepth,
        std::vector<geo::Pt3<uint8_t>> &imageDepth);

    void convert_to_infra_image(
        size_t infraWidth, size_t infraHeight,
        const std::vector<uint16_t> &uncompressedInfra,
        std::vector<geo::Pt3<uint8_t>> &imageInfra);

    static constexpr std::array<geo::Pt3f, 5> depthGradient ={
        geo::Pt3f{0.f,0.f,1.f},
        {0.f,1.f,1.f},
        {0.f,1.f,0.f},
        {1.f,1.f,0.f},
        {1.f,0.f,0.f},
    };

protected:
    std::vector<size_t> indicesDepths1D;

private:
    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};


struct CloudFrameUncompressor : public FrameUncompressor{

    // all steps
    bool uncompress(CompressedCloudFrame *cFrame, CloudFrame &uFrame);
    bool uncompress(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3f *colors);
    bool uncompress(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4f *colors);
    bool uncompress(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3<std::uint8_t> *colors);
    bool uncompress(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4<std::uint8_t> *colors);
    bool uncompress(CompressedCloudFrame *cFrame, VertexMeshData *vertices);

private:

    // process
    bool uncompress_color(CompressedCloudFrame *cFrame, std::vector<geo::Pt3<uint8_t>> &uncompressedColor);
    bool uncompress_vertices(CompressedCloudFrame *cFrame, std::vector<std::uint16_t> &decodedVertices);
    bool uncompress(CompressedCloudFrame *cFrame);
    // convert
    void update_id_array(size_t idV);
    void convert_to_cloud(CompressedCloudFrame *cFrame, CloudFrame &uFrame);
    void convert_to_cloud(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3f *colors);
    void convert_to_cloud(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4f *colors);
    void convert_to_cloud(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3<std::uint8_t> *colors);
    void convert_to_cloud(CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4<std::uint8_t> *colors);
    void convert_to_cloud(CompressedCloudFrame *cFrame, VertexMeshData *vertices);

    std::vector<size_t> indicesValid1D;
    std::vector<std::uint16_t> decodedVerticesData;
    std::vector<geo::Pt3<std::uint8_t>> decodedColorData;
};

struct FullFrameUncompressor : public FrameUncompressor{

    FullFrameUncompressor();
    ~FullFrameUncompressor();

    // per step
    bool uncompress_color(CompressedFullFrame *cFrame, std::vector<geo::Pt3<uint8_t>> &uncompressedColor);
    bool uncompress_depth(CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedDepth);
    bool uncompress_infra(CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedInfra);
    void generate_cloud(CompressedFullFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth);

    // all steps
    bool uncompress(CompressedFullFrame *cFrame, FullFrame &fframe);

    geo::Pt3<int16_t>* cloud_data();

    size_t convert_to_cloud(
        size_t validVerticesCount,
        const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        tool::camera::K4::ColoredCloudFrame &cloud);

    size_t convert_to_cloud(
        const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        geo::Pt3f *vertices, geo::Pt3f *colors);

    size_t convert_to_cloud(
        const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        geo::Pt3f *vertices, geo::Pt4f *colors);

    size_t convert_to_cloud(
        const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        geo::Pt3f *vertices, geo::Pt4<std::uint8_t> *colors);

    size_t convert_to_cloud(
        const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        VertexMeshData *vertices);


private:

    struct Impl;
    std::unique_ptr<Impl> i = nullptr;

};
}









