
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
#include "volumetric_full_video_resource.hpp"
#include "frame_uncompressor.hpp"

namespace tool::camera::K4{

class VolumetricFullVideoManager{
public:

    VolumetricFullVideoManager(VolumetricFullVideoResource *volumetricVideo);
    ~VolumetricFullVideoManager();

    // data
    geo::Pt3<int16_t>* cloud_data();

    // uncompress full frame
    bool uncompress_frame(CompressedFullFrame *cFrame, FullFrame &frame);

    // copy audio
    void audio_samples_all_channels(size_t idCamera, std::vector<std::vector<float>> &audioBuffer);
    void audio_samples_all_channels(size_t idCamera, std::vector<float> &audioBuffer);

    // convert to images
    void convert_to_depth_image(
        Mode mode,
        size_t depthWidth, size_t depthHeight,
        const std::vector<std::uint16_t> &uncompressedDepth,
        std::vector<std::uint8_t> &imageDepth);

    void convert_to_infra_image(
        size_t infraWidth, size_t infraHeight,
        const std::vector<std::uint16_t> &uncompressedInfra,
        std::vector<std::uint8_t> &imageInfra);

    // convert to cloud
    size_t convert_to_cloud(
        size_t validVerticesCount,
        const std::vector<std::uint8_t> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        tool::camera::K4::ColoredCloudFrame &cloud);

    size_t convert_to_cloud(
        const std::vector<std::uint8_t> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        geo::Pt3f *vertices, geo::Pt3f *colors);

    size_t convert_to_cloud(
        const std::vector<std::uint8_t> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        geo::Pt3f *vertices, geo::Pt4f *colors);

    size_t convert_to_cloud(
        const std::vector<std::uint8_t> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        geo::Pt3f *vertices, geo::Pt4<std::uint8_t> *colors);

    size_t convert_to_cloud(
        const std::vector<std::uint8_t> &uncompressedColor,
        const std::vector<std::uint16_t> &uncompressedDepth,
        VertexMeshData *vertices);

    // process
    void process_open3d_cloud(const std::vector<std::uint8_t> &uncompressedColor);
    void register_frames(size_t idCamera, size_t startFrame, size_t endFrame, double voxelDownSampleSize);
    void voxelize_registered_frames(double voxelSize, tool::camera::K4::ColoredCloudFrame &cloud);


    VolumetricFullVideoResource *vv = nullptr;
    FullFrameUncompressor ffu;

private:

    struct Impl;
    std::unique_ptr<Impl> m_p = nullptr;
    static constexpr std::array<geo::Pt3f, 5> depthGradient ={
        geo::Pt3f{0.f,0.f,1.f},
        {0.f,1.f,1.f},
        {0.f,1.f,0.f},
        {1.f,1.f,0.f},
        {1.f,0.f,0.f},
    };
};
}
