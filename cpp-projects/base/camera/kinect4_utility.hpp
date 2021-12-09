
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
#include "geometry/matrix4.hpp"
#include "kinect4_data_types.hpp"

namespace tool::camera::K4{

class CompressedFramesManager{
public:
    CompressedFramesManager();
    ~CompressedFramesManager();

    size_t nb_cameras() const noexcept;
    size_t nb_frames(size_t idCamera = 0) const noexcept;
    CompressedDataFrame *get_frame(size_t idFrame, size_t idCamera = 0);
    size_t frame_id(size_t idCamera, float timeMs);
    size_t valid_vertices_count(size_t idFrame, size_t idCamera = 0);

    void add_frame(size_t idCamera, std::int64_t timestamp, std::shared_ptr<CompressedDataFrame> frame);
    void clean_frames();
    void set_transform(size_t idCamera, geo::Mat4d tr);
    geo::Mat4d get_transform(size_t idCamera) const;

    std::int64_t start_time(size_t idCamera) const;
    std::int64_t end_time(size_t idCamera) const;

    bool save_to_file(const std::string &path);
    bool load_from_file(const std::string &path);

    bool uncompress_color(CompressedDataFrame *cFrame, std::vector<std::uint8_t> &uncompressedColor);
    bool uncompress_depth(CompressedDataFrame *cFrame, std::vector<std::uint16_t> &uncompressedDepth);
    bool uncompress_infra(CompressedDataFrame *cFrame, std::vector<std::uint16_t> &uncompressedInfra);

    void audio_samples_all_channels(size_t idCamera, std::vector<std::vector<float>> &audioBuffer);
    void audio_samples_all_channels(size_t idCamera, std::vector<float> &audioBuffer);


    void convert_to_depth_image(
        Mode mode,
        CompressedDataFrame *cFrame,
        const std::vector<std::uint16_t> &uncompressedDepth,
        std::vector<std::uint8_t> &imageDepth);

    void convert_to_infra_image(
        CompressedDataFrame *cFrame,
        const std::vector<std::uint16_t> &uncompressedInfra,
        std::vector<std::uint8_t> &imageInfra);

    void generate_cloud(CompressedDataFrame *cFrame,
        const std::vector<std::uint16_t> &uncompressedDepth);

    void process_open3d_cloud(const std::vector<std::uint8_t> &uncompressedColor);

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


    void register_frames(size_t idCamera, size_t startFrame, size_t endFrame, double voxelDownSampleSize);
    void voxelize(double voxelSize, tool::camera::K4::ColoredCloudFrame &cloud);

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
