
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

    // process
    void register_frames(size_t idCamera, size_t startFrame, size_t endFrame, double voxelDownSampleSize);
    void voxelize_registered_frames(double voxelSize, tool::camera::K4::ColoredCloudFrame &cloud);
    void voxelize(FullFrame &frame, float gridVoxelSize);

    VolumetricFullVideoResource *vv = nullptr;
    FullFrameUncompressor ffu;

private:

    struct Impl;
    std::unique_ptr<Impl> m_p = nullptr;

};
}
