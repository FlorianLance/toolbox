
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
#include "k4_volumetric_cloud_video_resource.hpp"
#include "k4_frame_uncompressor.hpp"


namespace tool::camera{

class K4VolumetricCloudVideoManager{
public:

    K4VolumetricCloudVideoManager(K4VolumetricCloudVideoResource *volumetricVideo);
    ~K4VolumetricCloudVideoManager();

    // uncompress cloud frame
    bool uncompress_frame(K4CompressedCloudFrame *cFrame, K4CloudFrame &frame);

    // copy audio
    void audio_samples_all_channels(size_t idCamera, std::vector<std::vector<float>> &audioBuffer);
    void audio_samples_all_channels(size_t idCamera, std::vector<float> &audioBuffer);

    K4VolumetricCloudVideoResource *vv = nullptr;
    K4CloudFrameUncompressor cfu;
private:

    struct Impl;
    std::unique_ptr<Impl> m_p = nullptr;

};
}
