

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

#include "volumetric_cloud_video_manager.hpp"

// std
#include <execution>

// open3d
//#include "open3d/geometry/PointCloud.h"
//#include "open3d/geometry/VoxelGrid.h"
//#include "open3d/pipelines/registration/ColoredICP.h"

// local
#include "utility/logger.hpp"

using namespace tool::geo;
using namespace tool::camera::K4;

struct VolumetricCloudVideoManager::Impl{

    std::vector<size_t> indicesDepths1D;

//    open3d::geometry::PointCloud open3dPointCloud;
//    open3d::geometry::PointCloud totalCloud;

    Impl(){
    }
};

VolumetricCloudVideoManager::VolumetricCloudVideoManager(VolumetricCloudVideoResource *volumetricVideo) : m_p(std::make_unique<VolumetricCloudVideoManager::Impl>()){
    vv = volumetricVideo;
}

VolumetricCloudVideoManager::~VolumetricCloudVideoManager(){
}

bool VolumetricCloudVideoManager::uncompress_frame(CompressedCloudFrame *cFrame, CloudFrame &uFrame){
    return cfu.uncompress(cFrame, uFrame);
}


void VolumetricCloudVideoManager::audio_samples_all_channels(size_t idCamera, std::vector<std::vector<float>> &audioBuffer){

    auto frames = vv->get_frames(idCamera);
    size_t samplesCount = 0;
    for(const auto &frame : *frames){
        samplesCount += frame.data->audioFrames.size();
    }

    audioBuffer.resize(7);
    for(auto &channelAudioBuffer : audioBuffer){
        channelAudioBuffer.reserve(samplesCount);
    }

    for(const auto &frame : *frames){
        for(size_t idChannel = 0; idChannel < 7; ++idChannel){
            for(const auto &channelsData : frame.data->audioFrames){
                audioBuffer[idChannel].push_back(channelsData[idChannel]);
            }
        }
    }
}

void VolumetricCloudVideoManager::audio_samples_all_channels(size_t idCamera, std::vector<float> &audioBuffer){

    auto frames = vv->get_frames(idCamera);
    size_t samplesCount = 0;
    for(const auto &frame : *frames){
        samplesCount += frame.data->audioFrames.size();
    }
    audioBuffer.resize(samplesCount*7);

    size_t id = 0;
    for(const auto &frame : *frames){
        for(const auto &channelsData : frame.data->audioFrames){
            for(size_t idChannel = 0; idChannel < 7; ++idChannel){
                audioBuffer[id++] = channelsData[idChannel];
            }
        }
    }
}
