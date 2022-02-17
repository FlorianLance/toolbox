

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

#include "k4_volumetric_full_video_manager.hpp"

// std
#include <execution>

// open3d
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/VoxelGrid.h"
//#include "open3d/io/VoxelGridIO.h"
#include "open3d/pipelines/registration/ColoredICP.h"

// local
#include "utility/logger.hpp"
#include "k4_frame_uncompressor.hpp"

using namespace tool::geo;
using namespace tool::camera;

struct K4VolumetricFullVideoManager::Impl{


    std::vector<size_t> indicesDepths1D;

    open3d::geometry::PointCloud totalCloud;

    void convert_cloud_frame_to_open3d_point_cloud(const tool::camera::ColoredCloudFrame &cloudFrame, open3d::geometry::PointCloud &open3dPC){

        open3dPC.points_.resize(cloudFrame.validVerticesCount);
        open3dPC.colors_.resize(cloudFrame.validVerticesCount);

        for(size_t ii = 0; ii < cloudFrame.vertices.size(); ++ii){
            const auto &pt = cloudFrame.vertices[ii];
            const auto &c  = cloudFrame.colors[ii];
            open3dPC.points_[ii] = Eigen::Vector3d{
                static_cast<double>(pt.x()),
                static_cast<double>(pt.y()),
                static_cast<double>(pt.z())
            };
            open3dPC.colors_[ii] = Eigen::Vector3d{
                static_cast<double>(c.x()),
                static_cast<double>(c.y()),
                static_cast<double>(c.z())
            };
        }
    }

    Impl(){
    }
};

K4VolumetricFullVideoManager::K4VolumetricFullVideoManager(K4VolumetricFullVideoResource *volumetricVideo) : m_p(std::make_unique<K4VolumetricFullVideoManager::Impl>()){
    vv = volumetricVideo;
}

K4VolumetricFullVideoManager::~K4VolumetricFullVideoManager(){
}


void K4VolumetricFullVideoManager::audio_samples_all_channels(size_t idCamera, std::vector<std::vector<float>> &audioBuffer){

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

void K4VolumetricFullVideoManager::audio_samples_all_channels(size_t idCamera, std::vector<float> &audioBuffer){

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


Pt3<int16_t> *K4VolumetricFullVideoManager::cloud_data(){
    return ffu.cloud_data();
}

bool K4VolumetricFullVideoManager::uncompress_frame(K4CompressedFullFrame *cFrame, K4FullFrame &frame){
    return ffu.uncompress(cFrame,frame);
}


void K4VolumetricFullVideoManager::register_frames(size_t idCamera, size_t startFrame, size_t endFrame, double voxelDownSampleSize){

    if(idCamera >= vv->nb_cameras() || startFrame >= endFrame){
        Logger::error("[VolumetricFullVideoManager::voxelize_all_frames] Invalid input\n");
        return;
    }

    std::vector<geo::Pt3<uint8_t>> uncompressedColor;
    std::vector<uint16_t> uncompressedDepth;
    open3d::geometry::PointCloud frameCloud;

    std::shared_ptr<open3d::geometry::PointCloud> previousDownSampledCloud = nullptr;
    geo::Mat4d totalTr = geo::Mat4d::identity();

    auto frames = vv->get_frames(idCamera);

    // count total vertices
    size_t totalFramesVertices = 0;
    for(size_t ii = startFrame; ii < endFrame; ++ii){
        const auto &frame = (*frames)[ii];

        if(!frame.data){
            return;
        }

        totalFramesVertices += frame.data->validVerticesCount;
        // Logger::message(std::format("frame {} {} \n", data->validVerticesCount, totalFramesVertices));
    }
    // init total cloud
    m_p->totalCloud.points_.resize(totalFramesVertices);
    m_p->totalCloud.colors_.resize(totalFramesVertices);

    // process frames
    //    Logger::message("process frames\n");
    size_t currentVerticesId = 0;
    for(size_t ii = startFrame; ii < endFrame; ++ii){


        const auto &frame = (*frames)[ii];
        auto data = reinterpret_cast<K4CompressedFullFrame*>(frame.data.get());

        ffu.uncompress_color(data, uncompressedColor);
        ffu.uncompress_depth(data, uncompressedDepth);
        ffu.generate_cloud(data, uncompressedDepth);

        auto cloudBuffer =  cloud_data();

        // resize depth indices
        if(m_p->indicesDepths1D.size() != uncompressedDepth.size()){
            m_p->indicesDepths1D.resize(uncompressedDepth.size());
            std::iota(std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), 0);
        }

        // resize cloud
        frameCloud.points_.resize(frame.data->validVerticesCount);
        frameCloud.colors_.resize(frame.data->validVerticesCount);

        // update cloud values
        size_t idV = 0;
        for_each(std::execution::unseq, std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), [&](size_t id){

            if(uncompressedDepth[id] == k4_invalid_depth_value){
                return;
            }

            frameCloud.points_[idV] = Eigen::Vector3d{
                static_cast<double>(-cloudBuffer[id].x()),
                static_cast<double>(-cloudBuffer[id].y()),
                static_cast<double>( cloudBuffer[id].z())
            }*0.01;
            frameCloud.colors_[idV] = Eigen::Vector3d{
                static_cast<double>(uncompressedColor[id].x()),
                static_cast<double>(uncompressedColor[id].y()),
                static_cast<double>(uncompressedColor[id].z())
            }/255.0;

            ++idV;
        });

        //  Logger::message(std::format("cloud set {} {}\n", idV, data->validVerticesCount));
        //  Logger::message("down sample cloud\n");
        auto opend3dDownCloud = frameCloud.VoxelDownSample(voxelDownSampleSize);
        double radius     = 0.1; // radius of the search
        int maxNeighbours = 30;  // max neighbours to be searched
        opend3dDownCloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius, maxNeighbours));

        //  Logger::message(std::format("{} {}\n", frameCloud.points_.size(), opend3dDownCloud->points_.size()));

        geo::Mat4d tr = geo::Mat4d::identity();
        if(previousDownSampledCloud != nullptr){

            Logger::message("Color icp\n");
            double maxDistance = 0.4;
            auto estimation = open3d::pipelines::registration::TransformationEstimationForColoredICP();
            auto criteria   = open3d::pipelines::registration::ICPConvergenceCriteria();

            auto registrationResult = open3d::pipelines::registration::RegistrationColoredICP(
                *opend3dDownCloud, *previousDownSampledCloud, maxDistance,
                Eigen::Matrix4d::Identity(),
                estimation, criteria
                );
            for(int ii = 0; ii < 4; ++ii){
                for(int jj = 0; jj < 4; ++jj){
                    tr.at(ii,jj) = static_cast<double>(registrationResult.transformation_(ii,jj));
                }
            }
            tr = transpose(tr);
            Logger::message(std::format("fitness {} inlier rmse {}\n",registrationResult.fitness_, registrationResult.inlier_rmse_));
            //            std::cout << tr << "\n";
        }
        previousDownSampledCloud = opend3dDownCloud;

        // cumul previous tr
        totalTr = totalTr * tr;

        // Logger::message("add transformed points to total cloud\n");
        // add tranformed points to total cloud
        for(size_t ii = 0; ii < frame.data->validVerticesCount; ++ii){
            const auto &p = frameCloud.points_[ii];
            auto trP = totalTr.multiply_point(geo::Pt3d{p.x(),p.y(),p.z()});
            m_p->totalCloud.points_[currentVerticesId]   = Eigen::Vector3d{trP.x(), trP.y(), trP.z()};
            m_p->totalCloud.colors_[currentVerticesId++] = frameCloud.colors_[ii];
        }

        size_t start = currentVerticesId - frame.data->validVerticesCount;
        size_t end  = currentVerticesId;
        std::string trFrameCloudStr = std::format("./tr_cloud_part_{}.obj", frame.idFrame);
        std::string frameCloudStr = std::format("./frame_cloud_part_{}.obj", frame.idFrame);
        std::string downSampledFrameCloudStr = std::format("./down_sampled_frame_cloud_part_{}.obj",frame.idFrame);

        //        auto startDP = reinterpret_cast<geo::Pt3d*>(totalCloud.points_.data());
        //        auto startDC = reinterpret_cast<geo::Pt3d*>(totalCloud.colors_.data());
        //        files::CloudIO::save_cloud(trFrameCloudStr, startDP + start, startDC + start, data->validVerticesCount);

        //        startDP = reinterpret_cast<geo::Pt3d*>(frameCloud.points_.data());
        //        startDC = reinterpret_cast<geo::Pt3d*>(frameCloud.colors_.data());
        //        files::CloudIO::save_cloud(frameCloudStr, startDP, startDC, data->validVerticesCount);

        auto startDP = reinterpret_cast<geo::Pt3d*>(opend3dDownCloud->points_.data());
        auto startDC = reinterpret_cast<geo::Pt3d*>(opend3dDownCloud->colors_.data());
        //files::CloudIO::save_cloud(downSampledFrameCloudStr, startDP, startDC, opend3dDownCloud->points_.size());

        Logger::message("end frame\n");
    }
}

void K4VolumetricFullVideoManager::voxelize_registered_frames(double voxelSize, ColoredCloudFrame &cloud){

    // voxelize
    auto voxelGrid = open3d::geometry::VoxelGrid::CreateFromPointCloudWithinBounds(
        m_p->totalCloud, static_cast<double>(voxelSize),
        Eigen::Vector3d{-1,-1,-1},
        Eigen::Vector3d{1,1,1}
    );

    // process grid
    // ...

    // voxelGrid->GetMinBound();
    // voxelGrid->GetMaxBound();
    // voxelGrid->GetCenter();
    // voxelGrid->voxels_.size();
    // voxelGrid->GetOrientedBoundingBox();

    // init cloud from voxel grid
    Logger::message("init cloud from voxel grid\n");
    const auto gridSize = voxelGrid->voxels_.size();
    cloud.vertices.resize(gridSize);
    cloud.colors.resize(gridSize);

    const geo::Pt3f min{-1.f,-1.f,-1.f};

    size_t idVoxel = 0;
    for(const auto &voxel : voxelGrid->voxels_){
        const auto &v  = std::get<1>(voxel);
        const auto &gi = v.grid_index_;
        const auto &c  = v.color_;
        cloud.vertices[idVoxel] = (min + geo::Pt3f{static_cast<float>(gi.x()), static_cast<float>(gi.y()), static_cast<float>(gi.z())})
                                  *static_cast<float>(voxelSize);
        cloud.colors[idVoxel++] = {static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z())};
    }
    cloud.validVerticesCount = gridSize;

    Logger::message(std::format("gridSize {}\n", gridSize));
}

void K4VolumetricFullVideoManager::voxelize(ColoredCloudFrame &cloudF, float gridVoxelSize){

    if(cloudF.validVerticesCount == 0){
        return;
    }

    open3d::geometry::PointCloud pc;
    m_p->convert_cloud_frame_to_open3d_point_cloud(cloudF, pc);

    auto res = open3d::geometry::VoxelGrid::CreateFromPointCloudWithinBounds(
        pc, static_cast<double>(gridVoxelSize),
        Eigen::Vector3d{-1,-1,-1},
        Eigen::Vector3d{1,1,1}
    );
    geo::Pt3f min{1.f,1.f,1.f};

    // Logger::message(std::format("voxelize {} {} \n", res->voxels_.size(),pc.points_.size() ));

    size_t id = 0;
    const auto voxels = res->GetVoxels();
    for(const auto &v : voxels){
        const auto &gi = v.grid_index_;
        const auto &c = v.color_;
        cloudF.vertices[id] = (min + geo::Pt3f{static_cast<float>(gi.x()), static_cast<float>(gi.y()), static_cast<float>(gi.z())}) * gridVoxelSize;
        cloudF.colors[id++] = {static_cast<float>(c.x()), static_cast<float>(c.y()), static_cast<float>(c.z())};
    }
    cloudF.validVerticesCount = voxels.size();
}




