

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


#include "kinect4_utility.hpp"

// std
#include <execution>
#include <iostream>

// turbojpg
#include <turbojpeg.h>

// kinect4
#include <k4a/k4a.hpp>

// open3d
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/VoxelGrid.h"
#include "open3d/io/VoxelGridIO.h"
#include "open3d/pipelines/registration/ColoredICP.h"

// local
#include "files/cloud_io.hpp"
#include "utility/io.hpp"
#include "utility/logger.hpp"
#include "data/integers_encoder.hpp"

using namespace tool::geo;
using namespace tool::camera::K4;


struct VolumetricVideoResource::Impl{
    VolumetricVideoMode mode = VolumetricVideoMode::Clouds;
    std::vector<CameraData> camData;

    void read_file(std::ifstream &file){

        // read mode
//        std::int8_t mode;
        read(file, &mode);
//        mode = static_cast<VolumetricVideoMode>(mode);

        // read nb of cameras
        std::int8_t nbCameras;
        read(file, &nbCameras);

        camData.resize(nbCameras);

        // read infos per camera
        std::int32_t nbFrames;
        for(auto &cameraData : camData){

            // read nb frames
            read(file, &nbFrames);

            // create frames
            cameraData.frames.reserve(nbFrames);
            for(size_t ii = 0; ii < static_cast<size_t>(nbFrames); ++ii){
                cameraData.frames.emplace_back(FrameData{0,0, std::make_shared<CompressedDataFrame>()});
            }

            // calibration matrix
            read_array(file, cameraData.transform.array.data(), 16);
        }

        // read frames
        size_t totalColorCompressed   = 0;
        size_t totalColorUncompressed = 0;
        size_t totalDepthCompressed   = 0;
        size_t totalDepthUncompressed = 0;
        size_t totalInfraCompressed   = 0;
        size_t totalInfraUncompressed = 0;

        for(auto &cameraData : camData){
            for(auto &frame : cameraData.frames){
                // read frame
                auto fData = frame.data.get();
                std::int32_t idFrame;
                std::int64_t timestamp;
                // # read frame info
                read(file, &idFrame);
                frame.idFrame = idFrame;
                read(file, &timestamp);
                frame.timeStamp = timestamp;

                if(mode == VolumetricVideoMode::Clouds){

                    read(file, &fData->mode);
                    //Logger::message(std::format("id frame {} ts {}\n", idFrame, timestamp));
                    read_array(file, reinterpret_cast<char*>(&fData->calibration), sizeof (k4a_calibration_t));
                    std::int32_t validVerticesCount;
                    read(file, &validVerticesCount);
                    fData->validVerticesCount = validVerticesCount;
                    // # read color
                    std::int16_t colorWidth, colorHeight;
                    std::int32_t colorBufferSize;
                    read(file, &colorWidth);
                    read(file, &colorHeight);
                    read(file, &colorBufferSize);
                    // Logger::message(std::format("colors sizes {} {} {} \n", (int)colorWidth, (int)colorHeight, colorBufferSize));
                    fData->colorWidth  = colorWidth;
                    fData->colorHeight = colorHeight;
                    fData->colorBuffer.resize(colorBufferSize);
                    totalColorUncompressed += colorWidth*colorHeight;
                    totalColorCompressed   += colorBufferSize;
                    read_array(file, fData->colorBuffer.data(), fData->colorBuffer.size());
                    // # read depth
                    std::int16_t depthWidth, depthHeight;
                    std::int32_t depthBufferSize;
                    read(file, &depthWidth);
                    read(file, &depthHeight);
                    read(file, &depthBufferSize);
                    // Logger::message(std::format("depth sizes {} {} {} \n", (int)depthWidth, (int)depthHeight, depthBufferSize));
                    fData->depthWidth  = depthWidth;
                    fData->depthHeight = depthHeight;
                    fData->depthBuffer.resize(depthBufferSize);
                    totalDepthUncompressed += depthWidth*depthHeight;
                    totalDepthCompressed   += depthBufferSize;
                    read_array(file, fData->depthBuffer.data(), fData->depthBuffer.size());
                    // # read infra
                    std::int16_t infraWidth, infraHeight;
                    std::int32_t infraBufferSize;
                    read(file, &infraWidth);
                    read(file, &infraHeight);
                    read(file, &infraBufferSize);
                    // Logger::message(std::format("infra sizes {} {} {} \n", (int)infraWidth, (int)infraHeight, infraBufferSize));
                    fData->infraWidth  = infraWidth;
                    fData->infraHeight = infraHeight;
                    fData->infraBuffer.resize(infraBufferSize);
                    totalInfraUncompressed += infraWidth*infraHeight;
                    totalInfraCompressed   += infraBufferSize;
                    read_array(file, fData->infraBuffer.data(), fData->infraBuffer.size());

                }else if(mode == VolumetricVideoMode::Voxels){

                    std::int32_t voxelsBufferSize;
                    read(file, &voxelsBufferSize);
                    fData->voxelsBuffer.resize(voxelsBufferSize);
                    read_array(file, fData->voxelsBuffer.data(), fData->voxelsBuffer.size());
                }

                // # read audio
                std::int32_t audioBufferSize;
                read(file, &audioBufferSize);
                fData->audioFrames.resize(audioBufferSize);
                read_array(file, reinterpret_cast<float*>(fData->audioFrames.data()),audioBufferSize*7);
                Logger::message(std::format("audio frames sizes {} \n", (int)audioBufferSize));
                // # read imu
                read_array(file, reinterpret_cast<char*>(&fData->imuSample), sizeof (ImuSample));
            }
        }

        if(totalColorUncompressed != 0 && totalDepthUncompressed != 0 && totalInfraUncompressed != 0){
            auto colCompressionFactor   = (1.0*totalColorCompressed)/totalColorUncompressed;
            auto depthCompressionFactor = (1.0*totalDepthCompressed)/totalDepthUncompressed;
            auto infraCompressionFactor = (1.0*totalInfraCompressed)/totalInfraUncompressed;
            Logger::message(std::format("Compression rates {} {} {}\n", colCompressionFactor, depthCompressionFactor, infraCompressionFactor));
        }
    }



    void write_file(std::ofstream &file){

        // write mode
        write(file, static_cast<std::int8_t>(mode));   // std::int8_t

        // write nb of cameras
        write(file, static_cast<std::int8_t>(camData.size()));             // std::int8_t

        // write infos per camera
        for(auto &cameraData : camData){
            // nb frames
            write(file, static_cast<std::int32_t>(cameraData.frames.size()));   // std::int32_t * cameras count
            // calibration matrix
            write_array(file, cameraData.transform.array.data(), 16);           // double * 16
        }

        // writes frames
        for(const auto &cameraData : camData){
            for(const auto &frame : cameraData.frames){
                // write frame
                auto fData = frame.data.get();
                // # write frame info
                write(file, static_cast<std::int32_t>(frame.idFrame));                      // std::int32_t
                write(file, frame.timeStamp);                                               // std::int64_t

                if(mode == VolumetricVideoMode::Clouds){

                    write(file, static_cast<std::int32_t>(fData->mode));                        // std::int32_t
                    write_array(file, reinterpret_cast<char*>(&fData->calibration),
                                sizeof (k4a_calibration_t));                                    // sizeof(k4a_calibration_t)
                    write(file, static_cast<std::int32_t>(fData->validVerticesCount));          // std::int32_t
                    // # write color
                    write(file, static_cast<std::int16_t>(fData->colorWidth));                  // std::int16_t
                    write(file, static_cast<std::int16_t>(fData->colorHeight));                 // std::int16_t
                    write(file, static_cast<std::int32_t>(fData->colorBuffer.size()));          // std::int32_t
                    write_array(file, fData->colorBuffer.data(), fData->colorBuffer.size());    // buffer size * std::int8_t
                    // # write depth
                    write(file, static_cast<std::int16_t>(fData->depthWidth));                  // std::int16_t
                    write(file, static_cast<std::int16_t>(fData->depthHeight));                 // std::int16_t
                    write(file, static_cast<std::int32_t>(fData->depthBuffer.size()));          // std::int32_t
                    write_array(file, fData->depthBuffer.data(), fData->depthBuffer.size());    // buffer size * std::int32_t
                    // # write infra
                    write(file, static_cast<std::int16_t>(fData->infraWidth));                  // std::int16_t
                    write(file, static_cast<std::int16_t>(fData->infraHeight));                 // std::int16_t
                    write(file, static_cast<std::int32_t>(fData->infraBuffer.size()));          // std::int32_t
                    write_array(file, fData->infraBuffer.data(), fData->infraBuffer.size());    // buffer size * std::int32_t

                }else if(mode == VolumetricVideoMode::Voxels){
                    // # write voxels
                    write(file, static_cast<std::int32_t>(fData->voxelsCount));                 // std::int32_t
                    write_array(file, fData->voxelsBuffer.data(), fData->voxelsBuffer.size());  // buffer size * std::int32_t
                }

                // # write audio
                write(file, static_cast<std::int32_t>(fData->audioFrames.size()));          // std::int32_t
                if(fData->audioFrames.size() > 0){
                    write_array(file, reinterpret_cast<float*>(fData->audioFrames.data()),
                                fData->audioFrames.size()*7);                               // buffer size * 7 * float
                }
                // # write imu
                write_array(file, reinterpret_cast<char*>(&fData->imuSample), sizeof (ImuSample));
            }
        }
    }

};

VolumetricVideoResource::VolumetricVideoResource() : m_p(std::make_unique<VolumetricVideoResource::Impl>()){

}

VolumetricVideoResource::~VolumetricVideoResource(){}

VolumetricVideoMode VolumetricVideoResource::mode() const noexcept{
    return m_p->mode;
}

size_t VolumetricVideoResource::nb_cameras() const noexcept{return m_p->camData.size();}

size_t VolumetricVideoResource::nb_frames(size_t idCamera) const noexcept {
    if(idCamera < m_p->camData.size()){
        return m_p->camData[idCamera].frames.size();
    }
    return 0;
}

CompressedDataFrame *VolumetricVideoResource::get_frame(size_t idFrame, size_t idCamera){
    if(idCamera < m_p->camData.size()){
        if(idFrame < m_p->camData[idCamera].frames.size()){
            return m_p->camData[idCamera].frames[idFrame].data.get();
        }
        Logger::error("[VolumetricVideoResource] Cannot get frame, invalid frame id.\n");
        return nullptr;
    }
    Logger::error("[VolumetricVideoResource] Cannot get frame, invalid camera id.\n");
    return nullptr;
}

std::vector<FrameData> *VolumetricVideoResource::get_frames(size_t idCamera){
    if(idCamera < m_p->camData.size()){
        return &m_p->camData[idCamera].frames;
    }
    Logger::error("[VolumetricVideoResource] Cannot get frame, invalid camera id.\n");
    return nullptr;
}

size_t VolumetricVideoResource::frame_id(size_t idCamera, float timeMs) const{

    const auto start = start_time(idCamera);
    size_t idFrame = 0;
    for(const auto &frame : m_p->camData[idCamera].frames){
        auto frameTimeMs = (frame.timeStamp-start)*0.000001f;
        if(timeMs < frameTimeMs){
            return idFrame;
        }
        idFrame = frame.idFrame;
    }
    return idFrame;
}

size_t VolumetricVideoResource::valid_vertices_count(size_t idFrame, size_t idCamera) const{
    if(idCamera < m_p->camData.size()){
        if(idFrame < m_p->camData[idCamera].frames.size()){
            return m_p->camData[idCamera].frames[idFrame].data->validVerticesCount;
        }
    }
    return 0;
}

std::int64_t VolumetricVideoResource::start_time(size_t idCamera) const{
    if(idCamera < m_p->camData.size()){
        if(m_p->camData[idCamera].frames.size() > 0){
            return m_p->camData[idCamera].frames[0].timeStamp;
        }
    }
    return -1;
}

int64_t VolumetricVideoResource::end_time(size_t idCamera) const{
    if(idCamera < m_p->camData.size()){
        if(m_p->camData[idCamera].frames.size() > 0){
            return m_p->camData[idCamera].frames[m_p->camData[idCamera].frames.size()-1].timeStamp;
        }
    }
    return -1;
}

void VolumetricVideoResource::set_transform(size_t idCamera, geo::Mat4d tr){
    if(idCamera < m_p->camData.size()){
        m_p->camData[idCamera].transform = tr;
        return;
    }
    Logger::error("[VolumetricVideoResource] Cannot set transform, invalid camera id.\n");
}

tool::geo::Mat4d VolumetricVideoResource::get_transform(size_t idCamera) const{
    if(idCamera < m_p->camData.size()){
        return m_p->camData[idCamera].transform;
    }
    Logger::error("[VolumetricVideoResource] Cannot get transform, invalid camera id.\n");
    return geo::Mat4d::identity();
}

void VolumetricVideoResource::add_frame(size_t idCamera, std::int64_t timestamp, std::shared_ptr<CompressedDataFrame> frame){
    if(idCamera >= m_p->camData.size()){
        m_p->camData.resize(idCamera+1);
    }
    auto idFrame = m_p->camData[idCamera].frames.size();
    m_p->camData[idCamera].frames.emplace_back(FrameData{idFrame, timestamp, frame});
}

void VolumetricVideoResource::clean_frames(){
    m_p->camData.clear();
}


bool VolumetricVideoResource::save_to_file(const std::string &path){

    if(path.length() == 0){
        Logger::error("[VolumetricVideoResource] Invalid path.\n");
        return false;
    }

    if(m_p->camData.size() == 0){
        Logger::error("[VolumetricVideoResource] No frames added.\n");
        return false;
    }

    std::ofstream file;
    file.open(path, std::ios_base::binary);
    if(!file.is_open()){
        Logger::error(std::format("[VolumetricVideoResource] Cannot save compressed frames to {}.\n", path));
        return false;
    }

    m_p->write_file(file);

    file.close();

    return true;
}

bool VolumetricVideoResource::load_from_file(const std::string &path){


    if(path.length() == 0){
        Logger::error("[CompressedFramesManager] Invalid path.\n");
        return false;
    }

    std::ifstream file(path, std::ios_base::binary);
    if(!file.is_open()){
        Logger::error(std::format("[CompressedFramesManager] Cannot open compressed frames file: {}.\n", path));
        return false;
    }

    // clean data
    clean_frames();

    m_p->read_file(file);

    return true;
}



struct VolumetricVideoManager::Impl{

    VolumetricVideoResource *vv = nullptr;

    tjhandle jpegUncompressor;
    data::IntegersEncoder integerUncompressor;

    // # k4 images
    k4a::image depthImage;
    k4a::image pointCloudImage;
    std::tuple<Mode,std::optional<k4a_transformation_t>> tr;

    std::vector<size_t> indicesDepths1D;

    open3d::geometry::PointCloud open3dPointCloud;
    open3d::geometry::PointCloud totalCloud;

    Impl(){
        jpegUncompressor = tjInitDecompress();
    }
};

VolumetricVideoManager::VolumetricVideoManager(VolumetricVideoResource *volumetricVideo) : m_p(std::make_unique<VolumetricVideoManager::Impl>()){
    m_p->vv = volumetricVideo;
}

VolumetricVideoManager::~VolumetricVideoManager(){
    tjDestroy(m_p->jpegUncompressor);
}

bool VolumetricVideoManager::uncompress_color(CompressedDataFrame *cFrame, std::vector<uint8_t> &uncompressedColor){

    // resize uncompressed buffer
    const size_t uncomressedColorSize = cFrame->colorWidth * cFrame->colorHeight*4;
    if(uncompressedColor.size() != uncomressedColorSize){
        uncompressedColor.resize(uncomressedColorSize);
    }

    // uncompress
    const int decompressStatus = tjDecompress2(
        m_p->jpegUncompressor,
        cFrame->colorBuffer.data(),
        static_cast<unsigned long>(cFrame->colorBuffer.size()),
        uncompressedColor.data(),
        cFrame->colorWidth,
        0, // pitch
        cFrame->colorHeight,
        TJPF_RGBA,
        TJFLAG_FASTDCT// | TJFLAG_FASTUPSAMPLE
    );
    if(decompressStatus == -1){
        Logger::error("[CompressedFramesManager] Error uncompress color.\n");
        return false;
    }


    return true;
}

bool VolumetricVideoManager::uncompress_depth(CompressedDataFrame *cFrame, std::vector<uint16_t> &uncompressedDepth){

    // resize uncompressed buffer
    const size_t uncomressedDepthSize = cFrame->depthWidth*cFrame->depthHeight;
    if(uncompressedDepth.size() != uncomressedDepthSize){
        uncompressedDepth.resize(uncomressedDepthSize);
    }

    // uncompress
    try{
        m_p->integerUncompressor.decode(
            cFrame->depthBuffer.data(),
            cFrame->depthBuffer.size(),
            reinterpret_cast<std::uint32_t*>(uncompressedDepth.data()),
            (cFrame->depthWidth*cFrame->depthHeight)/2
        );
    }catch(std::exception e){
        Logger::error(std::format("[CompressedFramesManager] Error uncompress depth {}.\n", e.what()));
        return false;
    }

    return true;
}

bool VolumetricVideoManager::uncompress_infra(CompressedDataFrame *cFrame, std::vector<uint16_t> &uncompressedInfra){

    // resize uncompressed buffer
    const size_t uncomressedInfraSize = cFrame->infraWidth*cFrame->infraHeight;
    if(uncompressedInfra.size() != uncomressedInfraSize){
        uncompressedInfra.resize(uncomressedInfraSize);
    }

    // uncompress
    try{
        m_p->integerUncompressor.decode(
            cFrame->infraBuffer.data(),
            cFrame->infraBuffer.size(),
            reinterpret_cast<std::uint32_t*>(uncompressedInfra.data()),
            (cFrame->infraWidth*cFrame->infraHeight)/2
        );
    }catch(std::exception e){
        Logger::error(std::format("[CompressedFramesManager] Error uncompress infra {}.\n", e.what()));
        return false;
    }

    return true;
}

void VolumetricVideoManager::audio_samples_all_channels(size_t idCamera, std::vector<std::vector<float>> &audioBuffer){

    auto frames = m_p->vv->get_frames(idCamera);

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

void VolumetricVideoManager::audio_samples_all_channels(size_t idCamera, std::vector<float> &audioBuffer){

    auto frames = m_p->vv->get_frames(idCamera);
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


void VolumetricVideoManager::convert_to_depth_image(Mode mode, CompressedDataFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth, std::vector<uint8_t> &imageDepth){

    // resize image buffer
    size_t imageDepthSize = cFrame->depthWidth * cFrame->depthHeight*4;
    if(imageDepth.size() != imageDepthSize){
        imageDepth.resize(imageDepthSize);
    }

    const auto dRange = range(mode)*1000.f;
    const auto diff = dRange.y() - dRange.x();

    // convert data
    for(size_t ii = 0; ii < uncompressedDepth.size(); ++ii){

        if(uncompressedDepth[ii] == K4::invalid_depth_value){
            imageDepth[ii*4+0] = 0;
            imageDepth[ii*4+1] = 0;
            imageDepth[ii*4+2] = 0;
            imageDepth[ii*4+3] = 255;
            continue;;
        }

        float vF = (static_cast<float>(uncompressedDepth[ii]) - dRange.x())/diff;
        float intPart;
        float decPart = std::modf((vF*(depthGradient.size()-1)), &intPart);
        size_t idG = static_cast<size_t>(intPart);

        auto col = depthGradient[idG]*(1.f-decPart) + depthGradient[idG+1]*decPart;
        imageDepth[ii*4+0] = static_cast<std::uint8_t>(255*col.x());
        imageDepth[ii*4+1] = static_cast<std::uint8_t>(255*col.y());
        imageDepth[ii*4+2] = static_cast<std::uint8_t>(255*col.z());
        imageDepth[ii*4+3] = 255;
    }

}

void VolumetricVideoManager::convert_to_infra_image(CompressedDataFrame *cFrame, const std::vector<uint16_t> &uncompressedInfra, std::vector<uint8_t> &imageInfra){

    // resize image buffer
    size_t imageInfraSize = cFrame->infraWidth * cFrame->infraHeight*4;
    if(imageInfra.size() != imageInfraSize){
        imageInfra.resize(imageInfraSize);
    }

    // convert data
    const float max = 2000;
    for(size_t ii = 0; ii < uncompressedInfra.size(); ++ii){

        float vF = static_cast<float>(uncompressedInfra[ii]);
        if(vF > max){
            vF = max;
        }
        vF/=max;

        imageInfra[ii*4+0] = static_cast<std::uint8_t>(255*vF);
        imageInfra[ii*4+1] = static_cast<std::uint8_t>(255*vF);
        imageInfra[ii*4+2] = static_cast<std::uint8_t>(255*vF);
        imageInfra[ii*4+3] = 255;
    }
}

void VolumetricVideoManager::generate_cloud(CompressedDataFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth){

    // reset k4a transformation if necessary
    if(!std::get<1>(m_p->tr).has_value() || (cFrame->mode != std::get<0>(m_p->tr))){
        std::get<0>(m_p->tr) = cFrame->mode;
        std::get<1>(m_p->tr) = k4a_transformation_create(&cFrame->calibration);
    }

    // reset k4a images if necessary
    bool resetK4AImages = false;
    if(m_p->depthImage.is_valid()){
        if(m_p->depthImage.get_width_pixels() != static_cast<int>(cFrame->depthWidth) ||
            m_p->depthImage.get_height_pixels()  != static_cast<int>(cFrame->depthHeight) ){
            resetK4AImages = true;
        }
    }else{
        resetK4AImages = true;
    }
    if(resetK4AImages){

        // generate k4a image for storing depth values
        m_p->depthImage = k4a::image::create(
            k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16,
            cFrame->depthWidth, cFrame->depthHeight,
            static_cast<int32_t>(cFrame->depthWidth * 1 * sizeof(uint16_t))
        );

        // generate k4a image for storing cloud values
        m_p->pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
            cFrame->depthWidth,
            cFrame->depthHeight,
            static_cast<int32_t>(cFrame->depthWidth * 3 * sizeof(int16_t))
        );
    }

    // copy depth values
    std::copy(uncompressedDepth.begin(), uncompressedDepth.end(), reinterpret_cast<std::uint16_t*>(m_p->depthImage.get_buffer()));    

    // generate point cloud from depth image
    k4a_transformation_depth_image_to_point_cloud(
        std::get<1>(m_p->tr).value(),
        m_p->depthImage.handle(),
        K4A_CALIBRATION_TYPE_DEPTH,
        m_p->pointCloudImage.handle()
    );

}

void VolumetricVideoManager::process_open3d_cloud(const std::vector<uint8_t> &uncompressedColor){

    const size_t width  = m_p->pointCloudImage.get_width_pixels();
    const size_t height = m_p->pointCloudImage.get_height_pixels();
    const size_t size = width * height;

//    auto &pc = m_p->open3dPointCloud;
//    pc.points_.resize(cloudFrame.validVerticesCount);
//    pc.colors_.resize(cloudFrame.validVerticesCount);

//    for(size_t ii = 0; ii < cloudFrame.vertices.size(); ++ii){
//        const auto &pt = cloudFrame.vertices[ii];
//        const auto &c  = cloudFrame.colors[ii];
//        open3dPC.points_[ii] = Eigen::Vector3d{
//            static_cast<double>(pt.x()),
//            static_cast<double>(pt.y()),
//            static_cast<double>(pt.z())
//        };
//        open3dPC.colors_[ii] = Eigen::Vector3d{
//            static_cast<double>(c.x()),
//            static_cast<double>(c.y()),
//            static_cast<double>(c.z())
//        };
//    }
}

size_t VolumetricVideoManager::convert_to_cloud(size_t validVerticesCount,
        const std::vector<uint8_t> &uncompressedColor, const std::vector<std::uint16_t> &uncompressedDepth, ColoredCloudFrame &cloud){

    auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(m_p->pointCloudImage.get_buffer());

    // resize cloud if necessary
    if(cloud.validVerticesCount < validVerticesCount){
        cloud.vertices.resize(validVerticesCount);
        cloud.colors.resize(validVerticesCount);
    }
    cloud.validVerticesCount = validVerticesCount;

    // resize depth indices
    if(m_p->indicesDepths1D.size() != uncompressedDepth.size()){
        m_p->indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), 0);
    }


    // update cloud values
    size_t idV = 0;
    for_each(std::execution::unseq, std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), [&](size_t id){

        if(uncompressedDepth[id] == invalid_depth_value){
            return;
        }

        cloud.vertices[idV]= geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.01f;
        cloud.colors[idV] = geo::Pt3f{
            static_cast<float>(uncompressedColor[id*4+0]),
            static_cast<float>(uncompressedColor[id*4+1]),
            static_cast<float>(uncompressedColor[id*4+2])
        }/255.f;

        ++idV;
    });
    return idV;
}

size_t VolumetricVideoManager::convert_to_cloud(const std::vector<uint8_t> &uncompressedColor, const std::vector<uint16_t> &uncompressedDepth,
        geo::Pt3f *vertices, geo::Pt3f *colors){

    // resize depth indices
    if(m_p->indicesDepths1D.size() != uncompressedDepth.size()){
        m_p->indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), 0);
    }

    auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(m_p->pointCloudImage.get_buffer());
    size_t idV = 0;
    for_each(std::execution::unseq, std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), [&](size_t id){

        if(uncompressedDepth[id] == invalid_depth_value){
            return;
        }

        vertices[idV]= geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.01f;
        colors[idV] = geo::Pt3f{
            static_cast<float>(uncompressedColor[id*4+0]),
            static_cast<float>(uncompressedColor[id*4+1]),
            static_cast<float>(uncompressedColor[id*4+2])
        }/255.f;

        ++idV;
    });
    return idV;
}

size_t VolumetricVideoManager::convert_to_cloud(const std::vector<uint8_t> &uncompressedColor, const std::vector<uint16_t> &uncompressedDepth, geo::Pt3f *vertices, geo::Pt4f *colors){
    // resize depth indices
    if(m_p->indicesDepths1D.size() != uncompressedDepth.size()){
        m_p->indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), 0);
    }

    auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(m_p->pointCloudImage.get_buffer());
    size_t idV = 0;
    for_each(std::execution::unseq, std::begin(m_p->indicesDepths1D), std::end(m_p->indicesDepths1D), [&](size_t id){

        if(uncompressedDepth[id] == invalid_depth_value){
            return;
        }

        vertices[idV]= geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.01f;
        colors[idV] = geo::Pt4f{
            static_cast<float>(uncompressedColor[id*4+0]),
            static_cast<float>(uncompressedColor[id*4+1]),
            static_cast<float>(uncompressedColor[id*4+2]),
            255.f
        }/255.f;

        ++idV;
    });
    return idV;
}

void VolumetricVideoManager::register_frames(size_t idCamera, size_t startFrame, size_t endFrame, double voxelDownSampleSize){

    if(idCamera >= m_p->vv->nb_cameras() || startFrame >= endFrame){
        Logger::error("[CompressedFramesManager::voxelize_all_frames] Invalid input\n");
        return;
    }

    std::vector<uint8_t> uncompressedColor;
    std::vector<uint16_t> uncompressedDepth;
    open3d::geometry::PointCloud frameCloud;

    std::shared_ptr<open3d::geometry::PointCloud> previousDownSampledCloud = nullptr;
    geo::Mat4d totalTr = geo::Mat4d::identity();

    std::vector<FrameData> *frames = m_p->vv->get_frames(idCamera);

    // count total vertices
    size_t totalFramesVertices = 0;
    for(size_t ii = startFrame; ii < endFrame; ++ii){
        const auto &frame = (*frames)[ii];
        totalFramesVertices += frame.data->validVerticesCount;
//        Logger::message(std::format("frame {} {} \n", data->validVerticesCount, totalFramesVertices));
    }
    // init total cloud
    m_p->totalCloud.points_.resize(totalFramesVertices);
    m_p->totalCloud.colors_.resize(totalFramesVertices);

    // process frames
//    Logger::message("process frames\n");
    size_t currentVerticesId = 0;
    for(size_t ii = startFrame; ii < endFrame; ++ii){
        const auto &frame = (*frames)[ii];
        uncompress_color(frame.data.get(), uncompressedColor);
        uncompress_depth(frame.data.get(), uncompressedDepth);
        generate_cloud(frame.data.get(), uncompressedDepth);

        auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(m_p->pointCloudImage.get_buffer());

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

            if(uncompressedDepth[id] == invalid_depth_value){
                return;
            }

            frameCloud.points_[idV] = Eigen::Vector3d{
                static_cast<double>(-cloudBuffer[id].x()),
                static_cast<double>(-cloudBuffer[id].y()),
                static_cast<double>( cloudBuffer[id].z())
            }*0.01;
            frameCloud.colors_[idV] = Eigen::Vector3d{
                static_cast<double>(uncompressedColor[id*4+0]),
                static_cast<double>(uncompressedColor[id*4+1]),
                static_cast<double>(uncompressedColor[id*4+2])
            }/255.0;

            ++idV;
        });

//        Logger::message(std::format("cloud set {} {}\n", idV, data->validVerticesCount));
//        Logger::message("down sample cloud\n");
        auto opend3dDownCloud = frameCloud.VoxelDownSample(voxelDownSampleSize);
        double radius     = 0.1; // radius of the search
        int maxNeighbours = 30;  // max neighbours to be searched
        opend3dDownCloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius, maxNeighbours));

//        Logger::message(std::format("{} {}\n", frameCloud.points_.size(), opend3dDownCloud->points_.size()));

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

//        Logger::message("add transformed points to total cloud\n");
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
        files::CloudIO::save_cloud(downSampledFrameCloudStr, startDP, startDC, opend3dDownCloud->points_.size());

        Logger::message("end frame\n");
    }
}

void VolumetricVideoManager::voxelize(double voxelSize, ColoredCloudFrame &cloud){

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




