

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

// turbojpg
#include <turbojpeg.h>

// kinect4
#include <k4a/k4a.hpp>

// open3d
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/VoxelGrid.h"
#include "open3d/io/VoxelGridIO.h"

// local
#include "utility/files.hpp"
#include "utility/logger.hpp"
#include "data/integers_encoder.hpp"

using namespace tool::camera::K4;

struct CompressedFramesManager::Impl{

    tjhandle jpegUncompressor;
    IntegersEncoder integerUncompressor;

    std::vector<geo::Mat4d> transforms;
    std::vector<std::vector<std::tuple<size_t, std::int64_t, std::shared_ptr<CompressedDataFrame>>>> framesPerCamera;

    // # k4 images
    k4a::image depthImage;
    k4a::image pointCloudImage;
    std::tuple<Mode,std::optional<k4a_transformation_t>> tr;

    open3d::geometry::PointCloud open3dPointCloud;

    Impl(){
        jpegUncompressor = tjInitDecompress();
    }
};

CompressedFramesManager::CompressedFramesManager() : m_p(std::make_unique<CompressedFramesManager::Impl>()){

}

void CompressedFramesManager::add_frame(size_t idCamera, std::int64_t timestamp, std::shared_ptr<CompressedDataFrame> frame){
    if(idCamera >= m_p->framesPerCamera.size()){
        m_p->framesPerCamera.resize(idCamera+1);
        m_p->transforms.resize(idCamera+1);
    }
    auto idFrame = m_p->framesPerCamera[idCamera].size();
    m_p->framesPerCamera[idCamera].push_back(std::make_tuple(idFrame, timestamp, frame));
}


CompressedFramesManager::~CompressedFramesManager(){
    tjDestroy(m_p->jpegUncompressor);
}

size_t CompressedFramesManager::nb_cameras() const noexcept{return m_p->framesPerCamera.size();}

size_t CompressedFramesManager::nb_frames(size_t idCamera) const noexcept {
    if(idCamera < m_p->framesPerCamera.size()){
        return m_p->framesPerCamera[idCamera].size();
    }
//    Logger::error("[CompressedFramesManager] Cannot get frame count, invalid camera id .\n");
    return 0;
}

CompressedDataFrame *CompressedFramesManager::get_frame(size_t idFrame, size_t idCamera){
    if(idCamera < m_p->framesPerCamera.size()){
        if(idFrame < m_p->framesPerCamera[idCamera].size()){
            return std::get<2>(m_p->framesPerCamera[idCamera][idFrame]).get();
        }
        Logger::error("[CompressedFramesManager] Cannot get frame, invalid frame id.\n");
        return nullptr;
    }
    Logger::error("[CompressedFramesManager] Cannot get frame, invalid camera id.\n");
    return nullptr;
}

void CompressedFramesManager::set_transform(size_t idCamera, geo::Mat4d tr){
    if(idCamera < m_p->transforms.size()){
        m_p->transforms[idCamera] = tr;
        return;
    }
    Logger::error("[CompressedFramesManager] Cannot set transform, invalid camera id.\n");
}

bool CompressedFramesManager::save_to_file(const std::string &path){

    if(path.length() == 0){
        Logger::error("[CompressedFramesManager] Invalid path.\n");
        return false;
    }

    if(m_p->framesPerCamera.size() == 0){
        Logger::error("[CompressedFramesManager] No frames added.\n");
        return false;
    }

    std::ofstream file;
    file.open(path, std::ios_base::binary);
    if(!file.is_open()){
        Logger::error(std::format("[CompressedFramesManager] Cannot save compressed frames to {}.\n", path));
        return false;
    }

    // write nb of cameras
    files::write(file, static_cast<std::int8_t>(m_p->framesPerCamera.size()));       // std::int8_t

    // write infos per camera
    size_t idCamera = 0;
    for(const auto &frames : m_p->framesPerCamera){
        // nb frames
        files::write(file, static_cast<std::int32_t>(frames.size()));           // std::int32_t * cameras count
        // calibration matrix
        files::write_array(file, m_p->transforms[idCamera++].array.data(), 16);      // double * 16
    }

    // writes frames
    for(const auto &frames : m_p->framesPerCamera){
        for(const auto &frame : frames){
            // write frame
            size_t idFrame = std::get<0>(frame);
            std::int64_t timestamp = std::get<1>(frame);
            auto data = std::get<2>(frame);
            // # write frame info
            files::write(file, static_cast<std::int32_t>(idFrame));                         // std::int32_t
            files::write(file, timestamp);                                                  // std::int64_t
            files::write(file, static_cast<std::int32_t>(data->mode));                      // std::int32_t
            files::write_array(file, reinterpret_cast<char*>(&data->calibration), sizeof (k4a_calibration_t)); // sizeof(k4a_calibration_t)
            // # write color
            files::write(file, static_cast<std::int16_t>(data->colorWidth));                // std::int16_t
            files::write(file, static_cast<std::int16_t>(data->colorHeight));               // std::int16_t
            files::write(file, static_cast<std::int32_t>(data->colorBuffer.size()));        // std::int32_t
            files::write_array(file, data->colorBuffer.data(), data->colorBuffer.size());   // buffer size * std::int8_t
            // # write depth
            files::write(file, static_cast<std::int16_t>(data->depthWidth));                // std::int16_t
            files::write(file, static_cast<std::int16_t>(data->depthHeight));               // std::int16_t
            files::write(file, static_cast<std::int32_t>(data->depthBuffer.size()));        // std::int32_t
            files::write_array(file, data->depthBuffer.data(), data->depthBuffer.size());   // buffer size * std::int32_t
            // # write infra
            files::write(file, static_cast<std::int16_t>(data->infraWidth));                // std::int16_t
            files::write(file, static_cast<std::int16_t>(data->infraHeight));               // std::int16_t
            files::write(file, static_cast<std::int32_t>(data->infraBuffer.size()));        // std::int32_t
            files::write_array(file, data->infraBuffer.data(), data->infraBuffer.size());   // buffer size * std::int32_t
        }
    }

    file.close();

    return true;
}

bool CompressedFramesManager::load_from_file(const std::string &path){

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

    // read nb of cameras
    std::int8_t nbCameras;
    files::read(file, &nbCameras);
    m_p->framesPerCamera.resize(nbCameras);
    m_p->transforms.resize(nbCameras);

    // read infos per camera
    size_t idCamera = 0;
    std::int32_t nbFrames;
    for(auto &frames : m_p->framesPerCamera){

        // read nb frames
        files::read(file, &nbFrames);

        // create frames
        frames.reserve(nbFrames);
        for(size_t ii = 0; ii < static_cast<size_t>(nbFrames); ++ii){
            frames.emplace_back(std::make_tuple(0,0, std::make_shared<CompressedDataFrame>()));
        }

        // calibration matrix
        files::read_array(file, m_p->transforms[idCamera++].array.data(), 16);
    }

    // read frames
    size_t totalColorCompressed   = 0;
    size_t totalColorUncompressed = 0;
    size_t totalDepthCompressed   = 0;
    size_t totalDepthUncompressed = 0;
    size_t totalInfraCompressed   = 0;
    size_t totalInfraUncompressed = 0;

    for(auto &frames : m_p->framesPerCamera){
        for(auto &frame : frames){

            auto data = std::get<2>(frame);

            std::int32_t idFrame;
            std::int64_t timestamp;
            // # read frame info
            files::read(file, &idFrame);
            std::get<0>(frame) = idFrame;
            files::read(file, &timestamp);
            std::get<1>(frame) = timestamp;
            files::read(file, &data->mode);
            //Logger::message(std::format("id frame {} ts {}\n", idFrame, timestamp));
            files::read_array(file, reinterpret_cast<char*>(&data->calibration), sizeof (k4a_calibration_t));            
            // # read color
            std::int16_t colorWidth, colorHeight;
            std::int32_t colorBufferSize;
            files::read(file, &colorWidth);
            files::read(file, &colorHeight);
            files::read(file, &colorBufferSize);
//            Logger::message(std::format("colors sizes {} {} {} \n", (int)colorWidth, (int)colorHeight, colorBufferSize));
            data->colorWidth  = colorWidth;
            data->colorHeight = colorHeight;
            data->colorBuffer.resize(colorBufferSize);
            totalColorUncompressed += colorWidth*colorHeight;
            totalColorCompressed   += colorBufferSize;
            files::read_array(file, data->colorBuffer.data(), data->colorBuffer.size());
            // # read depth
            std::int16_t depthWidth, depthHeight;
            std::int32_t depthBufferSize;
            files::read(file, &depthWidth);
            files::read(file, &depthHeight);
            files::read(file, &depthBufferSize);
//            Logger::message(std::format("depth sizes {} {} {} \n", (int)depthWidth, (int)depthHeight, depthBufferSize));
            data->depthWidth  = depthWidth;
            data->depthHeight = depthHeight;
            data->depthBuffer.resize(depthBufferSize);
            totalDepthUncompressed += depthWidth*depthHeight;
            totalDepthCompressed   += depthBufferSize;
            files::read_array(file, data->depthBuffer.data(), data->depthBuffer.size());
            // # read infra
            std::int16_t infraWidth, infraHeight;
            std::int32_t infraBufferSize;
            files::read(file, &infraWidth);
            files::read(file, &infraHeight);
            files::read(file, &infraBufferSize);
//            Logger::message(std::format("infra sizes {} {} {} \n", (int)infraWidth, (int)infraHeight, infraBufferSize));
            data->infraWidth  = infraWidth;
            data->infraHeight = infraHeight;
            data->infraBuffer.resize(infraBufferSize);
            totalInfraUncompressed += infraWidth*infraHeight;
            totalInfraCompressed   += infraBufferSize;
            files::read_array(file, data->infraBuffer.data(), data->infraBuffer.size());   // buffer size * std::int32_t
        }
    }

    if(totalColorUncompressed != 0 && totalDepthUncompressed != 0 && totalInfraUncompressed != 0){
        auto colCompressionFactor   = (1.0*totalColorCompressed)/totalColorUncompressed;
        auto depthCompressionFactor = (1.0*totalDepthCompressed)/totalDepthUncompressed;
        auto infraCompressionFactor = (1.0*totalInfraCompressed)/totalInfraUncompressed;
        Logger::message(std::format("Compression rates {} {} {}\n", colCompressionFactor, depthCompressionFactor, infraCompressionFactor));
    }

    return true;
}

bool CompressedFramesManager::uncompress_color(CompressedDataFrame *cFrame, std::vector<uint8_t> &uncompressedColor){

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

bool CompressedFramesManager::uncompress_depth(CompressedDataFrame *cFrame, std::vector<uint16_t> &uncompressedDepth){

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

bool CompressedFramesManager::uncompress_infra(CompressedDataFrame *cFrame, std::vector<uint16_t> &uncompressedInfra){

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

void CompressedFramesManager::convert_to_depth_image(Mode mode, CompressedDataFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth, std::vector<uint8_t> &imageDepth){

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

void CompressedFramesManager::convert_to_infra_image(CompressedDataFrame *cFrame, const std::vector<uint16_t> &uncompressedInfra, std::vector<uint8_t> &imageInfra){

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

void CompressedFramesManager::generate_cloud(CompressedDataFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth){

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

void CompressedFramesManager::process_open3d_cloud(const std::vector<uint8_t> &uncompressedColor){

//    const size_t width  = m_p->pointCloudImage.get_width_pixels();
//    const size_t height = m_p->pointCloudImage.get_height_pixels();
//    const size_t size = width * height;

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

void CompressedFramesManager::convert_to_cloud(const std::vector<uint8_t> &uncompressedColor, ColoredCloudFrame &cloud){


    auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(m_p->pointCloudImage.get_buffer());
    const size_t width  = m_p->pointCloudImage.get_width_pixels();
    const size_t height = m_p->pointCloudImage.get_height_pixels();
    const size_t size = width * height;

    // resize cloud if necessary
    if(cloud.vertices.size() != size){
        cloud.vertices.resize(size);
        cloud.colors.resize(size);
        cloud.validVerticesCount = size;
    }

    // update cloud values
    for(size_t ii = 0; ii < size; ++ii){
        cloud.vertices[ii] = geo::Pt3f{
            static_cast<float>(-cloudBuffer[ii].x()),
            static_cast<float>(-cloudBuffer[ii].y()),
            static_cast<float>(cloudBuffer[ii].z())
        }*0.01f;

        cloud.colors[ii] = geo::Pt3f{
           static_cast<float>(uncompressedColor[ii*4+0]),
           static_cast<float>(uncompressedColor[ii*4+1]),
           static_cast<float>(uncompressedColor[ii*4+2])
       }/255.f;
    }
}

void CompressedFramesManager::clean_frames(){
    m_p->framesPerCamera.clear();
}


