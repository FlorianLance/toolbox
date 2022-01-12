

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

#include "frame_uncompressor.hpp"

// std
#include <execution>

// kinect
#include "k4a/k4a.hpp"

// turbojpg
#include <turbojpeg.h>

// turbopfor
#include "TurboPFor/vp4.h"

// local
// # data
#include "data/integers_encoder.hpp"
// # utility
#include "utility/logger.hpp"

using namespace tool;
using namespace tool::geo;
using namespace tool::camera;
using namespace tool::camera::K4;


struct FullFrameUncompressor::Impl{
    // compression
    // # integer compressor
    data::IntegersEncoder integerUncompressor;
    // # jpeg compressor
    tjhandle jpegUncompressor = nullptr;

    // # k4 images
    k4a::image depthImage;
    k4a::image pointCloudImage;
    std::tuple<Mode,std::optional<k4a_transformation_t>> tr;
};

FullFrameUncompressor::FullFrameUncompressor() : i(std::make_unique<Impl>()){
    i->jpegUncompressor = tjInitDecompress();
}
FullFrameUncompressor::~FullFrameUncompressor(){
    tjDestroy(i->jpegUncompressor);
}

bool FullFrameUncompressor::uncompress_color(CompressedFullFrame *cFrame, std::vector<uint8_t> &uncompressedColor){

    // resize uncompressed buffer
    const size_t uncomressedColorSize = cFrame->colorWidth * cFrame->colorHeight*4;
    if(uncompressedColor.size() != uncomressedColorSize){
        uncompressedColor.resize(uncomressedColorSize);
    }

    // uncompress
    const int decompressStatus = tjDecompress2(
        i->jpegUncompressor,
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
        Logger::error("[FullFrameUncompressor] Error uncompress color.\n");
        return false;
    }

    return true;
}

bool FullFrameUncompressor::uncompress_depth(CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedDepth){

    // resize uncompressed buffer
    const size_t uncomressedDepthSize = cFrame->depthWidth*cFrame->depthHeight;
    if(uncompressedDepth.size() != uncomressedDepthSize){
        uncompressedDepth.resize(uncomressedDepthSize);
    }

    // uncompress
    try{
        i->integerUncompressor.decode(
            cFrame->depthBuffer.data(),
            cFrame->depthBuffer.size(),
            reinterpret_cast<std::uint32_t*>(uncompressedDepth.data()),
            (cFrame->depthWidth*cFrame->depthHeight)/2
        );
    }catch(std::exception e){
        Logger::error(std::format("[FullFrameUncompressor] Error uncompress depth {}.\n", e.what()));
        return false;
    }

    return true;
}

bool FullFrameUncompressor::uncompress_infra(CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedInfra){

    // resize uncompressed buffer
    const size_t uncomressedInfraSize = cFrame->infraWidth*cFrame->infraHeight;
    if(uncompressedInfra.size() != uncomressedInfraSize){
        uncompressedInfra.resize(uncomressedInfraSize);
    }

    // uncompress
    try{
        i->integerUncompressor.decode(
            cFrame->infraBuffer.data(),
            cFrame->infraBuffer.size(),
            reinterpret_cast<std::uint32_t*>(uncompressedInfra.data()),
            (cFrame->infraWidth*cFrame->infraHeight)/2
        );
    }catch(std::exception e){
        Logger::error(std::format("[FullFrameUncompressor] Error uncompress infra {}.\n", e.what()));
        return false;
    }

    return true;
}

void FullFrameUncompressor::generate_cloud(CompressedFullFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth){

    // reset k4a transformation if necessary
    if(!std::get<1>(i->tr).has_value() || (cFrame->mode != std::get<0>(i->tr))){
        std::get<0>(i->tr) = cFrame->mode;
        std::get<1>(i->tr) = k4a_transformation_create(&cFrame->calibration);
    }

    // reset k4a images if necessary
    bool resetK4AImages = false;
    if(i->depthImage.is_valid()){
        if(i->depthImage.get_width_pixels() != static_cast<int>(cFrame->depthWidth) ||
            i->depthImage.get_height_pixels()  != static_cast<int>(cFrame->depthHeight) ){
            resetK4AImages = true;
        }
    }else{
        resetK4AImages = true;
    }
    if(resetK4AImages){

        // generate k4a image for storing depth values
        i->depthImage = k4a::image::create(
            k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16,
            cFrame->depthWidth, cFrame->depthHeight,
            static_cast<int32_t>(cFrame->depthWidth * 1 * sizeof(uint16_t))
        );

        // generate k4a image for storing cloud values
        i->pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
            cFrame->depthWidth,
            cFrame->depthHeight,
            static_cast<int32_t>(cFrame->depthWidth * 3 * sizeof(int16_t))
        );
    }

    // copy depth values
    std::copy(uncompressedDepth.begin(), uncompressedDepth.end(), reinterpret_cast<std::uint16_t*>(i->depthImage.get_buffer()));

    // generate point cloud from depth image
    k4a_transformation_depth_image_to_point_cloud(
        std::get<1>(i->tr).value(),
        i->depthImage.handle(),
        K4A_CALIBRATION_TYPE_DEPTH,
        i->pointCloudImage.handle()
    );
}

geo::Pt3<int16_t> *FullFrameUncompressor::cloud_data(){
    if(i->pointCloudImage.is_valid()){
        return reinterpret_cast<Pt3<int16_t>*>(i->pointCloudImage.get_buffer());
    }
    return nullptr;
}


struct CloudFrameUncompressor::Impl{
    tjhandle jpegUncompressor = nullptr;
    std::vector<size_t> indicesValid1D;
    std::vector<std::uint32_t> cloudBuffer;
};

CloudFrameUncompressor::CloudFrameUncompressor() : i(std::make_unique<Impl>()){
    i->jpegUncompressor = tjInitDecompress();
}
CloudFrameUncompressor::~CloudFrameUncompressor(){
    tjDestroy(i->jpegUncompressor);
}

bool CloudFrameUncompressor::uncompress_color(CompressedCloudFrame *cFrame, std::vector<uint8_t> &uncompressedColor){

    // resize uncompressed buffer
    const size_t uncomressedColorSize = cFrame->colorWidth * cFrame->colorHeight*3;
    if(uncompressedColor.size() != uncomressedColorSize){
        uncompressedColor.resize(uncomressedColorSize);
    }

    // uncompress
    const int decompressStatus = tjDecompress2(
        i->jpegUncompressor,
        cFrame->colorBuffer.data(),
        static_cast<unsigned long>(cFrame->colorBuffer.size()),
        uncompressedColor.data(),
        cFrame->colorWidth,
        0, // pitch
        cFrame->colorHeight,
        TJPF_RGB,
        TJFLAG_FASTDCT// | TJFLAG_FASTUPSAMPLE
    );
    if(decompressStatus == -1){
        Logger::error("[CloudFrameUncompressor] Error uncompress color.\n");
        return false;
    }

    return true;
}

bool CloudFrameUncompressor::uncompress(CompressedCloudFrame *cFrame, CloudFrame &frame){

    uncompress_color(cFrame, frame.colorData);

    // resize uncompressed cloud buffer
    const auto idV = cFrame->validVerticesCount;
    const size_t paddedDiff = 128-((idV*3)%128);
    const size_t cloudBufferSize = idV*3 + paddedDiff;
    if(i->cloudBuffer.size() < cloudBufferSize){
        i->cloudBuffer.resize(cloudBufferSize);
    }

    size_t decodedBytesNb = p4nzdec128v32(
        cFrame->cloudBuffer.data(),
        cloudBufferSize,
        i->cloudBuffer.data());
    if(decodedBytesNb == 0){
        Logger::error("[CloudFrameUncompressor] Error uncompress cloud.\n");
        return false;
    }
//    Logger::message(std::format("sizeb {} decodedb {} cbs {} |", cFrame->cloudBuffer.size(), decodedBytesNb, cloudBufferSize));

    // update valid id array
    if(i->indicesValid1D.size() < idV){
        i->indicesValid1D.resize(idV);
        std::iota(std::begin(i->indicesValid1D), std::end(i->indicesValid1D), 0);
    }

    frame.cloud.validVerticesCount = idV;
    frame.cloud.vertices.resize(idV);
    frame.cloud.colors.resize(idV);

    for_each(std::execution::par_unseq, std::begin(i->indicesValid1D), std::begin(i->indicesValid1D) + idV, [&](size_t id){

        auto x = static_cast<float>(i->cloudBuffer[        id])-4096;
        auto y = static_cast<float>(i->cloudBuffer[idV   + id])-4096;
        auto z = static_cast<float>(i->cloudBuffer[2*idV + id]);

        frame.cloud.vertices[id]= geo::Pt3f{
           static_cast<float>(-x),
           static_cast<float>(-y),
           static_cast<float>( z)
        }*0.001f;

        frame.cloud.colors[id] = geo::Pt3f{
            static_cast<float>(frame.colorData[id*3+0]),
            static_cast<float>(frame.colorData[id*3+1]),
            static_cast<float>(frame.colorData[id*3+2])
        }/255.f;
    });

    return true;
}


