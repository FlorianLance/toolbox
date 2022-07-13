

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

#include "k4_frame_compressor.hpp"

// std
#include <execution>

// turbojpg
#include <turbojpeg.h>

// turbopfor
#include "TurboPFor/vp4.h"

// local
// # utility
#include "utility/logger.hpp"
#include "utility/format.hpp"

using namespace tool;
using namespace tool::geo;
using namespace tool::camera;


struct K4CloudFrameCompressor::Impl{
    tjhandle jpegCompressor = nullptr;
    unsigned char *tjCompressedImage = nullptr;
    std::vector<size_t> indicesValid1D;
    std::vector<std::uint16_t> processedCloudData;
    std::vector<std::uint8_t> processedColorData;

    std::vector<std::int16_t> processedAudioData;
};

K4CloudFrameCompressor::K4CloudFrameCompressor() : i(std::make_unique<Impl>()){
    i->jpegCompressor = tjInitCompress();
}
K4CloudFrameCompressor::~K4CloudFrameCompressor(){
    if(i->tjCompressedImage != nullptr){
        tjFree(i->tjCompressedImage);
    }
    tjDestroy(i->jpegCompressor);
}

std::unique_ptr<K4CompressedCloudFrame> K4CloudFrameCompressor::compress(
    size_t validDepthValues,
    int jpegQuality,
    k4a::image colorImage,
    k4a::image depthImage,
    k4a::image cloud, float *audioData, size_t audioSize){

    // create compressed frame
    auto cFrame                = std::make_unique<K4CompressedCloudFrame>();
    cFrame->validVerticesCount = validDepthValues;
    cFrame->colorWidth         = colorImage.get_width_pixels();
    cFrame->colorHeight        = colorImage.get_height_pixels();


    // get buffers
    auto cloudData = reinterpret_cast<geo::Pt3<int16_t>*>(cloud.get_buffer());
    auto colorData = reinterpret_cast<const geo::Pt4<uint8_t>*>(colorImage.get_buffer());
    auto depthData = reinterpret_cast<const uint16_t*>(depthImage.get_buffer());

    // fill indices array
    const size_t depthSize = depthImage.get_width_pixels()*depthImage.get_height_pixels();
    if(i->indicesValid1D.size() < depthSize){
        i->indicesValid1D.resize(depthSize);
        std::iota(std::begin(i->indicesValid1D), std::end(i->indicesValid1D), 0);
    }

    const auto idV = cFrame->validVerticesCount;

    // fill valid id
    std::vector<size_t> validId;
    validId.reserve(idV);
    for_each(std::execution::unseq, std::begin(i->indicesValid1D), std::begin(i->indicesValid1D) + depthSize, [&](size_t id){
        if(depthData[id] != k4_invalid_depth_value){
            validId.push_back(id);
        }
    });

    // resize processed data
    size_t paddedCloudSizeDiff = 128-((idV*3)%128);
    size_t processsedCloudDataSize = idV*3 + paddedCloudSizeDiff;
    if(i->processedCloudData.size() < processsedCloudDataSize){
        i->processedCloudData.resize(processsedCloudDataSize);
    }
    std::fill(i->processedCloudData.begin(), i->processedCloudData.end(), 0);

    const size_t colorBufferSize = colorImage.get_width_pixels()*colorImage.get_height_pixels()*3;
    if(i->processedColorData.size() < colorBufferSize){
        i->processedColorData.resize(colorBufferSize);
    }
    std::fill(i->processedColorData.begin(), i->processedColorData.end(), 0);

    // process cloud buffer
    for_each(std::execution::par_unseq, std::begin(i->indicesValid1D), std::begin(i->indicesValid1D) + idV, [&](size_t id){
        i->processedCloudData[id]         = static_cast<std::uint16_t>(static_cast<std::int32_t>(cloudData[validId[id]].x())+4096);
        i->processedCloudData[idV   + id] = static_cast<std::uint16_t>(static_cast<std::int32_t>(cloudData[validId[id]].y())+4096);
        i->processedCloudData[2*idV + id] = static_cast<std::uint16_t>(cloudData[validId[id]].z());

        i->processedColorData[id*3+0]     = colorData[validId[id]].x();
        i->processedColorData[id*3+1]     = colorData[validId[id]].y();
        i->processedColorData[id*3+2]     = colorData[validId[id]].z();
    });

    // compress cloud buffer
    cFrame->encodedCloudData.resize(processsedCloudDataSize*2);
    size_t encodedBytesNb = p4nzenc128v16(
        i->processedCloudData.data(),
        processsedCloudDataSize,
        cFrame->encodedCloudData.data()
    );
    cFrame->encodedCloudData.resize(encodedBytesNb);

    // compress color buffer
    if(i->tjCompressedImage == nullptr){
        i->tjCompressedImage = tjAlloc(static_cast<int>(cFrame->colorHeight*cFrame->colorWidth*3));
    }

    long unsigned int jpegSize = 0;
    int ret = tjCompress2(
        i->jpegCompressor,
        i->processedColorData.data(),
        static_cast<int>(cFrame->colorWidth), 0, static_cast<int>(cFrame->colorHeight),
        TJPF_BGR,
        &i->tjCompressedImage, &jpegSize, TJSAMP_444, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT
    );

    if(ret == -1){
        Logger::error(std::format("[Kinect4] CloudFrameCompressor error with code: {}\n", tjGetErrorStr2(i->jpegCompressor)));
        return nullptr;
    }
    cFrame->encodedColorData.resize(jpegSize);
    std::copy(i->tjCompressedImage, i->tjCompressedImage + jpegSize, cFrame->encodedColorData.begin());

//    if(audioSize > 0){

        // resize processed data
//        const size_t paddedAudioSizeDiff = 128-((audioSize)%128);
//        const size_t processsedAudioDataSize = idV*3 + paddedDiff;
//        if(i->processedCloudData.size() < processsedCloudDataSize){
//            i->processedCloudData.resize(processsedCloudDataSize);
//        }
//        std::fill(i->processedCloudData.begin(), i->processedCloudData.end(), 0);

//        cFrame->encodedAudioData;

        // process audio buffer
        // ...

        // compress audio buffer
        // ...


        // -32768
        // 32767

        // -1.000 +1.0000
        // * 32767
        // -32767 +32767

        //    encodedBytesNb = p
//    }

    return cFrame;
}

struct K4FullFrameCompressor::Impl{
    tjhandle jpegCompressor = nullptr;
    unsigned char *tjCompressedImage = nullptr;
};

K4FullFrameCompressor::K4FullFrameCompressor() : i(std::make_unique<Impl>()){
    i->jpegCompressor = tjInitCompress();
}
K4FullFrameCompressor::~K4FullFrameCompressor(){
    if(i->tjCompressedImage != nullptr){
        tjFree(i->tjCompressedImage);
    }
    tjDestroy(i->jpegCompressor);
}

std::unique_ptr<K4CompressedFullFrame> K4FullFrameCompressor::compress(
    size_t validDepthValues,
    int jpegQuality,
    std::optional<k4a::image> colorImage,
    std::optional<k4a::image> depthImage,
    std::optional<k4a::image> infraredImage){

    auto cFrame = std::make_unique<K4CompressedFullFrame>();

    if(colorImage.has_value()){

        // init buffer
        cFrame->colorWidth  = colorImage->get_width_pixels();
        cFrame->colorHeight = colorImage->get_height_pixels();

        long unsigned int jpegColorSize = 0;
        if(i->tjCompressedImage == nullptr){
            i->tjCompressedImage = tjAlloc(static_cast<int>(cFrame->colorHeight*cFrame->colorWidth*4));
        }

        int ret = tjCompress2(i->jpegCompressor,
            reinterpret_cast<const unsigned char*>(colorImage->get_buffer()),
            static_cast<int>(cFrame->colorWidth), 0, static_cast<int>(cFrame->colorHeight),
            TJPF_BGRA,
            &i->tjCompressedImage, &jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);

        if(ret == -1){
            Logger::error(std::format("[Kinect4] tjCompress2 error with code: {}\n", tjGetErrorStr2(i->jpegCompressor)));
            return nullptr;
        }

        cFrame->encodedColorData.resize(jpegColorSize);
        std::copy(i->tjCompressedImage, i->tjCompressedImage + jpegColorSize, std::begin(cFrame->encodedColorData));

    }else{
        cFrame->colorWidth  = 0;
        cFrame->colorHeight = 0;
        cFrame->encodedColorData = {};
    }

    if(depthImage.has_value()){

        cFrame->validVerticesCount = validDepthValues;

        // init buffer
        cFrame->depthWidth  = depthImage->get_width_pixels();
        cFrame->depthHeight = depthImage->get_height_pixels();        
        auto depthSize   = cFrame->depthWidth*cFrame->depthHeight;
        auto depthBuffer = reinterpret_cast<uint16_t*>(depthImage->get_buffer());

        // compress depth buffer
        cFrame->encodedDepthData.resize(depthSize*2);

        // depth sizes for every mode have already a 128 padded size
        size_t encodedBytesNb = p4nzenc128v16(
            depthBuffer,
            depthSize,
            cFrame->encodedDepthData.data()
        );
        cFrame->encodedDepthData.resize(encodedBytesNb);
    }

    if(infraredImage.has_value()){

        // init buffer
        cFrame->infraWidth  = infraredImage->get_width_pixels();
        cFrame->infraHeight = infraredImage->get_height_pixels();
        auto infraSize = cFrame->infraWidth*cFrame->infraHeight;
        auto infraBuffer = reinterpret_cast<uint16_t*>(infraredImage->get_buffer());

        // compress depth buffer
        cFrame->encodedInfraData.resize(infraSize*2);

        // infra sizes for every mode have already a 128 padded size
        size_t encodedBytesNb = p4nzenc128v16(
            infraBuffer,
            infraSize,
            cFrame->encodedInfraData.data()
        );
        cFrame->encodedInfraData.resize(encodedBytesNb);
    }

    return cFrame;
}

