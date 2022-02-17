

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

#include "k4_frame_uncompressor.hpp"

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
//#include "data/integers_encoder.hpp"
// # utility
#include "utility/logger.hpp"

using namespace tool;
using namespace tool::geo;
using namespace tool::camera;



struct K4FrameUncompressor::Impl{
    tjhandle jpegUncompressor = nullptr;
};

K4FrameUncompressor::K4FrameUncompressor() : i(std::make_unique<Impl>()){
    i->jpegUncompressor = tjInitDecompress();
}
K4FrameUncompressor::~K4FrameUncompressor(){
    tjDestroy(i->jpegUncompressor);
}

bool K4FrameUncompressor::uncompress_color(size_t colorWidth, size_t colorHeight, const std::vector<uint8_t> &encoded, std::vector<geo::Pt3<uint8_t> > &uncompressedColor){

    // resize uncompressed buffer
    const size_t colorSize = colorWidth * colorHeight;
    if(uncompressedColor.size() != colorSize){
        uncompressedColor.resize(colorSize);
    }

    // uncompress
    const int decompressStatus = tjDecompress2(
        i->jpegUncompressor,
        encoded.data(),
        static_cast<unsigned long>(encoded.size()),
        reinterpret_cast<std::uint8_t*>(uncompressedColor.data()),
        colorWidth,
        0, // pitch
        colorHeight,
        TJPF_RGB,
        TJFLAG_FASTDCT// | TJFLAG_FASTUPSAMPLE
    );
    if(decompressStatus == -1){
        Logger::error("[FrameUncompressor] Error uncompress color.\n");
        return false;
    }

    return true;
}



struct K4FullFrameUncompressor::Impl{ 
    k4a::image depthImage;
    k4a::image pointCloudImage;
    std::tuple<K4Mode,std::optional<k4a_transformation_t>> tr;
};

K4FullFrameUncompressor::K4FullFrameUncompressor() : i(std::make_unique<Impl>()){
}
K4FullFrameUncompressor::~K4FullFrameUncompressor(){
}



bool K4FullFrameUncompressor::uncompress_color(K4CompressedFullFrame *cFrame, std::vector<geo::Pt3<uint8_t>> &uncompressedColor) {
    return K4FrameUncompressor::uncompress_color(
        cFrame->colorWidth,
        cFrame->colorHeight,
        cFrame->encodedColorData,
        uncompressedColor
    );
}

bool K4FullFrameUncompressor::uncompress_depth(K4CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedDepth){

    // resize decoded data
    const size_t decodedDepthSize = cFrame->depthWidth*cFrame->depthHeight;
    if(uncompressedDepth.size() != decodedDepthSize){
        uncompressedDepth.resize(decodedDepthSize);
    }

    size_t decodedBytesNb = p4nzdec128v16(
        cFrame->encodedDepthData.data(),
        decodedDepthSize,
        uncompressedDepth.data());
    if(decodedBytesNb == 0){
        Logger::error("[FullFrameUncompressor] Error decoding depth.\n");
        return false;
    }
    return true;
}

bool K4FullFrameUncompressor::uncompress_infra(K4CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedInfra){

    // resize decoded data
    const size_t decodedInfraSize = cFrame->infraWidth*cFrame->infraHeight;
    if(uncompressedInfra.size() != decodedInfraSize){
        uncompressedInfra.resize(decodedInfraSize);
    }

    size_t decodedBytesNb = p4nzdec128v16(
        cFrame->encodedInfraData.data(),
        decodedInfraSize,
        uncompressedInfra.data());
    if(decodedBytesNb == 0){
        Logger::error("[FullFrameUncompressor] Error decoding infra.\n");
        return false;
    }

    return true;
}

void K4FullFrameUncompressor::generate_cloud(K4CompressedFullFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth){

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

bool K4FullFrameUncompressor::uncompress(K4CompressedFullFrame *cFrame, K4FullFrame &fframe){

    fframe.idCapture      = cFrame->idCapture;
    fframe.afterCaptureTS = cFrame->afterCaptureTS;

    fframe.colorWidth  = 0;
    fframe.colorHeight = 0;
    fframe.depthWidth  = 0;
    fframe.depthHeight = 0;
    fframe.infraWidth  = 0;
    fframe.infraHeight = 0;

    if(cFrame->encodedColorData.size() > 0){
        if(!uncompress_color(cFrame, fframe.imageColorData)){
            return false;
        }
        fframe.colorWidth  = cFrame->colorWidth;
        fframe.colorHeight = cFrame->colorHeight;
    }
    // depth
    if(cFrame->encodedDepthData.size() > 0){
        if(!uncompress_depth(cFrame, fframe.rawDepthData)){
            return false;
        }
        convert_to_depth_image(cFrame->mode, cFrame->depthWidth, cFrame->depthHeight, fframe.rawDepthData, fframe.imageDepthData);
        fframe.depthWidth  = cFrame->depthWidth;
        fframe.depthHeight = cFrame->depthHeight;
    }
    // infra
    if(cFrame->encodedInfraData.size() > 0){
        uncompress_infra(cFrame, fframe.rawInfraData);
        convert_to_infra_image(cFrame->infraWidth, cFrame->infraHeight, fframe.rawInfraData, fframe.imageInfraData);
        fframe.infraWidth  = cFrame->infraWidth;
        fframe.infraHeight = cFrame->infraHeight;
    }
    // cloud
    if(cFrame->encodedDepthData.size() > 0){
        generate_cloud(cFrame, fframe.rawDepthData);
        convert_to_cloud(cFrame->validVerticesCount, fframe.imageColorData, fframe.rawDepthData, fframe.cloud);
    }

    return true;
}


void K4FrameUncompressor::convert_to_depth_image(K4Mode mode, size_t depthWidth, size_t depthHeight, const std::vector<uint16_t> &uncompressedDepth, std::vector<geo::Pt3<uint8_t> > &imageDepth){

    // resize image buffer
    size_t imageDepthSize = depthWidth * depthHeight*3;
    if(imageDepth.size() != imageDepthSize){
        imageDepth.resize(imageDepthSize);
    }

    const auto dRange = range(mode)*1000.f;
    const auto diff = dRange.y() - dRange.x();

    // convert data
    for(size_t ii = 0; ii < uncompressedDepth.size(); ++ii){

        if(uncompressedDepth[ii] == k4_invalid_depth_value){
            imageDepth[ii] = {0,0,0};
            continue;;
        }

        float vF = (static_cast<float>(uncompressedDepth[ii]) - dRange.x())/diff;
        float intPart;
        float decPart = std::modf((vF*(depthGradient.size()-1)), &intPart);
        size_t idG = static_cast<size_t>(intPart);

        auto col = depthGradient[idG]*(1.f-decPart) + depthGradient[idG+1]*decPart;
        imageDepth[ii] = {
            static_cast<std::uint8_t>(255*col.x()),
            static_cast<std::uint8_t>(255*col.y()),
            static_cast<std::uint8_t>(255*col.z())
        };
    }
}

void K4FrameUncompressor::convert_to_infra_image(size_t infraWidth, size_t infraHeight, const std::vector<uint16_t> &uncompressedInfra, std::vector<geo::Pt3<uint8_t> > &imageInfra){

    // resize image buffer
    size_t imageInfraSize = infraWidth * infraHeight;
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

        imageInfra[ii] = {
            static_cast<std::uint8_t>(255*vF),
            static_cast<std::uint8_t>(255*vF),
            static_cast<std::uint8_t>(255*vF)
        };
    }
}

size_t K4FullFrameUncompressor::convert_to_cloud(size_t validVerticesCount,
    const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor,
    const std::vector<std::uint16_t> &uncompressedDepth,
    ColoredCloudFrame &cloud){

    auto cloudBuffer = cloud_data();

    // resize cloud if necessary
    if(cloud.validVerticesCount < validVerticesCount){
        cloud.vertices.resize(validVerticesCount);
        cloud.colors.resize(validVerticesCount);
    }
    cloud.validVerticesCount = validVerticesCount;

    // resize depth indices
    if(indicesDepths1D.size() != uncompressedDepth.size()){
        indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);
    }

    // update cloud values
    size_t idV = 0;
    for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

        if(uncompressedDepth[id] == k4_invalid_depth_value){
            return;
        }

        cloud.vertices[idV]= geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.001f;
        cloud.colors[idV] = geo::Pt3f{
            static_cast<float>(uncompressedColor[id].x()),
            static_cast<float>(uncompressedColor[id].y()),
            static_cast<float>(uncompressedColor[id].z())
        }/255.f;

        ++idV;
    });
    return idV;
}

size_t K4FullFrameUncompressor::convert_to_cloud(const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor, const std::vector<uint16_t> &uncompressedDepth,
    geo::Pt3f *vertices, geo::Pt3f *colors){

    // resize depth indices
    if(indicesDepths1D.size() != uncompressedDepth.size()){
        indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);
    }

    auto cloudBuffer = cloud_data();
    size_t idV = 0;
    for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

        if(uncompressedDepth[id] == k4_invalid_depth_value){
            return;
        }

        vertices[idV]= geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.001f;
        colors[idV] = geo::Pt3f{
            static_cast<float>(uncompressedColor[id].x()),
            static_cast<float>(uncompressedColor[id].y()),
            static_cast<float>(uncompressedColor[id].z())
        }/255.f;

        ++idV;
    });
    return idV;
}

size_t K4FullFrameUncompressor::convert_to_cloud(const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor, const std::vector<uint16_t> &uncompressedDepth, geo::Pt3f *vertices, geo::Pt4f *colors){

    // resize depth indices
    if(indicesDepths1D.size() != uncompressedDepth.size()){
        indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);
    }

    auto cloudBuffer = cloud_data();
    size_t idV = 0;
    for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

        if(uncompressedDepth[id] == k4_invalid_depth_value){
            return;
        }

        vertices[idV]= geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.001f;
        colors[idV] = geo::Pt4f{
            static_cast<float>(uncompressedColor[id].x()),
            static_cast<float>(uncompressedColor[id].y()),
            static_cast<float>(uncompressedColor[id].z()),
            255.f
        }/255.f;

        ++idV;
    });
    return idV;
}

size_t K4FullFrameUncompressor::convert_to_cloud(const std::vector<geo::Pt3<std::uint8_t>> &uncompressedColor, const std::vector<uint16_t> &uncompressedDepth, geo::Pt3f *vertices, geo::Pt4<uint8_t> *colors){

    // resize depth indices
    if(indicesDepths1D.size() != uncompressedDepth.size()){
        indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);
    }

    auto cloudBuffer = cloud_data();
    size_t idV = 0;

    for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

        if(uncompressedDepth[id] == k4_invalid_depth_value){
            return;
        }

        vertices[idV] = geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.001f;

        colors[idV] = {
            uncompressedColor[id].x(),
            uncompressedColor[id].y(),
            uncompressedColor[id].z(),
            255
        };

        ++idV;
    });
    return idV;
}

size_t K4FullFrameUncompressor::convert_to_cloud(const std::vector<geo::Pt3<uint8_t>> &uncompressedColor, const std::vector<uint16_t> &uncompressedDepth, K4VertexMeshData *vertices){

    // resize depth indices
    if(indicesDepths1D.size() < uncompressedDepth.size()){
        indicesDepths1D.resize(uncompressedDepth.size());
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);
    }

    auto cloudBuffer = cloud_data();
    size_t idV = 0;

    for_each(std::execution::unseq, std::begin(indicesDepths1D), std::begin(indicesDepths1D) + uncompressedDepth.size(), [&](size_t id){

        if(uncompressedDepth[id] == k4_invalid_depth_value){
            return;
        }

        vertices[idV].pos = geo::Pt3f{
            static_cast<float>(-cloudBuffer[id].x()),
            static_cast<float>(-cloudBuffer[id].y()),
            static_cast<float>( cloudBuffer[id].z())
        }*0.001f;

        vertices[idV].col = {
            uncompressedColor[id].x(),
            uncompressedColor[id].y(),
            uncompressedColor[id].z(),
            255
        };

        ++idV;
    });

    return idV;
}


geo::Pt3<int16_t> *K4FullFrameUncompressor::cloud_data(){
    if(i->pointCloudImage.is_valid()){
        return reinterpret_cast<Pt3<int16_t>*>(i->pointCloudImage.get_buffer());
    }
    return nullptr;
}


void K4CloudFrameUncompressor::update_id_array(size_t idV){
    if(indicesValid1D.size() < idV){
        indicesValid1D.resize(idV);
        std::iota(std::begin(indicesValid1D), std::end(indicesValid1D), 0);
    }
}

bool K4CloudFrameUncompressor::uncompress_color(K4CompressedCloudFrame *cFrame, std::vector<geo::Pt3<uint8_t>> &uncompressedColor){
    return K4FrameUncompressor::uncompress_color(
        cFrame->colorWidth,
        cFrame->colorHeight,
        cFrame->encodedColorData,
        uncompressedColor
    );
}

bool K4CloudFrameUncompressor::uncompress_vertices(K4CompressedCloudFrame *cFrame, std::vector<std::uint16_t> &decodedVertices){

    // resize decoded data
    const auto idV = cFrame->validVerticesCount;
    const size_t paddedDiff = 128-((idV*3)%128);
    const size_t decodedBufferSize = idV*3 + paddedDiff;
    if(decodedVertices.size() < decodedBufferSize){
        decodedVertices.resize(decodedBufferSize);
    }

    size_t decodedBytesNb = p4nzdec128v16(
        cFrame->encodedCloudData.data(),
        decodedBufferSize,
        decodedVertices.data());
    if(decodedBytesNb == 0){
        Logger::error("[CloudFrameUncompressor] Error decoding vertices.\n");
        return false;
    }

    return true;
}



void K4CloudFrameUncompressor::convert_to_cloud(K4CompressedCloudFrame *cFrame, K4CloudFrame &uFrame){

    const auto idV = cFrame->validVerticesCount;
    update_id_array(idV);

    uFrame.idCapture = cFrame->idCapture;
    uFrame.afterCaptureTS = cFrame->afterCaptureTS;
    uFrame.cloud.validVerticesCount = idV;
    uFrame.cloud.vertices.resize(idV);
    uFrame.cloud.colors.resize(idV);

    for_each(std::execution::par_unseq, std::begin(indicesValid1D), std::begin(indicesValid1D) + idV, [&](size_t id){

        uFrame.cloud.vertices[id]= geo::Pt3f{
            -(static_cast<float>(decodedVerticesData[        id])-4096),
            -(static_cast<float>(decodedVerticesData[idV   + id])-4096),
              static_cast<float>(decodedVerticesData[2*idV + id])
        }*0.001f;

        uFrame.cloud.colors[id] = geo::Pt3f{
            static_cast<float>(decodedColorData[id].x()),
            static_cast<float>(decodedColorData[id].y()),
            static_cast<float>(decodedColorData[id].z())
        }/255.f;
    });

}

void K4CloudFrameUncompressor::convert_to_cloud(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3f *colors){

    const auto idV = cFrame->validVerticesCount;
    update_id_array(idV);

    for_each(std::execution::par_unseq, std::begin(indicesValid1D), std::begin(indicesValid1D) + idV, [&](size_t id){

        vertices[id]= geo::Pt3f{
            -(static_cast<float>(decodedVerticesData[        id])-4096),
            -(static_cast<float>(decodedVerticesData[idV   + id])-4096),
            static_cast<float>(decodedVerticesData[2*idV + id])
        }*0.001f;

        colors[id] = geo::Pt3f{
              static_cast<float>(decodedColorData[id].x()),
              static_cast<float>(decodedColorData[id].y()),
              static_cast<float>(decodedColorData[id].z())
          }/255.f;
    });
}

void K4CloudFrameUncompressor::convert_to_cloud(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4f *colors){

    const auto idV = cFrame->validVerticesCount;
    update_id_array(idV);

    for_each(std::execution::par_unseq, std::begin(indicesValid1D), std::begin(indicesValid1D) + idV, [&](size_t id){

        vertices[id]= geo::Pt3f{
            -(static_cast<float>(decodedVerticesData[        id])-4096),
            -(static_cast<float>(decodedVerticesData[idV   + id])-4096),
            static_cast<float>(decodedVerticesData[2*idV + id])
        }*0.001f;

        colors[id] = geo::Pt4f{
            static_cast<float>(decodedColorData[id].x()),
            static_cast<float>(decodedColorData[id].y()),
            static_cast<float>(decodedColorData[id].z()),
            255.f
         }/255.f;
    });
}

void K4CloudFrameUncompressor::convert_to_cloud(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3<uint8_t> *colors){
    const auto idV = cFrame->validVerticesCount;
    update_id_array(idV);

    for_each(std::execution::par_unseq, std::begin(indicesValid1D), std::begin(indicesValid1D) + idV, [&](size_t id){

        vertices[id]= geo::Pt3f{
            -(static_cast<float>(decodedVerticesData[        id])-4096),
            -(static_cast<float>(decodedVerticesData[idV   + id])-4096),
            static_cast<float>(decodedVerticesData[2*idV + id])
        }*0.001f;

        colors[id] = {
            decodedColorData[id].x(),
            decodedColorData[id].y(),
            decodedColorData[id].z(),
        };
    });
}

void K4CloudFrameUncompressor::convert_to_cloud(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4<uint8_t> *colors){
    const auto idV = cFrame->validVerticesCount;
    update_id_array(idV);

    for_each(std::execution::par_unseq, std::begin(indicesValid1D), std::begin(indicesValid1D) + idV, [&](size_t id){

        vertices[id]= geo::Pt3f{
            -(static_cast<float>(decodedVerticesData[        id])-4096),
            -(static_cast<float>(decodedVerticesData[idV   + id])-4096),
            static_cast<float>(decodedVerticesData[2*idV + id])
        }*0.001f;

        colors[id] = {
            decodedColorData[id].x(),
            decodedColorData[id].y(),
            decodedColorData[id].z(),
            255
        };
    });
}

void K4CloudFrameUncompressor::convert_to_cloud(K4CompressedCloudFrame *cFrame, K4VertexMeshData *vertices){

    const auto idV = cFrame->validVerticesCount;
    update_id_array(idV);

    for_each(std::execution::par_unseq, std::begin(indicesValid1D), std::begin(indicesValid1D) + idV, [&](size_t id){

        vertices[id].pos = geo::Pt3f{
            -(static_cast<float>(decodedVerticesData[        id])-4096),
            -(static_cast<float>(decodedVerticesData[idV   + id])-4096),
            static_cast<float>(decodedVerticesData[2*idV + id])
        }*0.001f;

        vertices[id].col = {
            decodedColorData[id].x(),
            decodedColorData[id].y(),
            decodedColorData[id].z(),
            255
        };
    });
}

bool K4CloudFrameUncompressor::uncompress(K4CompressedCloudFrame *cFrame){
    if(!uncompress_color(cFrame, decodedColorData)){
        return false;
    }
    if(!uncompress_vertices(cFrame, decodedVerticesData)){
        return false;
    }
    return true;
}

bool K4CloudFrameUncompressor::uncompress(K4CompressedCloudFrame *cFrame, K4CloudFrame &uFrame){
    if(uncompress(cFrame)){
        convert_to_cloud(cFrame, uFrame);
        return true;
    }
    return false;
}

bool K4CloudFrameUncompressor::uncompress(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3f *colors){
    if(uncompress(cFrame)){
        convert_to_cloud(cFrame, vertices, colors);
        return true;
    }
    return false;
}

bool K4CloudFrameUncompressor::uncompress(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4f *colors){
    if(uncompress(cFrame)){
        convert_to_cloud(cFrame, vertices, colors);
        return true;
    }
    return false;
}

bool K4CloudFrameUncompressor::uncompress(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt3<uint8_t> *colors){
    if(uncompress(cFrame)){
        convert_to_cloud(cFrame, vertices, colors);
        return true;
    }
    return false;
}

bool K4CloudFrameUncompressor::uncompress(K4CompressedCloudFrame *cFrame, geo::Pt3f *vertices, geo::Pt4<uint8_t> *colors){
    if(uncompress(cFrame)){
        convert_to_cloud(cFrame, vertices, colors);
        return true;
    }
    return false;
}

bool K4CloudFrameUncompressor::uncompress(K4CompressedCloudFrame *cFrame, K4VertexMeshData *vertices){
    if(uncompress(cFrame)){
        convert_to_cloud(cFrame, vertices);
        return true;
    }
    return false;
}




