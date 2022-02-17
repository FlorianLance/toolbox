
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


#include "k4_volumetric_video_resource.hpp"

// std
#include <fstream>

// local
#include "utility/logger.hpp"
#include "utility/format.hpp"

using namespace tool::camera;

size_t K4VolumetricVideoResource::nb_cameras() const noexcept{
    return camData.size();
}

size_t K4VolumetricVideoResource::nb_frames(size_t idCamera) const noexcept {
    if(idCamera < camData.size()){
        return camData[idCamera].frames.size();
    }
    return 0;
}

K4CompressedFrame *K4VolumetricVideoResource::get_frame(size_t idFrame, size_t idCamera){
    if(idCamera < camData.size()){
        if(idFrame < camData[idCamera].frames.size()){
            return camData[idCamera].frames[idFrame].data.get();
        }
        Logger::error("[VolumetricVideoResource] Cannot get full frame, invalid frame id.\n");
        return nullptr;
    }
    Logger::error("[VolumetricVideoResource] Cannot get full frame, invalid camera id.\n");
    return nullptr;
}

std::vector<K4FrameData> *K4VolumetricVideoResource::get_frames(size_t idCamera){
    if(idCamera < camData.size()){
        return &camData[idCamera].frames;
    }
    Logger::error("[VolumetricVideoResource] Cannot get frame, invalid camera id.\n");
    return nullptr;
}


size_t K4VolumetricVideoResource::frame_id(size_t idCamera, float timeMs) const{

    const auto start = start_time(idCamera);
    size_t idFrame = 0;
    for(const auto &frame : camData[idCamera].frames){
        auto frameTimeMs = (frame.timeStamp-start)*0.000001f;
        if(timeMs < frameTimeMs){
            return idFrame;
        }
        idFrame = frame.idFrame;
    }
    return idFrame;
}

size_t K4VolumetricVideoResource::valid_vertices_count(size_t idFrame, size_t idCamera) const{
    if(idCamera < camData.size()){
        if(idFrame < camData[idCamera].frames.size()){
            return camData[idCamera].frames[idFrame].data->validVerticesCount;
        }
    }
    return 0;
}

std::int64_t K4VolumetricVideoResource::start_time(size_t idCamera) const{
    if(idCamera < camData.size()){
        if(camData[idCamera].frames.size() > 0){
            return camData[idCamera].frames[0].timeStamp;
        }
    }
    return -1;
}

int64_t K4VolumetricVideoResource::end_time(size_t idCamera) const{
    if(idCamera < camData.size()){
        if(camData[idCamera].frames.size() > 0){
            return camData[idCamera].frames[camData[idCamera].frames.size()-1].timeStamp;
        }
    }
    return -1;
}

void K4VolumetricVideoResource::set_transform(size_t idCamera, geo::Mat4d tr){
    if(idCamera < camData.size()){
        camData[idCamera].transform = tr;
        return;
    }
    Logger::error("[VolumetricVideoResource] Cannot set transform, invalid camera id.\n");
}

tool::geo::Mat4d K4VolumetricVideoResource::get_transform(size_t idCamera) const{
    if(idCamera < camData.size()){
        return camData[idCamera].transform;
    }
    Logger::error("[VolumetricVideoResource] Cannot get transform, invalid camera id.\n");
    return geo::Mat4d::identity();
}

void K4VolumetricVideoResource::remove_frames_until(size_t idFrame){
    if(idFrame < camData.size()){
        camData.erase(camData.begin(), camData.begin() + idFrame);
    }else{
        // error
    }
}

void K4VolumetricVideoResource::remove_frames_after(size_t idFrame){
    if(idFrame < camData.size()){
        camData.erase(camData.begin()+ idFrame, camData.end());
    }else{
        // error
    }
}

void K4VolumetricVideoResource::clean_frames(){
    camData.clear();
}

void K4VolumetricVideoResource::add_frame(size_t idCamera, std::int64_t timestamp, std::shared_ptr<K4CompressedFrame> frame){
    if(idCamera >= camData.size()){
        camData.resize(idCamera+1);
    }
    auto idFrame = camData[idCamera].frames.size();
    camData[idCamera].frames.emplace_back(K4FrameData{idFrame, timestamp, frame});
}



bool K4VolumetricVideoResource::save_to_file(const std::string &path){

    if(path.length() == 0){
        Logger::error("[VolumetricVideoResource] Invalid path.\n");
        return false;
    }

    if(camData.size() == 0){
        Logger::error("[VolumetricVideoResource] No frames added.\n");
        return false;
    }

    std::ofstream file;
    file.open(path, std::ios_base::binary);
    if(!file.is_open()){
        Logger::error(fmt("[VolumetricVideoResource] Cannot save compressed frames to {}.\n", path));
        return false;
    }

    write_file(file);

    file.close();

    return true;
}

bool K4VolumetricVideoResource::load_from_file(const std::string &path){

    if(path.length() == 0){
        Logger::error("[VolumetricVideoResource] Invalid path.\n");
        return false;
    }

    std::ifstream file(path, std::ios_base::binary);
    if(!file.is_open()){
        Logger::error(fmt("[VolumetricVideoResource] Cannot open compressed frames file: {}.\n", path));
        return false;
    }

    // clean data
    clean_frames();

    return read_file(file);
}

