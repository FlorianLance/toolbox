
#include "k4_volumetric_cloud_video_resource.hpp"

// local
#include "utility/io.hpp"
#include "utility/logger.hpp"

using namespace tool::geo;
using namespace tool::camera;

K4CompressedCloudFrame *K4VolumetricCloudVideoResource::get_cloud_frame(size_t idFrame, size_t idCamera){
    auto frame = get_frame(idFrame, idCamera);
    if(frame != nullptr){
        return reinterpret_cast<K4CompressedCloudFrame*>(frame);
    }
    return nullptr;
}

void K4VolumetricCloudVideoResource::read_frame(std::ifstream &file, K4CompressedCloudFrame *fData){

    read(file, &fData->afterCaptureTS);

    // # read cloud
    std::int32_t validVerticesCount;
    read(file, &validVerticesCount);
    fData->validVerticesCount = validVerticesCount;

    std::int32_t cloudBufferSize;
    read(file, &cloudBufferSize);        
    fData->encodedCloudData.resize(cloudBufferSize);
    read_array(file, fData->encodedCloudData.data(), fData->encodedCloudData.size());

    // # read color
    std::int16_t colorWidth, colorHeight;
    std::int32_t colorBufferSize;
    read(file, &colorWidth);
    read(file, &colorHeight);
    read(file, &colorBufferSize);
    fData->colorWidth  = colorWidth;
    fData->colorHeight = colorHeight;
    fData->encodedColorData.resize(colorBufferSize);
    read_array(file, fData->encodedColorData.data(), fData->encodedColorData.size());

    // # read audio
    std::int32_t audioBufferSize;
    read(file, &audioBufferSize);
    fData->audioFrames.resize(audioBufferSize);
    read_array(file, reinterpret_cast<float*>(fData->audioFrames.data()),audioBufferSize*7);
    // # read imu
    read_array(file, reinterpret_cast<char*>(&fData->imuSample), sizeof (K4ImuSample));
}

void K4VolumetricCloudVideoResource::write_frame(std::ofstream &file, K4CompressedCloudFrame *fData){

    // # write cloud
    write(file, fData->afterCaptureTS);                                                 // std::int64_t
    write(file, static_cast<std::int32_t>(fData->validVerticesCount));                  // std::int32_t
    write(file, static_cast<std::int32_t>(fData->encodedCloudData.size()));             // std::int32_t
    write_array(file, fData->encodedCloudData.data(), fData->encodedCloudData.size());  // buffer size * std::int32_t

    // # write color
    write(file, static_cast<std::int16_t>(fData->colorWidth));                          // std::int16_t
    write(file, static_cast<std::int16_t>(fData->colorHeight));                         // std::int16_t
    write(file, static_cast<std::int32_t>(fData->encodedColorData.size()));             // std::int32_t
    write_array(file, fData->encodedColorData.data(), fData->encodedColorData.size());  // buffer size * std::int8_t

    // # write audio
    write(file, static_cast<std::int32_t>(fData->audioFrames.size()));                  // std::int32_t
    if(fData->audioFrames.size() > 0){
        write_array(file,
            reinterpret_cast<float*>(fData->audioFrames.data()),
            fData->audioFrames.size()*7);                                       // buffer size * 7 * float
    }
    // # write imu
    write_array(file, reinterpret_cast<char*>(&fData->imuSample), sizeof (K4ImuSample));
}

bool K4VolumetricCloudVideoResource::read_file(std::ifstream &file){

    // read mode
    std::int8_t videoType;
    read(file, &videoType);
    if(videoType != 1){
        Logger::error("[VolumetricCloudVideoResource] Invalid video type.\n");
        return false;
    }

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
            cameraData.frames.emplace_back(K4FrameData{0,0, std::make_unique<K4CompressedCloudFrame>()});
        }

        // calibration matrix
        read_array(file, cameraData.transform.array.data(), 16);
    }

    // read frames
    for(auto &cameraData : camData){
        for(auto &frame : cameraData.frames){

            std::int32_t idFrame;
            std::int64_t timestamp;
            // # read frame info
            read(file, &idFrame);
            frame.idFrame = idFrame;
            read(file, &timestamp);
            frame.timeStamp = timestamp;

            // read frame
            read_frame(file, reinterpret_cast<K4CompressedCloudFrame*>(frame.data.get()));
        }
    }

    return true;
}


void K4VolumetricCloudVideoResource::write_file(std::ofstream &file){

    // write video type
    std::int8_t videoType = 1;
    write(file, videoType);                                                             // std::int8_t

    // write nb of cameras
    write(file, static_cast<std::int8_t>(camData.size()));                              // std::int8_t

    // write infos per camera
    for(auto &cameraData : camData){
        // nb frames
        write(file, static_cast<std::int32_t>(cameraData.frames.size()));               // std::int32_t * cameras count
        // calibration matrix
        write_array(file, cameraData.transform.array.data(), 16);                       // double * 16
    }

    // writes frames
    for(const auto &cameraData : camData){
        for(const auto &frame : cameraData.frames){

            // # write frame info
            write(file, static_cast<std::int32_t>(frame.idFrame));                      // std::int32_t
            write(file, frame.timeStamp);                                               // std::int64_t

            // write frame
            write_frame(file, reinterpret_cast<K4CompressedCloudFrame*>(frame.data.get()));
        }
    }
}


