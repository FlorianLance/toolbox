
#include "k4_volumetric_full_video_resource.hpp"

// local
#include "utility/io.hpp"
#include "utility/logger.hpp"


using namespace tool::geo;
using namespace tool::camera;


void K4VolumetricFullVideoResource::read_frame(std::ifstream &file, K4CompressedFullFrame *fData){

    read(file, &fData->afterCaptureTS);
    read(file, &fData->mode);
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
    fData->colorWidth  = colorWidth;
    fData->colorHeight = colorHeight;
    fData->encodedColorData.resize(colorBufferSize);
    read_array(file, fData->encodedColorData.data(), fData->encodedColorData.size());
    // # read depth
    std::int16_t depthWidth, depthHeight;
    std::int32_t depthBufferSize;
    read(file, &depthWidth);
    read(file, &depthHeight);
    read(file, &depthBufferSize);
    fData->depthWidth  = depthWidth;
    fData->depthHeight = depthHeight;
    fData->encodedDepthData.resize(depthBufferSize);
    read_array(file, fData->encodedDepthData.data(), fData->encodedDepthData.size());
    // # read infra
    std::int16_t infraWidth, infraHeight;
    std::int32_t infraBufferSize;
    read(file, &infraWidth);
    read(file, &infraHeight);
    read(file, &infraBufferSize);
    fData->infraWidth  = infraWidth;
    fData->infraHeight = infraHeight;
    fData->encodedInfraData.resize(infraBufferSize);
    read_array(file, fData->encodedInfraData.data(), fData->encodedInfraData.size());

    // # read audio
    std::int32_t audioBufferSize;
    read(file, &audioBufferSize);
    fData->audioFrames.resize(audioBufferSize);
    read_array(file, reinterpret_cast<float*>(fData->audioFrames.data()),audioBufferSize*7);
    // # read imu
    read_array(file, reinterpret_cast<char*>(&fData->imuSample), sizeof (K4ImuSample));
}

bool K4VolumetricFullVideoResource::read_file(std::ifstream &file){

    // read mode
    std::int8_t videoType;
    read(file, &videoType);
    if(videoType != 0){
        // log error
        Logger::error("[VolumetricFullVideoResource] Invalid video type.\n");
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
            cameraData.frames.emplace_back(K4FrameData{0,0, std::make_unique<K4CompressedFullFrame>()});
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
            read_frame(file, reinterpret_cast<K4CompressedFullFrame*>(frame.data.get()));
        }
    }

    return true;
}


void K4VolumetricFullVideoResource::write_frame(std::ofstream &file, K4CompressedFullFrame *fData){

    write(file, fData->afterCaptureTS);                                             // std::int64_t
    write(file, static_cast<std::int32_t>(fData->mode));                            // std::int32_t
    write_array(file, reinterpret_cast<char*>(&fData->calibration),
                sizeof (k4a_calibration_t));                                        // sizeof(k4a_calibration_t)
    write(file, static_cast<std::int32_t>(fData->validVerticesCount));              // std::int32_t
    // # write color
    write(file, static_cast<std::int16_t>(fData->colorWidth));                      // std::int16_t
    write(file, static_cast<std::int16_t>(fData->colorHeight));                     // std::int16_t
    write(file, static_cast<std::int32_t>(fData->encodedColorData.size()));         // std::int32_t
    write_array(file, fData->encodedColorData.data(),
                fData->encodedColorData.size());                                    // buffer size * std::int8_t
    // # write depth
    write(file, static_cast<std::int16_t>(fData->depthWidth));                      // std::int16_t
    write(file, static_cast<std::int16_t>(fData->depthHeight));                     // std::int16_t
    write(file, static_cast<std::int32_t>(fData->encodedDepthData.size()));         // std::int32_t
    write_array(file, fData->encodedDepthData.data(),
                fData->encodedDepthData.size());                                    // buffer size * std::int32_t
    // # write infra
    write(file, static_cast<std::int16_t>(fData->infraWidth));                      // std::int16_t
    write(file, static_cast<std::int16_t>(fData->infraHeight));                     // std::int16_t
    write(file, static_cast<std::int32_t>(fData->encodedInfraData.size()));         // std::int32_t
    write_array(file, fData->encodedInfraData.data(),
                fData->encodedInfraData.size());                                    // buffer size * std::int32_t

    // # write audio
    write(file, static_cast<std::int32_t>(fData->audioFrames.size()));              // std::int32_t
    if(fData->audioFrames.size() > 0){
        write_array(file, reinterpret_cast<float*>(fData->audioFrames.data()),
                    fData->audioFrames.size()*7);                                   // buffer size * 7 * float
    }
    // # write imu
    write_array(file, reinterpret_cast<char*>(&fData->imuSample), sizeof (K4ImuSample));
}


void K4VolumetricFullVideoResource::write_file(std::ofstream &file){

    // write video type
    std::int8_t videoType = 0;
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
            write_frame(file, reinterpret_cast<K4CompressedFullFrame*>(frame.data.get()));
        }
    }
}

K4CompressedFullFrame *K4VolumetricFullVideoResource::get_full_frame(size_t idFrame, size_t idCamera){
    auto frame = get_frame(idFrame, idCamera);
    if(frame != nullptr){
        return reinterpret_cast<K4CompressedFullFrame*>(frame);
    }
    return nullptr;
}
