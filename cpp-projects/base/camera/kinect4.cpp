
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


#include "kinect4.hpp"

// std
#include <thread>
#include <execution>
#include <format>

// kinect4
#include <k4a/k4a.hpp>

// local
// # kinect4
#include "k4a/k4astaticimageproperties.h"
#include "k4a/k4amicrophonelistener.h"
#include "k4a/k4aaudiomanager.h"
#include "k4a/k4aaudiochanneldatagraph.h"
// # utility
#include "utility/logger.hpp"
#include "utility/benchmark.hpp"
#include "utility/vector.hpp"
// # geometry
#include "geometry/point4.hpp"
// # camera
#include "camera/frame_compressor.hpp"

using namespace tool;
using namespace tool::geo;
using namespace tool::camera;
using namespace tool::camera::K4;

auto nanoseconds_since_epoch(){
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
}

struct Kinect4::Impl{

    Kinect4 *kinect4 = nullptr;

    // device
    uint32_t deviceCount = 0;
    k4a::device device = nullptr;
    k4a::calibration calibration;    
    k4a::transformation transformation;    
    k4a_device_configuration_t k4aConfig;
    K4::Config config;
    std::unique_ptr<k4a::capture> capture = nullptr;

    // images
    // # capture
    std::optional<k4a::image> colorImage      = std::nullopt;
    std::optional<k4a::image> depthImage      = std::nullopt;
    std::optional<k4a::image> infraredImage   = std::nullopt;
    std::optional<k4a::image> pointCloudImage = std::nullopt;
    // # processing
    std::optional<k4a::image> convertedColorImage = std::nullopt;
    std::optional<k4a::image> depthSizedColorImage = std::nullopt;

    // audio
    std::shared_ptr<k4a::K4AMicrophone> microphone = nullptr;
    std::shared_ptr<k4a::K4AMicrophoneListener> audioListener = nullptr;
    size_t lastFrameCount = 0;
    std::vector<k4a::K4AMicrophoneFrame> audioFrames;

    // imu
    ImuSample imuSample;

    // parameters
    Parameters parameters;

    // infos
    size_t colorWidth   = 0;
    size_t colorHeight  = 0;
    size_t colorSize    = 0;
    size_t depthWidth   = 0;
    size_t depthHeight  = 0;
    size_t depthSize    = 0;
    ColorResolution colorResolution;
    ImageFormat imageFormat;
    DepthMode depthMode;

    // state
    std::atomic_bool readFramesFromCameras = false;
    float temperature = 0.f;
    size_t validDepthValues = 0;

    // arrays indices
    std_v1<size_t> indicesDepths1D;
    std_v1<size_t> indicesDepths1DNoBorders;
    std_v1<geo::Pt3<size_t>> indicesDepths3D;
    std_v1<size_t> indicesColors1D;
    std_v1<bool> depthMask;

    // compression
    FullFrameCompressor ffc;
    CloudFrameCompressor cfc;

    // display frames
    size_t currentDisplayFramesId = 0;
    std::vector<std::shared_ptr<DisplayDataFrame>> displayFrames;

    // thread/lockers
    std::mutex parametersM; /**< mutex for reading parameters at beginning of a new frame in thread function */
    std::unique_ptr<std::thread> frameReaderT = nullptr;

    // functions
    void read_frames(K4::Mode mode);
    // # init data
    void init_data(K4::Mode mode);
    // # read data
    void read_from_microphones();
    void read_from_imu();
    // # get images
    bool get_color_image();
    bool get_depth_image();
    bool get_infra_image(Mode mode);
    // # processing
    void convert_color_image(const Parameters &p);
    void resize_color_to_fit_depth();
    void filter_depth_image(const K4::Parameters &p, K4::Mode mode);
    void filter_color_image(const K4::Parameters &p);
    void filter_infrared_image(const K4::Parameters &p);
    void generate_cloud(Mode mode);
    void compress_cloud_frame();
    void compress_full_frame(Mode mode);
    void display_frame(const Parameters &p, Mode mode);
};


k4a_device_configuration_t Kinect4::generate_config(const K4::Config &config){

    k4a_device_configuration_t ka4Config        = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    ka4Config.color_format                      = static_cast<k4a_image_format_t>(image_format(config.mode));
    ka4Config.color_resolution                  = static_cast<k4a_color_resolution_t>(color_resolution(config.mode));
    ka4Config.depth_mode                        = static_cast<k4a_depth_mode_t>(depth_mode(config.mode));
    ka4Config.camera_fps                        = static_cast<k4a_fps_t>(framerate(config.mode));

    ka4Config.synchronized_images_only = false;

    if(depth_mode(config.mode) == DepthMode::OFF){
        ka4Config.synchronized_images_only = false;
    }else{
        ka4Config.synchronized_images_only = config.synchronizeColorAndDepth;
    }

    ka4Config.depth_delay_off_color_usec        = config.delayBetweenColorAndDepthUsec;
    ka4Config.wired_sync_mode                   = static_cast<k4a_wired_sync_mode_t>(config.synchMode);;
    ka4Config.subordinate_delay_off_master_usec = config.subordinateDelayUsec;
    ka4Config.disable_streaming_indicator       = config.disableLED;
    return ka4Config;
}

k4a_device_configuration_t Kinect4::generate_config(
    ImageFormat colFormat,
    ColorResolution colResolution,
    DepthMode depthMode,
    Framerate fps,
    bool synchronizeColorAndDepth,
    int delayBetweenColorAndDepthUsec,
    SynchronisationMode synchMode,
    int subordinateDelayUsec,
    bool disableLED){

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format                      = static_cast<k4a_image_format_t>(colFormat);
    config.color_resolution                  = static_cast<k4a_color_resolution_t>(colResolution);
    config.depth_mode                        = static_cast<k4a_depth_mode_t>(depthMode);
    config.camera_fps                        = static_cast<k4a_fps_t>(fps);

    if(depthMode == DepthMode::OFF){
        config.synchronized_images_only = false;
    }else{
        config.synchronized_images_only = synchronizeColorAndDepth;
    }
    config.depth_delay_off_color_usec        = delayBetweenColorAndDepthUsec;
    config.wired_sync_mode                   = static_cast<k4a_wired_sync_mode_t>(synchMode);;
    config.subordinate_delay_off_master_usec = subordinateDelayUsec;
    config.disable_streaming_indicator       = disableLED;

    Logger::message(std::format("config.color_format: {}\n", static_cast<int>(config.color_format)));
    Logger::message(std::format("config.color_resolution: {}\n", static_cast<int>(config.color_resolution)));
    Logger::message(std::format("config.depth_mode: {}\n", static_cast<int>(config.depth_mode)));
    Logger::message(std::format("config.camera_fps: {}\n", static_cast<int>(config.camera_fps)));

    return config;
}

Kinect4::Kinect4() : i(std::make_unique<Impl>()){

    i->kinect4 = this;
    i->deviceCount = k4a::device::get_installed_count();
    if(i->deviceCount == 0){
        Logger::error("No K4A devices found\n");
    }else{
        Logger::message(std::format("Devices found: {}\n", i->deviceCount));
    }


    const int audioInitStatus = k4a::K4AAudioManager::Instance().Initialize();
    if (audioInitStatus != SoundIoErrorNone){
        Logger::error("Failed to initialize audio backend: {}\n", soundio_strerror(audioInitStatus));
    }else{
        size_t nbDevices = k4a::K4AAudioManager::Instance().get_devices_count();
        Logger::message(std::format("Audio devices count: {}\n", nbDevices));

        for(size_t ii = 0; ii < nbDevices; ++ii){
            std::string deviceName = k4a::K4AAudioManager::Instance().get_device_name(ii);
            Logger::message(std::format(" - {}\n", deviceName));
            if (deviceName.find("Azure Kinect Microphone Array") != std::string::npos) {
                Logger::message(std::format("Found Azure kinect microphones array.\n"));
                i->microphone = k4a::K4AAudioManager::Instance().get_microphone_for_device(deviceName);
                if(i->microphone == nullptr){
                    Logger::error(std::format("Cannot init microphone.\n"));
                    i->audioListener = nullptr;
                    return;
                }
                i->microphone->Start();
                if(i->microphone->IsStarted()){
                    i->audioListener = i->microphone->CreateListener();
                }else{
                    Logger::error(std::format("Cannot start microphone.\n"));
                }
//                i->audioListener =  std::make_shared<k4a::K4AMicrophoneListener>(i->microphone);
                if(i->audioListener == nullptr){
                    Logger::error(std::format("Cannot init audio listener.\n"));
                    return;
                }
                break;
            }
        }

    }
}

Kinect4::~Kinect4(){

    if(is_opened()){
        if(is_reading_frames()){
            stop_reading();
        }
        stop_cameras();
        close();
    }
}

bool Kinect4::open(uint32_t deviceId){

    if(deviceId >= i->deviceCount){
        Logger::error("Invalid device id\n");
        return false;
    }

    try {
        i->device = k4a::device::open(deviceId);
    }  catch (std::runtime_error error) {
        Logger::error(std::format("[Kinect4] open error: {}\n", error.what()));
        return false;
    }

    const auto version = i->device.get_version();
    const auto fb = version.firmware_build;
    const auto fs = version.firmware_signature;
    bool debugFB =  fb == K4A_FIRMWARE_BUILD_RELEASE;
    std::string fsStr;
    switch (fs) {
    case K4A_FIRMWARE_SIGNATURE_MSFT:
        fsStr = "Microsoft signed";
        break;
    case K4A_FIRMWARE_SIGNATURE_TEST:
        fsStr = "Test signed";
        break;
    case K4A_FIRMWARE_SIGNATURE_UNSIGNED:
        fsStr = "Unsigned";
        break;
    }

    Logger::message("Opened device:\n");
    Logger::message(std::format("  Serialnum: {}\n", i->device.get_serialnum()));
    Logger::message("  Version:\n");
    Logger::message(std::format("      Firmware build: {}\n", (debugFB ? "[debug]" : "[release]")));
    Logger::message(std::format("      Firmware signature: {}\n", fsStr));
    Logger::message(std::format("      Color camera firmware version {}.{}\n", version.rgb.major, version.rgb.minor));
    Logger::message(std::format("      Depth camera firmware version {}.{}\n", version.depth.major, version.depth.minor));
    Logger::message(std::format("      Audio device firmware version {}.{}\n", version.audio.major, version.audio.minor));
    Logger::message(std::format("      Depth device firmware version {}.{}\n", version.depth_sensor.major, version.depth_sensor.minor));
    Logger::message("  Synch:\n");
    Logger::message(std::format("      IN connected {}\n", i->device.is_sync_in_connected()));
    Logger::message(std::format("      OUT connected {}\n", i->device.is_sync_out_connected()));

    return true;
}

bool Kinect4::is_opened() const{
    return i->device.is_valid();
}

bool Kinect4::is_reading_frames() const{return i->readFramesFromCameras;}

void Kinect4::close(){

    if(i->microphone){
        if(i->microphone->IsStarted()){
            i->microphone->Stop();
        }
        if(i->audioListener){
            i->audioListener = nullptr;
        }
    }

    if(i->readFramesFromCameras){
        Logger::error("Reading must be stopped before closing the device.\n");
        return;
    }
    i->device.close();
}


void Kinect4::start_reading(){

    if(!is_opened()){
        Logger::error("Device must be opened for reading.\n");
        return;
    }

    if(i->frameReaderT == nullptr){
        Logger::message("Start reading frames.\n");
        i->frameReaderT = std::make_unique<std::thread>(&Kinect4::Impl::read_frames, i.get(), i->config.mode);
    }else{
        Logger::error("Reading thread already started.\n");
    }
}

void Kinect4::stop_reading(){

    i->readFramesFromCameras = false;
    if(i->frameReaderT != nullptr){
        if(i->frameReaderT->joinable()){
            i->frameReaderT->join();
        }
        i->frameReaderT = nullptr;
    }
}


void Kinect4::set_parameters(const Parameters &parameters){
    i->parametersM.lock();
    i->parameters = parameters;
    i->parametersM.unlock();
}

bool Kinect4::start_cameras(const K4::Config &config){
    return start_cameras(i->k4aConfig = generate_config(i->config = config));
}

bool Kinect4::start_cameras(const k4a_device_configuration_t &k4aConfig){

    // set config
    i->k4aConfig = k4aConfig;

    try {

        Logger::message("[Kinect4] retrieve calibration\n");
        i->calibration = i->device.get_calibration(i->k4aConfig.depth_mode, i->k4aConfig.color_resolution);

        const auto &c = i->calibration;
        Logger::message("Calibration:\n");
        Logger::message(std::format("  color resolution: {}\n", static_cast<int>(c.color_resolution)));
        Logger::message("  color camera:\n");
        Logger::message(std::format("      width: {}\n", c.color_camera_calibration.resolution_width));
        Logger::message(std::format("      height: {}\n", c.color_camera_calibration.resolution_height));
        Logger::message(std::format("      metric radius: {}\n", c.color_camera_calibration.metric_radius));

        Logger::message("  depth mode:\n");
        Logger::message(std::format("      width: {}\n", c.depth_camera_calibration.resolution_width));
        Logger::message(std::format("      height: {}\n", c.depth_camera_calibration.resolution_height));

        Logger::message("[Kinect4] start cameras\n");
        i->device.start_cameras(&i->k4aConfig);

        Logger::message("[Kinect4] start imu\n");
        i->device.start_imu();        

    }  catch (std::runtime_error error) {
        Logger::error("[Kinect4] start_cameras error: {]\n", error.what());
        i->k4aConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        return false;
    }

//    using ccc = k4a_color_control_command_t;
//    auto autoExposurePriority = ccc::K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY;
//    ccc::K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION;
//    ccc::K4A_COLOR_CONTROL_BRIGHTNESS;
//    ccc::K4A_COLOR_CONTROL_CONTRAST;
//    ccc::K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE;
//    ccc::K4A_COLOR_CONTROL_GAIN;
//    ccc::K4A_COLOR_CONTROL_POWERLINE_FREQUENCY;
//    ccc::K4A_COLOR_CONTROL_SATURATION;
//    ccc::K4A_COLOR_CONTROL_SHARPNESS;
//    ccc::K4A_COLOR_CONTROL_WHITEBALANCE;

    //        using ccm = k4a_color_control_mode_t;
    //        auto autoControl = k4a_color_control_mode_t::K4A_COLOR_CONTROL_MODE_AUTO;
    //        auto manualControl = k4a_color_control_mode_t::K4A_COLOR_CONTROL_MODE_MANUAL;

    //        try {
    //            i->device.set_color_control(autoExposurePriority, autoControl, 0);
    //        }  catch () {

    //        }

    return true;
}

void Kinect4::stop_cameras(){
    i->device.stop_cameras();
    i->device.stop_imu();
}


void Kinect4::Impl::read_frames(K4::Mode mode){

    // check device
    if(!device.is_valid() || readFramesFromCameras){
        Logger::error("[Kinect4] Cannot start reading frames.\n");
        return;
    }

    // initialization
    init_data(mode);

    // start loop
    readFramesFromCameras = true;
    while(readFramesFromCameras){

        // copy parameters
        parametersM.lock();
        const auto p = parameters;
        parametersM.unlock();

        // read data from device
        try {
            Bench::start("[Kinect4] Device get_capture");
            const int32_t timeoutMs = 300;
            bool success = device.get_capture(capture.get(), std::chrono::milliseconds(timeoutMs));
            Bench::stop();

            if(!success){
                Logger::error("[Kinect4] get_capture timeout\n");
                continue;
            }

            read_from_microphones();
            read_from_imu();
            temperature = capture->get_temperature_c();

        }  catch (std::runtime_error error) {
            Logger::error(std::format("[Kinect4] get_capture error: {}\n", error.what()));
            readFramesFromCameras = false;
            break;
        }

        // get images
        if(!get_color_image()){
            continue;
        }
        if(!get_depth_image()){
            continue;
        }
        if(!get_infra_image(mode)){
            continue;
        }

        // process
        convert_color_image(p);
        resize_color_to_fit_depth();
        filter_depth_image(p, mode);
        filter_color_image(p);
        filter_infrared_image(p);
        generate_cloud(mode);

        // send frames
        if(p.sendCompressedCloudFrame){
            compress_cloud_frame();
        }
        if(p.sendCompressedFullFrame){
            compress_full_frame(mode);
        }            
        if(p.sendDisplayColorFrame || p.sendDisplayDepthFrame || p.sendDisplayInfraredFrame || p.sendDisplayCloud){
            display_frame(p, mode);
        }
    }
    readFramesFromCameras = false;
}

void Kinect4::Impl::filter_depth_image(const Parameters &p, Mode mode){

    if(!depthImage.has_value()){
        return;
    }

    Bench::start("[Kinect4] Filter depth");

    // retrieve buffers
    auto depthBuffer = reinterpret_cast<int16_t*>(depthImage->get_buffer());
    geo::Pt4<uint8_t>* colorBuffer = colorImage.has_value() ? reinterpret_cast<geo::Pt4<uint8_t>*>(colorImage.value().get_buffer()) : nullptr;
    uint16_t* infraredBuffer = infraredImage.has_value() ? reinterpret_cast<uint16_t*>(infraredImage.value().get_buffer()) : nullptr;

    static_cast<void>(infraredBuffer);

    const auto dRange = range(mode)*1000.f;
    auto minD = p.minDepthValue < dRange.x() ? static_cast<std::int16_t>(dRange.x()) : p.minDepthValue;
    auto maxD = p.maxDepthValue > dRange.y() ? static_cast<std::int16_t>(dRange.y()) : p.maxDepthValue;

    const auto res = depth_resolution(mode);


    for_each(std::execution::par_unseq, std::begin(indicesDepths3D), std::end(indicesDepths3D), [&](const Pt3<size_t> &dIndex){

        const size_t id = dIndex.x();
        const size_t ii = dIndex.y();
        const size_t jj = dIndex.z();

        depthMask[id] = false;
        if(depthBuffer[id] == invalid_depth_value){
            return;
        }

        // depth filtering
        if(ii < p.minHeight){
            return;
        }else if(ii > p.maxHeight){
            return;
        }

        if(jj < p.minWidth){
            return;
        }else if(jj > p.maxWidth){
            return;
        }

        if(depthBuffer[id] < minD){
            return;
        }else if(depthBuffer[id] > maxD){
            return;
        }

        // color filtering
        if(colorImage.has_value() && p.filterDepthWithColor){
            auto delta = norm(colorBuffer[id].xyz().conv<int>()-p.filterColor.conv<int>());
            if(delta > p.maxDiffColor.x()){
                return;
            }

//            if(colorBuffer->x() == 0 && colorBuffer->y() == 0 && colorBuffer->z() == 0 && colorBuffer->w() == 0){
//                depthBuffer[id] = invalid_depth_value;
//                return;
//            }
        }

        // infrared filtering
        // ...

        depthMask[id] = true;
    });

    const bool doLocalDiffFiltering = true;
    const size_t widthPlusOne  = res.x() +1;
    const size_t widthMinusOne = res.x() -1;

    if(doLocalDiffFiltering){
        const float mLocal =  parameters.maxLocalDiff;
        for_each(std::execution::par_unseq, std::begin(indicesDepths1DNoBorders), std::end(indicesDepths1DNoBorders), [&](size_t id){

            if(!depthMask[id]){
                return;
            }

            // A B C
            // D I E
            // F G H
            float meanDiff = 0;
            size_t count = 0;
            float currDepth = depthBuffer[id];
            const size_t idA = id - widthPlusOne;
            const size_t idB = idA + 1;
            const size_t idC = idB + 1;
            const size_t idD = id - 1;
            const size_t idE = id + 1;
            const size_t idF = id + widthMinusOne;
            const size_t idG = idF + 1;
            const size_t idH = idG + 1;


            // B I G
            // A I H
            // F I C
//            float diffH = -1.f; // D I E
//            if(depthMask[idD] && depthMask[idE]){
//                diffH = (abs(depthBuffer[idD]-currDepth) + abs(depthBuffer[idE]-currDepth))/3.f;
//            }
//            if(diffH > mLocal){
//                depthMask[id] = false;
//                return;
//            }

//            float diffV = -1.f; // B I G
//            if(depthMask[idB] && depthMask[idG]){
//                diffV = (abs(depthBuffer[idB]-currDepth) + abs(depthBuffer[idG]-currDepth))/3.f;
//            }
//            if(diffV > mLocal){
//                depthMask[id] = false;
//                return;
//            }

//            float diffD1 = -1.f; // A I H
//            if(depthMask[idA] && depthMask[idH]){
//                diffD1 = (abs(depthBuffer[idA]-currDepth) + abs(depthBuffer[idH]-currDepth))/3.f;
//            }
//            if(diffD1 > mLocal){
//                depthMask[id] = false;
//                return;
//            }

//            float diffD2 = -1.f; // F I C
//            if(depthMask[idF] && depthMask[idC]){
//                diffD2 = (abs(depthBuffer[idF]-currDepth) + abs(depthBuffer[idC]-currDepth))/3.f;
//            }
//            if(diffD2 > mLocal){
//                depthMask[id] = false;
//                return;
//            }
//            depthMask[id] = (count == 0) ? false : (1.*meanDiff/count < mLocal);

            if(depthMask[idA]){
                meanDiff += abs(depthBuffer[idA]-currDepth);
                ++count;
            }
            if(depthMask[idB]){
                meanDiff += abs(depthBuffer[idB]-currDepth);
                ++count;
            }
            if(depthMask[idC]){
                meanDiff += abs(depthBuffer[idC]-currDepth);
                ++count;
            }
            if(depthMask[idD]){
                meanDiff += abs(depthBuffer[idD]-currDepth);
                ++count;
            }
            if(depthMask[idE]){
                meanDiff += abs(depthBuffer[idE]-currDepth);
                ++count;
            }
            if(depthMask[idF]){
                meanDiff += abs(depthBuffer[idF]-currDepth);
                ++count;
            }
            if(depthMask[idG]){
                meanDiff += abs(depthBuffer[idG]-currDepth);
                ++count;
            }
            if(depthMask[idH]){
                meanDiff += abs(depthBuffer[idH]-currDepth);
                ++count;
            }

            depthMask[id] = (count == 0) ? false : (1.*meanDiff/count < mLocal);
        });
    }

    validDepthValues = 0;
    for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){
        if(!depthMask[id]){
            depthBuffer[id] = invalid_depth_value;
        }else{
            validDepthValues++;
        }
        // increment counter
//        validDepthValues.fetch_add(1, std::memory_order_relaxed);
    });

    Bench::stop();
}

void Kinect4::Impl::filter_color_image(const K4::Parameters &p){

    if(!colorImage.has_value()){
        return;
    }

    if(!depthImage.has_value() && !infraredImage.has_value()){
        return;
    }

    Bench::start("[Kinect4] Filter color");

    // retrieve buffers
    geo::Pt4<uint8_t>* colorBuffer = reinterpret_cast<geo::Pt4<uint8_t>*>(colorImage->get_buffer());
    int16_t*  depthBuffer    = depthImage.has_value() ? reinterpret_cast<int16_t*>(depthImage.value().get_buffer()) : nullptr;
    uint16_t* infraredBuffer = infraredImage.has_value() ? reinterpret_cast<uint16_t*>(infraredImage.value().get_buffer()) : nullptr;

    static_cast<void>(infraredBuffer);

    for_each(std::execution::par_unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){
        if(p.invalidateColorFromDepth){
            if(depthBuffer[id] == invalid_depth_value){
                colorBuffer[id] = invalid_color_value;
            }
        }
    });

    Bench::stop();
}

void Kinect4::Impl::filter_infrared_image(const K4::Parameters &p){

    if(!infraredImage.has_value()){
        return;
    }

    if(!colorImage.has_value() && !depthImage.has_value()){
        return;
    }

    Bench::start("[Kinect4] Filter infra");

    // retrieve buffers
    uint16_t* infraredBuffer = reinterpret_cast<uint16_t*>(infraredImage->get_buffer());
    geo::Pt4<uint8_t>* colorBuffer = colorImage.has_value() ? reinterpret_cast<geo::Pt4<uint8_t>*>(colorImage.value().get_buffer()) : nullptr;
    int16_t*  depthBuffer         = depthImage.has_value() ? reinterpret_cast<int16_t*>(depthImage.value().get_buffer()) : nullptr;

    static_cast<void>(colorBuffer);

    for_each(std::execution::par_unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){
        if(p.invalidateInfraFromDepth){
            if(depthBuffer[id] == invalid_depth_value){
                infraredBuffer[id] = invalid_infra_value;
            }
        }
    });

    Bench::stop();
}

void Kinect4::Impl::generate_cloud(Mode mode){

    if(has_cloud(mode) && pointCloudImage.has_value()){
        Bench::start("[Kinect4] Transformation depth_image_to_point_cloud");
        transformation.depth_image_to_point_cloud(depthImage.value(), K4A_CALIBRATION_TYPE_DEPTH, &pointCloudImage.value());
        Bench::stop();
    }
}

void Kinect4::Impl::compress_cloud_frame(){

    if(!colorImage.has_value() || !depthImage.has_value()){
        return;
    }

    tool::Bench::start("[Kinect4] Generate compressed cloud frame");
    auto compressedCloudFrame = cfc.compress(
        validDepthValues,
        parameters.jpegCompressionRate,
        colorImage, depthImage, pointCloudImage);

    if(compressedCloudFrame != nullptr){
        // add imu
        compressedCloudFrame->imuSample   = imuSample;
        // add audio frames
        if(lastFrameCount > 0){
            compressedCloudFrame->audioFrames.resize(lastFrameCount);
            auto ad1 = reinterpret_cast<float*>(audioFrames.data());
            auto ad2 = reinterpret_cast<float*>(compressedCloudFrame->audioFrames.data());
            std::copy(ad1, ad1 + 7*lastFrameCount, ad2);
        }
    }
    tool::Bench::stop();

    // send
    if(compressedCloudFrame != nullptr){
        kinect4->new_compressed_cloud_frame_signal(compressedCloudFrame);
    }
}

void Kinect4::Impl::compress_full_frame(Mode mode){

    if(!colorImage.has_value() || !depthImage.has_value() || !infraredImage.has_value()){
        return;
    }

    tool::Bench::start("[Kinect4] Generate compressed full frame");

    auto compressedFullFrame = ffc.compress(
        validDepthValues,
        parameters.jpegCompressionRate,
        colorImage, depthImage, infraredImage);

    if(compressedFullFrame != nullptr){
        // add infos
        compressedFullFrame->mode        = mode;
        compressedFullFrame->calibration = calibration;
        // add imu
        compressedFullFrame->imuSample   = imuSample;
        // add audio frames
        if(lastFrameCount > 0){
            compressedFullFrame->audioFrames.resize(lastFrameCount);
            auto ad1 = reinterpret_cast<float*>(audioFrames.data());
            auto ad2 = reinterpret_cast<float*>(compressedFullFrame->audioFrames.data());
            std::copy(ad1, ad1 + 7*lastFrameCount, ad2);
        }
    }
    tool::Bench::stop();

    // send
    if(compressedFullFrame != nullptr){
        kinect4->new_compressed_full_frame_signal(compressedFullFrame);
    }
}

void Kinect4::Impl::display_frame(const Parameters &p, Mode mode){

    // write frame
    tool::Bench::start("[Kinect4] Write display data frame");

    auto currDisplayFrame = displayFrames[currentDisplayFramesId++];
    currentDisplayFramesId = currentDisplayFramesId%displayFrames.size();

    auto dFrame = currDisplayFrame.get();

    // init depth frame
    const std_v1<Pt3f> depthGradient ={
        {0.f,0.f,1.f},
        {0.f,1.f,1.f},
        {0.f,1.f,0.f},
        {1.f,1.f,0.f},
        {1.f,0.f,0.f},
    };

    // send audio
    if(p.sendAudio && lastFrameCount != 0){
        // copy audio frames
        dFrame->audioFrames.resize(lastFrameCount);
        auto ad1 = reinterpret_cast<float*>(audioFrames.data());
        auto ad2 = reinterpret_cast<float*>(dFrame->audioFrames.data());
        std::copy(ad1, ad1 + 7*lastFrameCount, ad2);
    }

    // send color frame
    if(p.sendDisplayColorFrame && colorImage.has_value()){

        tool::Bench::start("[Kinect4] sendDisplayColorFrame");

        size_t width, height;
        if(depthImage.has_value()){
            width = depthImage->get_width_pixels();
            height = depthImage->get_height_pixels();
        }else{
            width = colorImage->get_width_pixels();
            height = colorImage->get_height_pixels();
        }

        dFrame->colorFrame.width  = width;
        dFrame->colorFrame.height = height;

        auto colorBuffer = reinterpret_cast<const geo::Pt4<uint8_t>*>(colorImage->get_buffer());
        std::vector<size_t> *ids;
        if(depthImage.has_value()){
            ids = &indicesDepths1D;
        }else{
            ids = &indicesColors1D;
        }

        for_each(std::execution::par_unseq, std::begin(*ids), std::end(*ids), [&](size_t id){
            dFrame->colorFrame.pixels[id] = {
                colorBuffer[id].z(),
                colorBuffer[id].y(),
                colorBuffer[id].x()
            };
        });

        tool::Bench::stop();
    }

    // send depth frame
    if(p.sendDisplayDepthFrame && depthImage.has_value()){

        tool::Bench::start("[Kinect4] sendDisplayDepthFrame");

        const size_t width = depthImage->get_width_pixels();
        const size_t height = depthImage->get_height_pixels();
        dFrame->depthFrame.width  = width;
        dFrame->depthFrame.height = height;

        auto depthBuffer = reinterpret_cast<const uint16_t*>(depthImage->get_buffer());
        const auto dRange = range(mode)*1000.f;
        const auto diff = dRange.y() - dRange.x();

        for_each(std::execution::par_unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            if(depthBuffer[id] == invalid_depth_value){
                dFrame->depthFrame.pixels[id] = {};
                return;
            }

            float vF = (static_cast<float>(depthBuffer[id]) - dRange.x())/diff;
            float intPart;
            float decPart = std::modf((vF*(depthGradient.size()-1)), &intPart);
            size_t idG = static_cast<size_t>(intPart);

            auto col = depthGradient[idG]*(1.f-decPart) + depthGradient[idG+1]*decPart;
            dFrame->depthFrame.pixels[id] = {
                static_cast<std::uint8_t>(255*col.x()),
                static_cast<std::uint8_t>(255*col.y()),
                static_cast<std::uint8_t>(255*col.z())
            };
        });

        tool::Bench::stop();
    }

    // send infrared frame
    if(p.sendDisplayInfraredFrame && infraredImage.has_value()){

        tool::Bench::start("[Kinect4] sendDisplayInfraredFrame");

        const size_t width = infraredImage->get_width_pixels();
        const size_t height = infraredImage->get_height_pixels();
        dFrame->infraredFrame.width  = width;
        dFrame->infraredFrame.height = height;

        auto infraBuffer = reinterpret_cast<const uint16_t*>(infraredImage->get_buffer());

        const float max = 2000;
        for_each(std::execution::par_unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            float vF = static_cast<float>(infraBuffer[id]);
            if(vF > max){
                vF = max;
            }
            vF/=max;
            dFrame->infraredFrame.pixels[id] = {
                static_cast<std::uint8_t>(255*vF),
                static_cast<std::uint8_t>(255*vF),
                static_cast<std::uint8_t>(255*vF)
            };
        });

        tool::Bench::stop();
    }

    if(p.sendDisplayCloud && pointCloudImage.has_value() && colorImage.has_value() && depthImage.has_value()){

        auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(pointCloudImage->get_buffer());
        auto colorBuffer = reinterpret_cast<const geo::Pt4<uint8_t>*>(colorImage->get_buffer());
        auto depthBuffer = reinterpret_cast<const uint16_t*>(depthImage->get_buffer());

        dFrame->cloud.validVerticesCount = validDepthValues;

        size_t idV = 0;
        for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            if(depthBuffer[id] == invalid_depth_value){
                return;
            }

            dFrame->cloud.vertices[idV]= geo::Pt3f{
                static_cast<float>(-cloudBuffer[id].x()),
                static_cast<float>(-cloudBuffer[id].y()),
                static_cast<float>( cloudBuffer[id].z())
            }*0.001f;
            dFrame->cloud.colors[idV] = geo::Pt3f{
                static_cast<float>(colorBuffer[id].z()),
                static_cast<float>(colorBuffer[id].y()),
                static_cast<float>(colorBuffer[id].x())
            }/255.f;

            ++idV;
        });
    }

    // copy imu sample
    dFrame->imuSample = imuSample;

    tool::Bench::stop();

    kinect4->new_display_frame_signal(currDisplayFrame);
}


void Kinect4::Impl::init_data(K4::Mode mode){

    // init capture
    capture = std::make_unique<k4a::capture>();

    // init transform
    transformation = k4a::transformation(calibration);

    // reset images
    // # capture
    colorImage           = std::nullopt;
    depthImage           = std::nullopt;
    infraredImage        = std::nullopt;
    pointCloudImage      = std::nullopt;
    // # processing
    convertedColorImage  = std::nullopt;
    depthSizedColorImage = std::nullopt;

    // reset sizes
    colorWidth           = 0;
    colorHeight          = 0;
    colorSize            = 0;
    depthWidth           = 0;
    depthHeight          = 0;
    depthSize            = 0;
    colorResolution      = color_resolution(mode);
    imageFormat          = image_format(mode);
    depthMode            = depth_mode(mode);

    if(colorResolution != ColorResolution::OFF){

        // retrieve colors dimensions
        const auto colorDims     = k4a::GetColorDimensions(static_cast<k4a_color_resolution_t>(colorResolution));
        colorWidth  = std::get<0>(colorDims);
        colorHeight = std::get<1>(colorDims);
        colorSize   = colorWidth*colorHeight;

        if(imageFormat == ImageFormat::YUY2){
            convertedColorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                colorWidth,
                colorHeight,
                static_cast<int32_t>(colorWidth * 4 * sizeof(uint8_t))
            );
        }

        // set color indices
        indicesColors1D.resize(colorSize);
        std::iota(std::begin(indicesColors1D), std::end(indicesColors1D), 0);
    }


    if(depthMode != DepthMode::OFF){

        // retrieve depth dimensions
        auto depthRes = depth_resolution(mode);
        depthWidth  = depthRes.x();
        depthHeight  = depthRes.y();
        depthSize   = depthWidth*depthHeight;

        Logger::message(std::format("depthframe {} {} \n", depthWidth, depthHeight));

        // init resized color image
        if(colorResolution != ColorResolution::OFF){
            depthSizedColorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                depthWidth,
                depthHeight,
                static_cast<int32_t>(depthWidth * 4 * sizeof(uint8_t))
            );
        }

        // set depth indices
        depthMask.resize(depthSize);
        indicesDepths1D.resize(depthSize);
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);
        indicesDepths3D.resize(depthSize);
        indicesDepths1DNoBorders.reserve((depthWidth-2)*(depthHeight-2));
        size_t id = 0;
        for(size_t ii = 0; ii < depthHeight; ++ii){
            for(size_t jj = 0; jj < depthWidth; ++jj){
                indicesDepths3D[id] = {id,ii,jj};
                if(ii > 0 && ii < depthHeight-1 && jj > 0 && jj < depthWidth-1 ){
                    indicesDepths1DNoBorders.push_back(id);
                }
                ++id;
            }
        }

        // init cloud image
        if(has_cloud(mode)){
            pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                depthWidth,
                depthHeight,
                static_cast<int32_t>(depthWidth * 3 * sizeof(int16_t))
            );
        }

        Logger::message(std::format("depth {} {} \n", depthWidth, depthHeight));
    }

    // init display frames
    displayFrames.resize(10);
    currentDisplayFramesId = 0;
    for(auto &frame : displayFrames){

        frame = std::make_shared<DisplayDataFrame>();

        if(depthSize > 0){
            frame->depthFrame.width  = depthWidth;
            frame->depthFrame.height = depthHeight;
            frame->depthFrame.pixels.resize(depthSize);

            if(has_infrared(mode)){
                frame->infraredFrame.width  = depthWidth;
                frame->infraredFrame.height = depthHeight;
                frame->infraredFrame.pixels.resize(depthSize);
            }

            if(has_cloud(mode)){
                frame->cloud.vertices.resize(depthSize);
                frame->cloud.colors.resize(depthSize);
                frame->cloud.validVerticesCount = 0;
            }
        }

        if(color_resolution(mode) != K4::ColorResolution::OFF){
            if(mode == K4::Mode::Cloud_1024x1024 || mode == K4::Mode::Full_frame_1024x1024){
                frame->colorFrame.width  = depthWidth;
                frame->colorFrame.height = depthHeight;
                frame->colorFrame.pixels.resize(depthSize);
            }else{

                frame->colorFrame.width  = colorWidth;
                frame->colorFrame.height = colorHeight;
                frame->colorFrame.pixels.resize(colorSize);
            }
        }
    }
}

void Kinect4::Impl::read_from_microphones(){

    lastFrameCount = 0;
    if(audioListener != nullptr){

        // process audio frame
        audioListener->ProcessFrames([&](k4a::K4AMicrophoneFrame *frame, const size_t frameCount) {

            // store last count
            lastFrameCount = frameCount;
            if(lastFrameCount == 0){
                return lastFrameCount;
            }

            // resize audio buffer
            if(audioFrames.size() < lastFrameCount){
                audioFrames.resize(lastFrameCount);
            }

            // copy data
            std::copy(frame, frame + lastFrameCount, audioFrames.begin());

            return lastFrameCount;
        });

        if (audioListener->GetStatus() != SoundIoErrorNone){
            Logger::error(std::format("Error while recording {}\n", soundio_strerror(audioListener->GetStatus())));
        }else if (audioListener->Overflowed()){
            Logger::warning(std::format("Warning: sound overflow detected!\n"));
            audioListener->ClearOverflowed();
        }
    }
}

void Kinect4::Impl::read_from_imu(){
    k4a_imu_sample_t sample;
    if(device.get_imu_sample(&sample, std::chrono::milliseconds(1))){
        imuSample.temperature = sample.temperature;
        const auto &dAcc = sample.acc_sample.xyz;
        imuSample.acc ={dAcc.x,dAcc.y,dAcc.z};
        imuSample.accTsMs = sample.acc_timestamp_usec;
        const auto &dGyr = sample.gyro_sample.xyz;
        imuSample.gyr = {dGyr.x,dGyr.y,dGyr.z};
        imuSample.gyrTsMs = sample.gyro_timestamp_usec;
    }
}

bool Kinect4::Impl::get_color_image(){
    if(colorResolution != ColorResolution::OFF){

        Bench::start("[Kinect4] Capture get_color_image");
        colorImage = capture->get_color_image();
        Bench::stop();

        if (!colorImage->is_valid()){
            Logger::error("Failed to get color image from capture\n");
            return false;
        }
    }
    return true;
}

bool Kinect4::Impl::get_depth_image(){

    if(depthMode != DepthMode::OFF){

        Bench::start("[Kinect4] Capture get_depth_image");
        depthImage = capture->get_depth_image();
        Bench::stop();

        if (!depthImage->is_valid()){
            Logger::error("Failed to get depth image from capture\n");
            return false;
        }
    }

    return true;
}

bool Kinect4::Impl::get_infra_image(Mode mode){

    if(has_infrared(mode)){

        Bench::start("[Kinect4] Capture get_ir_image");
        infraredImage = capture->get_ir_image();
        Bench::stop();

        if (!infraredImage->is_valid()){
            Logger::error("Failed to get infrared image from capture\n");
            return false;
        }
    }

    return true;
}

void Kinect4::Impl::convert_color_image(const Parameters &p){

    if(colorResolution == ColorResolution::OFF){
        return;
    }

    if(imageFormat == ImageFormat::YUY2 ){

        Bench::start("[Kinect4] YUY2 convert");

        auto colorsYuy2 = reinterpret_cast<geo::Pt4<std::uint8_t>*>(colorImage->get_buffer());
        auto colorsBGRA = reinterpret_cast<geo::Pt4<std::uint8_t>*>(convertedColorImage->get_buffer());

        for_each(std::execution::par_unseq, std::begin(indicesColors1D), std::end(indicesColors1D), [&](size_t oidC){

            auto idC = oidC;
            if(idC%2 == 0){

                auto &currentPixel = colorsBGRA[idC];
                idC = idC/2;

                const Pt4<uint8_t> &yuy2 = colorsYuy2[idC];

                // convert to rgb
                const int y0 = std::clamp(static_cast<int>(p.yFactor*yuy2.x()), 0, 255);
                auto ci = 298 * (y0 - 16);

                const int u0 = std::clamp(static_cast<int>(p.uFactor*yuy2.y()), 0, 255);
                const auto d = u0 - 128;

                const int v0 = std::clamp(static_cast<int>(p.vFactor*yuy2.w()), 0, 255);
                const auto e = v0 - 128;

                currentPixel = geo::Pt4<uint8_t>{
                    static_cast<uint8_t>(std::clamp((ci + (516 * d) + 128)              >> 8, 0, 255)), // blue
                    static_cast<uint8_t>(std::clamp((ci + (-100 * d) - (208 * e) + 128) >> 8, 0, 255)), // green
                    static_cast<uint8_t>(std::clamp((ci + (409 * e) + 128)              >> 8, 0, 255)), // red
                    255
                };

            }else{

                auto &currentPixel = colorsBGRA[idC];
                idC = (idC-1)/2;

                const Pt4<uint8_t> &color = colorsYuy2[idC];
                const int y0 = std::clamp(static_cast<int>(p.yFactor*color.x()), 0, 255);
                const int u0 = std::clamp(static_cast<int>(p.uFactor*color.y()), 0, 255);
                const int y1 = std::clamp(static_cast<int>(p.yFactor*color.z()), 0, 255);
                const int v0 = std::clamp(static_cast<int>(p.vFactor*color.w()), 0, 255);

                // convert to rgb
                auto c = y0 - 16;
                const auto d = u0 - 128;
                const auto e = v0 - 128;

                auto ci = 298 * c;
                const auto v1 = (516 * d) + 128;
                const auto v2 = (-100 * d) - (208 * e) + 128;
                const auto v3 = (409 * e) + 128;

                c = y1 - 16;
                ci = 298 * c;

                currentPixel = geo::Pt4<uint8_t>{
                    static_cast<uint8_t>(std::clamp((ci + v1) >> 8, 0, 255)), // blue
                    static_cast<uint8_t>(std::clamp((ci + v2) >> 8, 0, 255)), // green
                    static_cast<uint8_t>(std::clamp((ci + v3) >> 8, 0, 255)), // red
                    255
                };
            }
        });

        colorImage = convertedColorImage;

        Bench::stop();

    }else if(imageFormat == ImageFormat::NV12){
        // ... not managed
    }else if(imageFormat == ImageFormat::MJPEG){
        // ... not managed
    }else if(imageFormat == ImageFormat::BGRA32){
        // nothing to do
    }
}

void Kinect4::Impl::resize_color_to_fit_depth(){

    if(colorImage.has_value() && depthImage.has_value()){

        Bench::start("[Kinect4] Transformation color_image_to_depth_camera");
        transformation.color_image_to_depth_camera(
            depthImage.value(),
            colorImage.value(),
            &depthSizedColorImage.value());

        colorImage = depthSizedColorImage;
        Bench::stop();
    }
}



