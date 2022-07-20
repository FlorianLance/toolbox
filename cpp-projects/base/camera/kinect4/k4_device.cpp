
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


#include "k4_device.hpp"

// std
#include <thread>
#include <execution>

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
#include "utility/format.hpp"
// # geometry
#include "geometry/point4.hpp"
// # camera
#include "k4_frame_compressor.hpp"

using namespace tool;
using namespace tool::geo;
using namespace tool::camera;

auto nanoseconds_since_epoch(){
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
}

struct K4Device::Impl{

    K4Device *kinect4 = nullptr;

    // device
    uint32_t deviceCount = 0;
    std::string serialNumber;
    k4a::device device = nullptr;
    k4a::calibration calibration;    
    k4a::transformation transformation;    
    k4a_device_configuration_t k4aConfig;
    K4ConfigSettings config;
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
    K4ImuSample imuSample;

    // parameters&
    K4DataSettings data;
    K4Filters filters;

    // infos
    size_t idCapture    = 0;
    size_t colorWidth   = 0;
    size_t colorHeight  = 0;
    size_t colorSize    = 0;
    size_t depthWidth   = 0;
    size_t depthHeight  = 0;
    size_t depthSize    = 0;
    K4ColorResolution colorResolution;
    K4ImageFormat imageFormat;
    K4DepthMode depthMode;

    // times
    std::chrono::nanoseconds afterCaptureTS;

    // state
    bool camerasStarted = false;
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
    K4FullFrameCompressor ffc;
    K4CloudFrameCompressor cfc;

    // thread/lockers
    std::mutex parametersM; /**< mutex for reading parameters at beginning of a new frame in thread function */
    std::unique_ptr<std::thread> frameReaderT = nullptr;

    // functions
    void read_frames(K4Mode mode);
    // # init data
    void init_data(K4Mode mode);
    // # read data
    void read_from_microphones();
    void read_from_imu();
    // # get images
    bool get_color_image();
    bool get_depth_image();
    bool get_infra_image(K4Mode mode);
    // # processing
    void convert_color_image(const K4Filters &f);
    void resize_color_to_fit_depth();
    void filter_depth_image(const K4Filters &f, K4Mode mode);
    void filter_color_image(const K4Filters &f);
    void filter_infrared_image(const K4Filters &f);
    void generate_cloud(K4Mode mode);
    void compress_cloud_frame(const K4Filters &f, const K4DataSettings &d);
    void compress_full_frame(const K4Filters &f, const K4DataSettings &d, K4Mode mode);
    void display_frame(const K4DataSettings &d, K4Mode mode);
};


k4a_device_configuration_t K4Device::generate_config(const K4ConfigSettings &config){

    k4a_device_configuration_t ka4Config        = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    ka4Config.color_format                      = static_cast<k4a_image_format_t>(image_format(config.mode));
    ka4Config.color_resolution                  = static_cast<k4a_color_resolution_t>(color_resolution(config.mode));
    ka4Config.depth_mode                        = static_cast<k4a_depth_mode_t>(depth_mode(config.mode));
    ka4Config.camera_fps                        = static_cast<k4a_fps_t>(framerate(config.mode));

    ka4Config.synchronized_images_only = false;

    if(depth_mode(config.mode) == K4DepthMode::OFF){
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

k4a_device_configuration_t K4Device::generate_config(
    K4ImageFormat colFormat,
    K4ColorResolution colResolution,
    K4DepthMode depthMode,
    K4Framerate fps,
    bool synchronizeColorAndDepth,
    int delayBetweenColorAndDepthUsec,
    K4SynchronisationMode synchMode,
    int subordinateDelayUsec,
    bool disableLED){

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format                      = static_cast<k4a_image_format_t>(colFormat);
    config.color_resolution                  = static_cast<k4a_color_resolution_t>(colResolution);
    config.depth_mode                        = static_cast<k4a_depth_mode_t>(depthMode);
    config.camera_fps                        = static_cast<k4a_fps_t>(fps);

    if(depthMode == K4DepthMode::OFF){
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

K4Device::K4Device() : i(std::make_unique<Impl>()){

    i->kinect4 = this;
    i->deviceCount = k4a::device::get_installed_count();

    if(i->deviceCount == 0){
        Logger::error("[Kinect4] No K4A devices found\n");
    }else{
        Logger::message(std::format("[Kinect4] Devices found: {}\n", i->deviceCount));
    }

    const int audioInitStatus = k4a::K4AAudioManager::Instance().Initialize();
    if (audioInitStatus != SoundIoErrorNone){
        Logger::error("[Kinect4] Failed to initialize audio backend: {}\n", soundio_strerror(audioInitStatus));
    }else{
        size_t nbDevices = k4a::K4AAudioManager::Instance().get_devices_count();
        Logger::message(std::format("[Kinect4] Audio devices count: {}\n", nbDevices));

        for(size_t ii = 0; ii < nbDevices; ++ii){
            std::string deviceName = k4a::K4AAudioManager::Instance().get_device_name(ii);
            Logger::message(std::format(" - {}\n", deviceName));
            if (deviceName.find("Azure Kinect Microphone Array") != std::string::npos) {
                Logger::message(std::format("Found Azure kinect microphones array.\n"));
                i->microphone = k4a::K4AAudioManager::Instance().get_microphone_for_device(deviceName);
                if(i->microphone == nullptr){
                    Logger::error(std::format("[Kinect4] Cannot init microphone.\n"));
                    i->audioListener = nullptr;
                    return;
                }
                i->microphone->Start();
                if(i->microphone->IsStarted()){
                    i->audioListener = i->microphone->CreateListener();
                }else{
                    Logger::error(std::format("[Kinect4] Cannot start microphone.\n"));
                }
//                i->audioListener =  std::make_shared<k4a::K4AMicrophoneListener>(i->microphone);
                if(i->audioListener == nullptr){
                    Logger::error(std::format("[Kinect4] Cannot init audio listener.\n"));
                    return;
                }
                break;
            }
        }

    }
}

K4Device::~K4Device(){
    clean();
}

bool K4Device::open(){

    if(i->config.idDevice >= i->deviceCount){
        Logger::error("[Kinect4] Invalid device id\n");
        return false;
    }

    try {
        i->device = k4a::device::open(i->config.idDevice);
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

    i->serialNumber = i->device.get_serialnum();

    Logger::message("[Kinect4] Device opened:\n");
    Logger::message(std::format("  Serialnum: {}\n", i->serialNumber));
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

uint32_t K4Device::nb_devices() const noexcept {
    return i->deviceCount;
}

std::string K4Device::device_name() const{
    return i->serialNumber;
}

bool K4Device::is_opened() const{
    return i->device.is_valid();
}

bool K4Device::cameras_started() const{
    return i->camerasStarted;
}

bool K4Device::is_reading_frames() const{return i->readFramesFromCameras;}

K4Mode K4Device::mode() const{
    return i->config.mode;
}

K4CompressMode K4Device::compress_mode() const{
    return i->data.compressMode;
}

void K4Device::close(){

    if(i->microphone){
        if(i->microphone->IsStarted()){
            i->microphone->Stop();
        }
        if(i->audioListener){
            i->audioListener = nullptr;
        }
    }

    if(i->readFramesFromCameras){
        Logger::error("[Kinect4] Reading must be stopped before closing the device.\n");
        return;
    }
    i->device.close();
}


void K4Device::clean(){
    if(is_opened()){
        if(is_reading_frames()){
            stop_reading();
        }
        stop_cameras();
        close();
    }
}

uint32_t K4Device::current_device_opened() const{
    if(is_opened()){
        return i->config.idDevice;
    }
    Logger::error("[Kinect4] No device opened.\n");
    return 0;
}

bool K4Device::start_reading(){

    if(!is_opened()){
        Logger::error("[Kinect4] Device must be opened for reading.\n");
        return false;
    }

    if(i->frameReaderT == nullptr){
        i->frameReaderT = std::make_unique<std::thread>(&K4Device::Impl::read_frames, i.get(), i->config.mode);
        return true;
    }else{
        Logger::error("[Kinect4] Reading thread already started.\n");
    }
    return false;
}

void K4Device::stop_reading(){

    i->readFramesFromCameras = false;   
    if(i->frameReaderT != nullptr){
        if(i->frameReaderT->joinable()){
            i->frameReaderT->join();
        }
        i->frameReaderT = nullptr;
    }
}

void K4Device::set_data_settings(const K4DataSettings &dataS){
    i->parametersM.lock();
    i->data = dataS;
    i->parametersM.unlock();
}

void K4Device::set_filters(const K4Filters &filters){
    i->parametersM.lock();
    i->filters = filters;
    i->parametersM.unlock();
}

bool K4Device::start_cameras(const K4ConfigSettings &configS){
    return start_cameras(i->k4aConfig = generate_config(i->config = configS));
}

bool K4Device::start_cameras(const k4a_device_configuration_t &k4aConfig){

    // set config
    i->k4aConfig = k4aConfig;

    try {

        Logger::message("[Kinect4] Retrieve calibration\n");
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

    i->camerasStarted = true;
    return true;
}

void K4Device::stop_cameras(){
    i->device.stop_cameras();
    i->device.stop_imu();
    i->camerasStarted = false;
}

void K4Device::Impl::read_frames(K4Mode mode){

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
        const auto f = filters;
        const auto d = data;
        parametersM.unlock();

        // read data from device
        try {
            Bench::start("[Kinect4] Device get_capture");
            const int32_t timeoutMs = 300;
            bool success   = device.get_capture(capture.get(), std::chrono::milliseconds(timeoutMs));
            Bench::stop();

            if(!success){
                Logger::error("[Kinect4] get_capture timeout\n");
                continue;
            }

            // update capture timestamp
            afterCaptureTS = nanoseconds_since_epoch();

            if(d.captureAudio){
                read_from_microphones();
            }

            if(d.captureIMU){
                read_from_imu();
            }

            temperature = capture->get_temperature_c();

        }   catch (std::runtime_error error) {
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
        convert_color_image(f);
        resize_color_to_fit_depth();
        filter_depth_image(f, mode);
        filter_color_image(f);
        filter_infrared_image(f);
        generate_cloud(mode);

        // send frames
        if(d.compressMode == K4CompressMode::Cloud){
            compress_cloud_frame(f,d);
        }

        if(d.compressMode == K4CompressMode::Full){
            compress_full_frame(f, d, mode);
        }

        if(d.generateRGBDisplayFrame || d.generateDepthDisplayFrame || d.generateInfraDisplayFrame || d.generateCloudDisplay){
            display_frame(d, mode);
        }

        idCapture++;
    }
}

void K4Device::Impl::filter_depth_image(const K4Filters &f, K4Mode mode){

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
    auto minD = f.minDepthValue < dRange.x() ? static_cast<std::int16_t>(dRange.x()) : f.minDepthValue;
    auto maxD = f.maxDepthValue > dRange.y() ? static_cast<std::int16_t>(dRange.y()) : f.maxDepthValue;

    for_each(std::execution::par_unseq, std::begin(indicesDepths3D), std::end(indicesDepths3D), [&](const Pt3<size_t> &dIndex){

        size_t id = dIndex.x();
        size_t ii = dIndex.y();
        size_t jj = dIndex.z();

        depthMask[id] = false;
        if(depthBuffer[id] == k4_invalid_depth_value){
            return;
        }

        // depth filtering
        if(ii < f.minHeight){
            return;
        }else if(ii > f.maxHeight){
            return;
        }

        if(jj < f.minWidth){
            return;
        }else if(jj > f.maxWidth){
            return;
        }

        if(depthBuffer[id] < minD){
            return;
        }else if(depthBuffer[id] > maxD){
            return;
        }

        // color filtering
        if(colorImage.has_value() && f.filterDepthWithColor){
            auto delta = norm(colorBuffer[id].xyz().conv<int>()-f.filterColor.conv<int>());
            if(delta > f.maxDiffColor.x()){
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
    const auto res = depth_resolution(mode);
    const size_t widthPlusOne  = res.x() +1;
    const size_t widthMinusOne = res.x() -1;

    if(doLocalDiffFiltering){
        const float mLocal =  f.maxLocalDiff;
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

    // count valid depth values
    validDepthValues = 0;
    for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){
        if(!depthMask[id]){
            depthBuffer[id] = k4_invalid_depth_value;
        }else{
            validDepthValues++;
        }
        // increment counter
//        validDepthValues.fetch_add(1, std::memory_order_relaxed);
    });

    Bench::stop();
}

void K4Device::Impl::filter_color_image(const K4Filters &f){

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
        if(f.invalidateColorFromDepth){
            if(depthBuffer[id] == k4_invalid_depth_value){
                colorBuffer[id] = k4_invalid_color_value;
            }
        }
    });

    Bench::stop();
}

void K4Device::Impl::filter_infrared_image(const K4Filters &f){

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
        if(f.invalidateInfraFromDepth){
            if(depthBuffer[id] == k4_invalid_depth_value){
                infraredBuffer[id] = k4_invalid_infra_value;
            }
        }
    });

    Bench::stop();
}

void K4Device::Impl::generate_cloud(K4Mode mode){

    if(has_cloud(mode) && pointCloudImage.has_value()){
        Bench::start("[Kinect4] Transformation depth_image_to_point_cloud");
        transformation.depth_image_to_point_cloud(depthImage.value(), K4A_CALIBRATION_TYPE_DEPTH, &pointCloudImage.value());
        Bench::stop();
    }
}

void K4Device::Impl::compress_cloud_frame(const K4Filters &f, const K4DataSettings &d){

    if(!colorImage.has_value() || !depthImage.has_value() || !pointCloudImage.has_value()){
        return;
    }

    tool::Bench::start("[Kinect4] Generate compressed cloud frame");
    auto compressedCloudFrame = cfc.compress(
        validDepthValues,
        f.jpegCompressionRate,
        colorImage.value(),
        depthImage.value(),
        pointCloudImage.value(),
        reinterpret_cast<float*>(audioFrames.data()), 7*lastFrameCount);

    compressedCloudFrame->idCapture      = static_cast<std::int32_t>(idCapture);
    compressedCloudFrame->afterCaptureTS = afterCaptureTS.count();


    if(compressedCloudFrame != nullptr){

        // add imu
        if(d.captureIMU){
            compressedCloudFrame->imuSample   = imuSample;
        }

        // add audio frames
        if(d.captureAudio && lastFrameCount > 0){
            compressedCloudFrame->audioFrames.resize(lastFrameCount);
            auto ad1 = reinterpret_cast<float*>(audioFrames.data());
            auto ad2 = reinterpret_cast<float*>(compressedCloudFrame->audioFrames.data());
            std::copy(ad1, ad1 + 7*lastFrameCount, ad2);
        }
    }
    tool::Bench::stop();

    // send
    if(compressedCloudFrame != nullptr){
        kinect4->new_compressed_cloud_frame_signal(std::shared_ptr<tool::camera::K4CompressedCloudFrame>(std::move(compressedCloudFrame)));
    }
}

void K4Device::Impl::compress_full_frame(const K4Filters &f, const K4DataSettings &d, K4Mode mode){

    if(!colorImage.has_value() || !depthImage.has_value() || !infraredImage.has_value()){
        return;
    }

    tool::Bench::start("[Kinect4] Generate compressed full frame");

    auto compressedFullFrame = ffc.compress(
        validDepthValues,
        f.jpegCompressionRate,
        colorImage, depthImage, infraredImage);

    compressedFullFrame->idCapture     = static_cast<std::int32_t>(idCapture);
    compressedFullFrame->afterCaptureTS = afterCaptureTS.count();

    if(compressedFullFrame != nullptr){

        // add infos
        compressedFullFrame->mode        = mode;
        compressedFullFrame->calibration = calibration;

        // add imu
        if(d.captureIMU){
            compressedFullFrame->imuSample   = imuSample;
        }

        // add audio frames                
        if(d.captureAudio && lastFrameCount > 0){
            compressedFullFrame->audioFrames.resize(lastFrameCount);
            auto ad1 = reinterpret_cast<float*>(audioFrames.data());
            auto ad2 = reinterpret_cast<float*>(compressedFullFrame->audioFrames.data());
            std::copy(ad1, ad1 + 7*lastFrameCount, ad2);
        }
    }
    tool::Bench::stop();

    // send
    if(compressedFullFrame != nullptr){
        kinect4->new_compressed_full_frame_signal(std::shared_ptr<K4CompressedFullFrame>(std::move(compressedFullFrame)));
    }
}

void K4Device::Impl::display_frame(const K4DataSettings &d, K4Mode mode){

    // write frame
    tool::Bench::start("[Kinect4] Write display data frame");

    auto currDisplayFrame = std::make_unique<K4DisplayFrame>();
    currDisplayFrame->idCapture      = static_cast<std::int32_t>(idCapture);
    currDisplayFrame->afterCaptureTS = afterCaptureTS.count();

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
    if(d.captureAudio && lastFrameCount != 0){

        tool::Bench::start("[Kinect4] display_frame::audio");

        // copy audio frames
        dFrame->audioFrames.resize(lastFrameCount);
        auto ad1 = reinterpret_cast<float*>(audioFrames.data());
        auto ad2 = reinterpret_cast<float*>(dFrame->audioFrames.data());
        std::copy(ad1, ad1 + 7*lastFrameCount, ad2);

        tool::Bench::stop();
    }

    // send color frame
    if(d.generateRGBDisplayFrame && colorImage.has_value()){

        tool::Bench::start("[Kinect4] display_frame::color");

        size_t width, height;
        if(depthImage.has_value()){
            width  = depthImage->get_width_pixels();
            height = depthImage->get_height_pixels();
        }else{
            width  = colorImage->get_width_pixels();
            height = colorImage->get_height_pixels();
        }
        dFrame->colorFrame.width  = width;
        dFrame->colorFrame.height = height;
        dFrame->colorFrame.pixels.resize(width*height);

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
    if(d.generateDepthDisplayFrame && depthImage.has_value()){

        tool::Bench::start("[Kinect4] display_frame::depth");

        dFrame->depthFrame.width  = depthImage->get_width_pixels();
        dFrame->depthFrame.height = depthImage->get_height_pixels();
        dFrame->depthFrame.pixels.resize(dFrame->depthFrame.width*dFrame->depthFrame.height);

        auto depthBuffer  = reinterpret_cast<const uint16_t*>(depthImage->get_buffer());
        const auto dRange = range(mode)*1000.f;
        const auto diff   = dRange.y() - dRange.x();

        for_each(std::execution::par_unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            if(depthBuffer[id] == k4_invalid_depth_value){
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
    if(d.generateInfraDisplayFrame && infraredImage.has_value()){

        tool::Bench::start("[Kinect4] display_frame::infrared");

        dFrame->infraredFrame.width  = infraredImage->get_width_pixels();;
        dFrame->infraredFrame.height = infraredImage->get_height_pixels();;
        dFrame->infraredFrame.pixels.resize(currDisplayFrame->infraredFrame.width*currDisplayFrame->infraredFrame.height);

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

    if(d.generateCloudDisplay && pointCloudImage.has_value() && colorImage.has_value() && depthImage.has_value()){

        tool::Bench::start("[Kinect4] display_frame::cloud");

        auto size = depthImage->get_width_pixels() * depthImage->get_height_pixels();
        dFrame->cloud.vertices.resize(size);
        dFrame->cloud.colors.resize(size);
        dFrame->cloud.validVerticesCount = 0;

        auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(pointCloudImage->get_buffer());
        auto colorBuffer = reinterpret_cast<const geo::Pt4<uint8_t>*>(colorImage->get_buffer());
        auto depthBuffer = reinterpret_cast<const uint16_t*>(depthImage->get_buffer());

        dFrame->cloud.validVerticesCount = validDepthValues;

        size_t idV = 0;
        for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            if(depthBuffer[id] == k4_invalid_depth_value){
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

        tool::Bench::stop();
    }

    // copy imu sample
    dFrame->imuSample = imuSample;

    tool::Bench::stop();

    kinect4->new_display_frame_signal(std::shared_ptr<K4DisplayFrame>(std::move(currDisplayFrame)));
}


void K4Device::Impl::init_data(K4Mode mode){

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
    idCapture            = 0;
    colorWidth           = 0;
    colorHeight          = 0;
    colorSize            = 0;
    depthWidth           = 0;
    depthHeight          = 0;
    depthSize            = 0;
    colorResolution      = color_resolution(mode);
    imageFormat          = image_format(mode);
    depthMode            = depth_mode(mode);

    if(colorResolution != K4ColorResolution::OFF){

        // retrieve colors dimensions
        const auto colorDims     = k4a::GetColorDimensions(static_cast<k4a_color_resolution_t>(colorResolution));
        colorWidth  = std::get<0>(colorDims);
        colorHeight = std::get<1>(colorDims);
        colorSize   = colorWidth*colorHeight;

        if(imageFormat == K4ImageFormat::YUY2){
            convertedColorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                static_cast<int>(colorWidth),
                static_cast<int>(colorHeight),
                static_cast<int32_t>(colorWidth * 4 * sizeof(uint8_t))
            );
        }

        // set color indices
        indicesColors1D.resize(colorSize);
        std::iota(std::begin(indicesColors1D), std::end(indicesColors1D), 0);
    }


    if(depthMode != K4DepthMode::OFF){

        // retrieve depth dimensions
        auto depthRes = depth_resolution(mode);
        depthWidth  = depthRes.x();
        depthHeight = depthRes.y();
        depthSize   = depthWidth*depthHeight;

        // init resized color image
        if(colorResolution != K4ColorResolution::OFF){
            depthSizedColorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                static_cast<int>(depthWidth),
                static_cast<int>(depthHeight),
                static_cast<int32_t>(depthWidth * 4 * sizeof(uint8_t))
            );
        }

        // set depth indices
        depthMask.resize(depthSize);
        indicesDepths1D.resize(depthSize);
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);
        indicesDepths3D.resize(depthSize);

        indicesDepths1DNoBorders.clear();
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
                static_cast<int>(depthWidth),
                static_cast<int>(depthHeight),
                static_cast<int32_t>(depthWidth * 3 * sizeof(int16_t))
            );
        }
    }
}

void K4Device::Impl::read_from_microphones(){

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
            Logger::error(std::format("[Kinect4] Error while recording {}\n", soundio_strerror(audioListener->GetStatus())));
        }else if (audioListener->Overflowed()){
            Logger::warning(std::format("[Kinect4] Warning: sound overflow detected!\n"));
            audioListener->ClearOverflowed();
        }
    }
}

void K4Device::Impl::read_from_imu(){
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

bool K4Device::Impl::get_color_image(){
    if(colorResolution != K4ColorResolution::OFF){

        Bench::start("[Kinect4] Capture get_color_image");
        colorImage = capture->get_color_image();
        Bench::stop();

        if (!colorImage->is_valid()){
            Logger::error("[Kinect4] Failed to get color image from capture\n");
            return false;
        }
    }
    return true;
}

bool K4Device::Impl::get_depth_image(){

    if(depthMode != K4DepthMode::OFF){

        Bench::start("[Kinect4] Capture get_depth_image");
        depthImage = capture->get_depth_image();
        Bench::stop();

        if (!depthImage->is_valid()){
            Logger::error("[Kinect4] Failed to get depth image from capture\n");
            return false;
        }
    }

    return true;
}

bool K4Device::Impl::get_infra_image(K4Mode mode){

    if(has_infrared(mode)){

        Bench::start("[Kinect4] Capture get_ir_image");
        infraredImage = capture->get_ir_image();
        Bench::stop();

        if (!infraredImage->is_valid()){
            Logger::error("[Kinect4] Failed to get infrared image from capture\n");
            return false;
        }
    }

    return true;
}

void K4Device::Impl::convert_color_image(const K4Filters &f){

    if(colorResolution == K4ColorResolution::OFF){
        return;
    }

    if(imageFormat == K4ImageFormat::YUY2 ){

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
                const int y0 = std::clamp(static_cast<int>(f.yFactor*yuy2.x()), 0, 255);
                auto ci = 298 * (y0 - 16);

                const int u0 = std::clamp(static_cast<int>(f.uFactor*yuy2.y()), 0, 255);
                const auto d = u0 - 128;

                const int v0 = std::clamp(static_cast<int>(f.vFactor*yuy2.w()), 0, 255);
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
                const int y0 = std::clamp(static_cast<int>(f.yFactor*color.x()), 0, 255);
                const int u0 = std::clamp(static_cast<int>(f.uFactor*color.y()), 0, 255);
                const int y1 = std::clamp(static_cast<int>(f.yFactor*color.z()), 0, 255);
                const int v0 = std::clamp(static_cast<int>(f.vFactor*color.w()), 0, 255);

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

    }else if(imageFormat == K4ImageFormat::NV12){
        // ... not managed
    }else if(imageFormat == K4ImageFormat::MJPEG){
        // ... not managed
    }else if(imageFormat == K4ImageFormat::BGRA32){
        // nothing to do
    }
}

void K4Device::Impl::resize_color_to_fit_depth(){

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



