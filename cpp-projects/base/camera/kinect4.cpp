

/*******************************************************************************
** exvr-scaner-base                                                           **
** No license (to be defined)                                                 **
** Copyright (c) [2018] [Florian Lance][EPFL-LNCO]                            **
********************************************************************************/

#include "kinect4.hpp"

// std
#include <thread>
#include <execution>
#include <format>

// turbojpg
#include <turbojpeg.h>

// kinect4
#include <k4a/k4a.hpp>

// local
// # kinect4
#include "k4a/k4astaticimageproperties.h"
// # utility
#include "utility/logger.hpp"
#include "utility/benchmark.hpp"
#include "utility/vector_utility.hpp"
// # geometry
#include "geometry/point4.hpp"
// # data
#include "data/integers_encoder.hpp"

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

    // parameters
    Parameters parameters;

    // state
    std::atomic_bool readFramesFromCameras = false;
    float temperature = 0.f;
    std::atomic<size_t> validDepthValues = 0;
    FrameReadingTimings times;

    // arrays indices
    std_v1<size_t> indicesDepths1D;
    std_v1<geo::Pt3<size_t>> indicesDepths3D;
    std_v1<size_t> indicesColors1D;

    // compression
    // # integer compressor
    IntegersEncoder depthCompressor;
    // # jpeg compressor
    tjhandle jpegCompressor = nullptr;
    unsigned char *tjCompressedImage = nullptr;

    // display frames
    size_t currentDisplayFramesId = 0;
    std::vector<std::shared_ptr<DisplayDataFrame>> displayFrames;

    // compressed frames
    size_t currentCompressedFrame = 0;
    std::vector<std::shared_ptr<CompressedDataFrame>> compressedFrames;

    // thread/lockers
    std::mutex parametersM; /**< mutex for reading parameters at beginning of a new frame in thread function */
    std::unique_ptr<std::thread> frameReaderT = nullptr;

    // functions
    std::shared_ptr<CompressedDataFrame> generate_compressed_data_frame(
        std::optional<k4a::image> colorImage,
        std::optional<k4a::image> depthImage,
        std::optional<k4a::image> infraredImage);

    void update_display_data_frame(DisplayDataFrame *d, const Parameters &p,
        std::optional<k4a::image> color,
        std::optional<k4a::image> depth,
        std::optional<k4a::image> infra,
        std::optional<k4a::image> cloud);

    void filter_depth_image(const K4::Parameters &p,
        k4a::image depthImage,
        std::optional<k4a::image> colorImage,
        std::optional<k4a::image> infraredImage);

    void filter_color_image(const K4::Parameters &p,
        k4a::image colorImage,
        std::optional<k4a::image> depthImage,
        std::optional<k4a::image> infraredImage){}

    void filter_infrared_image(const K4::Parameters &p,
        k4a::image infraredImage,
        std::optional<k4a::image> colorImage,
        std::optional<k4a::image> depthImage){}


    void read_frames(K4::Mode mode);
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
}

Kinect4::~Kinect4(){

    if(is_opened()){
        if(is_reading_frames()){
            stop_reading();
        }
        stop_cameras();
        close();
    }

    // clean jpeg compressor
    if(i->tjCompressedImage != nullptr){
        tjFree(i->tjCompressedImage);
    }
    tjDestroy(i->jpegCompressor);
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

    const int32_t timeoutMs = 300;

    if(!device.is_valid() || readFramesFromCameras){
        Logger::error("[Kinect4] Cannot start reading frames.\n");
        return;
    }

    // init compressor
    if(jpegCompressor == nullptr){
        jpegCompressor = tjInitCompress();
    }

    if(tjCompressedImage != nullptr){
        tjFree(tjCompressedImage);
        tjCompressedImage = nullptr;
    }

    // init capture
    k4a::capture capture;

    // init transform
    k4a::transformation transformation(calibration);

    // init images
    // # capture
    std::optional<k4a::image> colorImage      = std::nullopt;
    std::optional<k4a::image> depthImage      = std::nullopt;
    std::optional<k4a::image> infraredImage   = std::nullopt;
    std::optional<k4a::image> pointCloudImage = std::nullopt;
    // # processing
    std::optional<k4a::image> convertedColorImage = std::nullopt;
    std::optional<k4a::image> depthSizedColorImage = std::nullopt;


    size_t colorWidth  = 0;
    size_t colorHeight = 0;
    size_t colorSize   = 0;
    size_t depthWidth = 0;
    size_t depthHeight = 0;
    size_t depthSize = 0;

    const auto colorResolution = color_resolution(mode);
    const auto imageFormat  = image_format(mode);
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



        Logger::message(std::format("colorFrame {} {} \n", colorWidth, colorHeight));
    }

    const auto depthMode = depth_mode(mode);
    if(depthMode != DepthMode::OFF){

        // retrieve depth dimensions
        const auto depthDims     = k4a::GetDepthDimensions(static_cast<k4a_depth_mode_t>(depth_mode(mode)));
        depthWidth  = std::get<0>(depthDims);
        depthHeight = std::get<1>(depthDims);
        depthSize   = depthWidth*depthHeight;

        // init resized color image
        if(colorResolution != ColorResolution::OFF){
            depthSizedColorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                depthWidth,
                depthHeight,
                static_cast<int32_t>(depthWidth * 4 * sizeof(uint8_t))
            );                        
        }

        // set depth indices
        indicesDepths1D.resize(depthSize);
        std::iota(std::begin(indicesDepths1D), std::end(indicesDepths1D), 0);

        // set 3D depth indices
        indicesDepths3D.resize(depthSize);
        size_t id = 0;
        for(size_t ii = 0; ii < depthWidth; ++ii){
            for(size_t jj = 0; jj < depthHeight; ++jj){
                indicesDepths3D[id] = {id,ii,jj};
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

    // start loop
    readFramesFromCameras = true;
    while(readFramesFromCameras){

        //tool::Bench::reset();

        parametersM.lock();
        const auto p = parameters;
        parametersM.unlock();

        // get a capture
        times.startFrameReadingTS = nanoseconds_since_epoch();
        try {
            Bench::start("[Kinect4] Device get_capture");
            bool success = device.get_capture(&capture, std::chrono::milliseconds(timeoutMs));
            Bench::stop();

            if(!success){
                Logger::error("[Kinect4] get_capture timeout\n");
                continue;
            }

        }  catch (std::runtime_error error) {
            Logger::error(std::format("[Kinect4] get_capture error: {}\n", error.what()));
            readFramesFromCameras = false;
            break;
        }
        times.afterCaptureTS = nanoseconds_since_epoch();


        // get a color image
        if(colorResolution != ColorResolution::OFF){

            Bench::start("[Kinect4] Capture get_color_image");
            colorImage = capture.get_color_image();
            Bench::stop();

            if (!colorImage->is_valid()){
                 Logger::error("Failed to get color image from capture\n");
                continue;
            }            
        }
        times.getColorTS = nanoseconds_since_epoch();//colorImage->get_system_timestamp();

        // get a depth image
        if(depthMode != DepthMode::OFF){

            Bench::start("[Kinect4] Capture get_depth_image");
            depthImage = capture.get_depth_image();
            Bench::stop();

            if (!depthImage->is_valid()){
                Logger::error("Failed to get depth image from capture\n");
                continue;
            }            
        }
        times.getDepthTS = nanoseconds_since_epoch();//depthImage->get_system_timestamp();

        // get an infrared image
        if(has_infrared(mode)){

            Bench::start("[Kinect4] Capture get_ir_image");
            infraredImage = capture.get_ir_image();
            Bench::stop();

            if (!infraredImage->is_valid()){
                Logger::error("Failed to get infrared image from capture\n");
                continue;
            }            
        }
        times.getInfraTS = nanoseconds_since_epoch();//infraredImage->get_system_timestamp();

        // get temperature
        temperature = capture.get_temperature_c();

        // convert color image format
        if(colorResolution != ColorResolution::OFF){
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
        times.convertColorTS = nanoseconds_since_epoch();

        // resize color image to fit depth
        if(colorImage.has_value() && depthImage.has_value()){

            Bench::start("[Kinect4] Transformation color_image_to_depth_camera");
            transformation.color_image_to_depth_camera(
                depthImage.value(),
                colorImage.value(),
                &depthSizedColorImage.value());

            colorImage = depthSizedColorImage;
            Bench::stop();            
        }
        times.resizeColorTS = nanoseconds_since_epoch();

        // filter depth image
        if(depthImage.has_value()){
            Bench::start("[Kinect4] Filter depth");
            filter_depth_image(p, depthImage.value(), colorImage, infraredImage);
            Bench::stop();            
        }
        times.depthFilteringTS = nanoseconds_since_epoch();

        // generate cloud from depth image
        if(has_cloud(mode) && pointCloudImage.has_value()){
            Bench::start("[Kinect4] Transformation depth_image_to_point_cloud");
            transformation.depth_image_to_point_cloud(depthImage.value(), K4A_CALIBRATION_TYPE_DEPTH, &pointCloudImage.value());
            Bench::stop();            
        }
        times.cloudGenerationTS = nanoseconds_since_epoch();


        // compressed frame
        if(p.sendCompressedDataFrame && (colorImage.has_value() || depthImage.has_value() || infraredImage.has_value())){

            // compress frame
            tool::Bench::start("[Kinect4] Generate compressed data frame");
            auto compressedFrame = generate_compressed_data_frame(colorImage, depthImage, infraredImage);
            tool::Bench::stop();

            // send frame
            if(compressedFrame != nullptr){
                kinect4->new_compressed_data_frame_signal(compressedFrame);
            }            
        }
        times.compressFrameTS = nanoseconds_since_epoch();

        // display frame        
        if(p.sendDisplayColorFrame || p.sendDisplayDepthFrame || p.sendDisplayInfraredFrame || p.sendDisplayCloud){

            // write frame
            tool::Bench::start("[Kinect4] Write display data frame");
            auto currDisplayFrame = displayFrames[currentDisplayFramesId++];
            currentDisplayFramesId = currentDisplayFramesId%displayFrames.size();
            update_display_data_frame(currDisplayFrame.get(), p, colorImage, depthImage, infraredImage, pointCloudImage);
            tool::Bench::stop();

            kinect4->new_display_frame_signal(currDisplayFrame);
        }
        times.displayFrameTS    = nanoseconds_since_epoch();
        times.endFrameReadingTS = nanoseconds_since_epoch();

        kinect4->new_times_signal(times);
        //tool::Bench::display(BenchUnit::milliseconds, 0, true);
    }
    readFramesFromCameras = false;

    //tool::Bench::display(BenchUnit::milliseconds, 0, true);
}

void Kinect4::Impl::filter_depth_image(const Parameters &p,
    k4a::image depthImage, std::optional<k4a::image> colorImage, std::optional<k4a::image> infraredImage){

    // retrieve buffers
    auto depthBuffer = reinterpret_cast<int16_t*>(depthImage.get_buffer());
    geo::Pt4<uint8_t>* colorBuffer = colorImage.has_value() ? reinterpret_cast<geo::Pt4<uint8_t>*>(colorImage.value().get_buffer()) : nullptr;
    uint16_t* infraredBuffer = infraredImage.has_value() ? reinterpret_cast<uint16_t*>(infraredImage.value().get_buffer()) : nullptr;

    validDepthValues = 0;
    for_each(std::execution::par_unseq, std::begin(indicesDepths3D), std::end(indicesDepths3D), [&](const Pt3<size_t> &dIndex){

        const size_t id = dIndex.x();
        const size_t ii = dIndex.y();
        const size_t jj = dIndex.z();

        if(depthBuffer[id] == invalid_depth_value){
            return;
        }

        // depth filtering
        if(ii < p.minWidth){
            depthBuffer[id] = invalid_depth_value;
            return;
        }else if(ii > p.maxWidth){
            depthBuffer[id] = invalid_depth_value;
            return;
        }

        if(jj < p.minHeight){
            depthBuffer[id] = invalid_depth_value;
            return;
        }else if(jj > p.maxHeight){
            depthBuffer[id] = invalid_depth_value;
            return;
        }

        if(depthBuffer[id] < p.minDepthValue){
            depthBuffer[id] = invalid_depth_value;
            return;
        }else if(depthBuffer[id] > p.maxDepthValue){
            depthBuffer[id] = invalid_depth_value;
            return;
        }

        // color filtering
        if(colorImage.has_value() && p.filterDepthWithColor){

            auto delta = sqrt(
                (colorBuffer->x()-p.filterColor.x())*(colorBuffer->x()-p.filterColor.x()) +
                (colorBuffer->y()-p.filterColor.y())*(colorBuffer->y()-p.filterColor.y()) +
                (colorBuffer->z()-p.filterColor.z())*(colorBuffer->z()-p.filterColor.z())
            );
            if(delta > p.maxDiffColor.x()){
                depthBuffer[id] = invalid_depth_value;
                return;
            }

            if(colorBuffer->x() == 0 && colorBuffer->y() == 0 && colorBuffer->z() == 0 && colorBuffer->w() == 0){
                depthBuffer[id] = invalid_depth_value;
                return;
            }
        }

        // infrared filtering
        // ...

        // increment counter
        validDepthValues.fetch_add(1, std::memory_order_relaxed);
    });
}

std::shared_ptr<CompressedDataFrame> Kinect4::Impl::generate_compressed_data_frame(
    std::optional<k4a::image> colorImage, std::optional<k4a::image> depthImage, std::optional<k4a::image> infraredImage){

    auto frame = std::make_shared<CompressedDataFrame>();
    frame->calibration = calibration;

    if(colorImage.has_value()){

        // init buffer
        frame->colorWidth  = colorImage->get_width_pixels();
        frame->colorHeight = colorImage->get_height_pixels();

        const int jpegQuality = parameters.jpegCompressionRate;
        // Logger::message(std::format("jpeg compression {} {} {}\n", frame->colorWidth, frame->colorHeight, colorImage->get_size()));

        long unsigned int jpegColorSize = 0;

        if(tjCompressedImage == nullptr){
            tjCompressedImage = tjAlloc(frame->colorHeight*frame->colorWidth*4);
            // Logger::message(std::format("alloc {}\n", frame->colorHeight*frame->colorWidth*4));
        }

        int ret = tjCompress2(jpegCompressor,
            reinterpret_cast<const unsigned char*>(colorImage->get_buffer()), frame->colorWidth, 0, frame->colorHeight,
            TJPF_BGRA,
            &tjCompressedImage, &jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);

        if(ret == -1){
            Logger::error(std::format("[Kinect4] tjCompress2 error with code: {}\n", tjGetErrorStr2(jpegCompressor)));
            return nullptr;
        }

        frame->compressedColorBuffer.resize(jpegColorSize);
        std::copy(tjCompressedImage, tjCompressedImage + jpegColorSize, std::begin(frame->compressedColorBuffer));

    }else{
        frame->colorWidth  = 0;
        frame->colorHeight = 0;
        frame->compressedColorBuffer = {};
    }


    if(depthImage.has_value()){

        // init buffer
        frame->depthWidth  = depthImage->get_width_pixels();
        frame->depthHeight = depthImage->get_height_pixels();
        auto depthSize = frame->depthWidth*frame->depthHeight/2;
        frame->compressedDepthBuffer.resize(depthSize + 1024, 0);

        // fill buffer
        std::fill(std::begin(frame->compressedDepthBuffer), std::end(frame->compressedDepthBuffer), 0);

        // encode buffer
        size_t sizeDepthCompressed = depthCompressor.encode(
            reinterpret_cast<uint32_t*>(depthImage->get_buffer()), depthSize,
            frame->compressedDepthBuffer.data(),                   depthSize + 1024
        );

        if(sizeDepthCompressed == 0){
            Logger::error("[Kinect4] depthCompressor error\n");
            return nullptr;
        }
        if(frame->compressedDepthBuffer.size() != sizeDepthCompressed){
            frame->compressedDepthBuffer.resize(sizeDepthCompressed);
        }
        // Logger::message(std::format("size compressed {} {}\n", depthSize, sizeDepthCompressed));

    }

    if(infraredImage.has_value()){
        // ...
    }

    return frame;
}



void Kinect4::Impl::update_display_data_frame(DisplayDataFrame *d, const Parameters &p,
    std::optional<k4a::image> color,
    std::optional<k4a::image> depth,
    std::optional<k4a::image> infra,
    std::optional<k4a::image> cloud){

    // init depth frame
    float min=0.f,max =0.f,diff = 0.f;
    const std_v1<Pt3f> depthGradient ={
        {0.f,0.f,1.f},
        {0.f,1.f,1.f},
        {0.f,1.f,0.f},
        {1.f,1.f,0.f},
        {1.f,0.f,0.f},
    };

    const std_v1<Pt3f> infraGradient ={
        {0.f,0.f,1.f},
        {0.f,1.f,1.f},
        {0.f,1.f,0.f},
        {1.f,1.f,0.f},
        {1.f,0.f,0.f},
    };

    // send color frame
    if(p.sendDisplayColorFrame && color.has_value()){

        tool::Bench::start("[Kinect4] sendDisplayColorFrame");

        size_t width, height;
        if(depth.has_value()){
            width = depth->get_width_pixels();
            height = depth->get_height_pixels();
        }else{
            width = color->get_width_pixels();
            height = color->get_height_pixels();
        }

        d->colorFrame.width  = width;
        d->colorFrame.height = height;

        auto colorBuffer = reinterpret_cast<const geo::Pt4<uint8_t>*>(color->get_buffer());
        std::vector<size_t> *ids;
        if(depth.has_value()){
            ids = &indicesDepths1D;
        }else{
            ids = &indicesColors1D;
        }

        for_each(std::execution::par_unseq, std::begin(*ids), std::end(*ids), [&](size_t id){
            d->colorFrame.pixels[id] = {
                colorBuffer[id].z(),
                colorBuffer[id].y(),
                colorBuffer[id].x()
            };
        });

        tool::Bench::stop();
    }

    // send depth frame
    if(p.sendDisplayDepthFrame && depth.has_value()){

        tool::Bench::start("[Kinect4] sendDisplayDepthFrame");

        const size_t width = depth->get_width_pixels();
        const size_t height = depth->get_height_pixels();
        const size_t size = width * height;
        d->depthFrame.width  = width;
        d->depthFrame.height = height;
//        frame->depthFrame.pixels.resize(size);

        auto depthBuffer = reinterpret_cast<const uint16_t*>(depth->get_buffer());

        // find min/max
        const auto [pmin, pmax] = std::minmax_element(depthBuffer, depthBuffer + size);
        min = static_cast<float>(*pmin);
        max = static_cast<float>(*pmax);
        diff = max-min;

        for_each(std::execution::par_unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            float vF = (static_cast<float>(depthBuffer[id]) - min)/diff;
            float intPart;
            float decPart = std::modf((vF*(depthGradient.size()-1)), &intPart);
            size_t idG = static_cast<size_t>(intPart);

            auto col = depthGradient[idG]*(1.f-decPart) + depthGradient[idG+1]*decPart;
            d->depthFrame.pixels[id] = {
                static_cast<std::uint8_t>(255*col.x()),
                static_cast<std::uint8_t>(255*col.y()),
                static_cast<std::uint8_t>(255*col.z())
            };
        });

        tool::Bench::stop();
    }

    // send infrared frame
    if(p.sendDisplayInfraredFrame && infra.has_value()){

        tool::Bench::start("[Kinect4] sendDisplayInfraredFrame");

        const size_t width = infra->get_width_pixels();
        const size_t height = infra->get_height_pixels();
        const size_t size = width * height;
        d->infraredFrame.width  = width;
        d->infraredFrame.height = height;
        //frame->infraredFrame.pixels.resize(size);

        auto infraBuffer = reinterpret_cast<const uint16_t*>(infra->get_buffer());

        // find min/max
        const auto [pmin, pmax] = std::minmax_element(infraBuffer, infraBuffer + size);
        min = static_cast<float>(*pmin);
        max = static_cast<float>(*pmax);
        diff = max-min;

        for_each(std::execution::par_unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            float vF = static_cast<float>(infraBuffer[id] - min)/diff;
            float intPart;
            float decPart = std::modf((vF*(infraGradient.size()-1)), &intPart);
            size_t idG = static_cast<size_t>(intPart);

            auto col = infraGradient[idG]*(1.f-decPart) + infraGradient[idG+1]*decPart;
            d->infraredFrame.pixels[id] = {
                static_cast<std::uint8_t>(255*col.x()),
                static_cast<std::uint8_t>(255*col.y()),
                static_cast<std::uint8_t>(255*col.z())
            };
        });

        tool::Bench::stop();
    }

    if(p.sendDisplayCloud && cloud.has_value() && color.has_value() && depth.has_value()){

        auto cloudBuffer = reinterpret_cast<geo::Pt3<int16_t>*>(cloud->get_buffer());
        auto colorBuffer = reinterpret_cast<const geo::Pt4<uint8_t>*>(color->get_buffer());
        auto depthBuffer = reinterpret_cast<const uint16_t*>(depth->get_buffer());

        //frame->cloud.vertices.resize(validDepthValues);
        //frame->cloud.colors.resize(validDepthValues);
        d->cloud.validVerticesCount = validDepthValues;

        size_t idV = 0;
        for_each(std::execution::unseq, std::begin(indicesDepths1D), std::end(indicesDepths1D), [&](size_t id){

            if(depthBuffer[id] == invalid_depth_value){
                return;
            }

            d->cloud.vertices[idV]= geo::Pt3f{
                static_cast<float>(-cloudBuffer[id].x()),
                static_cast<float>(-cloudBuffer[id].y()),
                static_cast<float>( cloudBuffer[id].z())
            }*0.01f;
            d->cloud.colors[idV] = geo::Pt3f{
                static_cast<float>(colorBuffer[id].z()),
                static_cast<float>(colorBuffer[id].y()),
                static_cast<float>(colorBuffer[id].x())
            }/255.f;

            ++idV;
        });
    }
}



//void Kinect4::process_display_data(){

////    auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();

//    // transforms images
//    i->transformation.depth_image_to_color_camera(i->depthImage, &i->resizedDepthImage);

////    auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();

//    i->transformation.depth_image_to_point_cloud(i->resizedDepthImage, K4A_CALIBRATION_TYPE_COLOR, &i->pointCloudImage);

////    auto d3 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();

//    // retrieve buffers
//    auto pointCloudImageData      = reinterpret_cast<geo::Pt3<int16_t>*>(i->pointCloudImage.get_buffer());

//    // get current point cloud image size
//    i->pointCloudImageWidth    = i->pointCloudImage.get_width_pixels();
//    i->pointCloudImageHeight   = i->pointCloudImage.get_height_pixels();
//    i->pointCloudImageSize     = i->pointCloudImageWidth*i->pointCloudImageHeight;

//    i->validVerticesCount = 0;
//    for (size_t ii = 0; ii < i->pointCloudImageSize; ii++){
//        if(pointCloudImageData[ii].z() == 0){
//            i->colorCloudCorr[ii] = -1;
//        }else{
//            i->colorCloudCorr[ii] = i->validVerticesCount++;
//        }
//    }

////    auto d4 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();
////    std::cout << "d time: " << (d2 - d1) << " " << (d3 - d2) << " " << (d4 - d3) <<  "\n";
//}
//std::shared_ptr<K4ColoredCloud> Kinect4::Impl::generate_cloud(){

//    // retrieve buffers
//    auto *pointCloudImageData = reinterpret_cast<const geo::Pt3<int16_t>*>(pointCloudImage.get_buffer());
//    auto *colorImageData      = reinterpret_cast<const geo::Pt4<uint8_t>*>(uncompressedColorImage.get_buffer());

//    auto cloud = std::make_shared<K4ColoredCloud>();
//    cloud->vertices.reserve(validVerticesCount);
//    cloud->colors.reserve(validVerticesCount);

//    for(size_t ii = 0; ii < pointCloudImageSize; ++ii){

//        if(pointCloudImageData[ii].z() == 0){
//            continue;
//        }

//        cloud->vertices.emplace_back(geo::Pt3f{
//             static_cast<float>(-pointCloudImageData[ii].x()),
//             static_cast<float>(-pointCloudImageData[ii].y()),
//             static_cast<float>( pointCloudImageData[ii].z())
//         }*0.01f);

//        cloud->colors.emplace_back(geo::Pt3f{
//           static_cast<float>(colorImageData[ii].z()),
//           static_cast<float>(colorImageData[ii].y()),
//           static_cast<float>(colorImageData[ii].x())
//       }/255.f);
//    }
//    return cloud;
//}


//std::shared_ptr<K4PixelsFrame> Kinect4::Impl::generate_original_size_color_frame(){

//    // retrieve buffers
//    auto *colorImageData = reinterpret_cast<const uint8_t*>(uncompressedColorImage.get_buffer());

//    auto colorFrame = std::make_shared<K4PixelsFrame>();
//    colorFrame->width  = uncompressedColorImage.get_width_pixels();
//    colorFrame->height = uncompressedColorImage.get_height_pixels();
//    const auto size    = colorFrame->width*colorFrame->height;
//    colorFrame->pixels.resize(size);
//    for(size_t ii = 0; ii < size; ++ii){
//        colorFrame->pixels[ii] = {
//            (colorImageData[4 * ii + 2]/255.f),
//            (colorImageData[4 * ii + 1]/255.f),
//            (colorImageData[4 * ii + 0]/255.f)
//        };
//    }
//    return colorFrame;
//}


//std::shared_ptr<K4PixelsFrame> Kinect4::Impl::generate_depth_filtered_color_frame(){
//    return nullptr;
//}

//std::shared_ptr<K4PixelsFrame> Kinect4::Impl::generate_infrared_frame(){
//    return nullptr;
//}

//std::shared_ptr<K4PixelsFrame> Kinect4::Impl::generate_depth_frame(){

//    // retrieve buffers
//    auto buffer    = reinterpret_cast<const std::uint16_t*>(resizedDepthImage.get_buffer());

//    auto depthFrame = std::make_shared<K4PixelsFrame>();
//    depthFrame->width  = resizedDepthImage.get_width_pixels();
//    depthFrame->height = resizedDepthImage.get_height_pixels();

//    const auto size = depthFrame->width*depthFrame->height;
//    depthFrame->pixels.resize(size);
//    std::fill(std::begin(depthFrame->pixels), std::end(depthFrame->pixels), geo::Pt3f{0.f,0.f,0.f});

//    // find min/max
//    const auto [min, max] = std::minmax_element(buffer,buffer + size);
//    const auto diff = (*max-*min);

//    std_v1<Pt3f> depthGradient ={
//        {0.f,0.f,1.f},
//        {0.f,1.f,1.f},
//        {0.f,1.f,0.f},
//        {1.f,1.f,0.f},
//        {1.f,0.f,0.f},
//    };

//    for(size_t ii = 0; ii < size; ++ii){
//        if(buffer[ii] == 0){
//            continue;
//        }else{
//            float vF = static_cast<float>(buffer[ii] - (*min))/diff;
//            float intPart;
//            float decPart = std::modf((vF*(depthGradient.size()-1)), &intPart);
//            size_t id = static_cast<size_t>(intPart);
//            depthFrame->pixels[ii] = depthGradient[id]*(1.f-decPart) + depthGradient[id+1]*decPart;
//        }
//    }

//    return depthFrame;
//}


// not used

//void Kinect4::Impl::create_xy_table(const k4a::calibration &calibration, k4a::image &xyTable){

//    k4a_float2_t *tableData = reinterpret_cast<k4a_float2_t*>(xyTable.get_buffer());
//    const int width   = calibration.depth_camera_calibration.resolution_width;
//    const int height  = calibration.depth_camera_calibration.resolution_height;

//    k4a_float2_t point;
//    k4a_float3_t ray;
//    bool valid;

//    for (int y = 0, idx = 0; y < height; y++){

//        point.xy.y = static_cast<float>(y);
//        for (int x = 0; x < width; x++, idx++){
//            point.xy.x = static_cast<float>(x);

//            // Transform a 2D pixel coordinate with an associated depth value of the source camera into a 3D point
//            // of the target coordinate system.
//            valid = calibration.convert_2d_to_3d(
//                point,
//                1.f,
//                K4A_CALIBRATION_TYPE_DEPTH,
//                K4A_CALIBRATION_TYPE_DEPTH,
//                &ray
//                );

//            if (valid){
//                tableData[idx].xy.x = ray.xyz.x;
//                tableData[idx].xy.y = ray.xyz.y;
//            }else{
//                tableData[idx].xy.x = nanf("");
//                tableData[idx].xy.y = nanf("");
//            }
//        }
//    }
//}

//int Kinect4::Impl::generate_point_cloud(k4a::image &depthImage, k4a::image &xyTable, k4a::image &pointCloud){

//    const int width   = depthImage.get_width_pixels();
//    const int height  = depthImage.get_height_pixels();

//    uint16_t     *depthData      = reinterpret_cast<uint16_t*>(depthImage.get_buffer());
//    k4a_float2_t *xyTableData    = reinterpret_cast<k4a_float2_t*>(xyTable.get_buffer());
//    k4a_float3_t *pointCloudData = reinterpret_cast<k4a_float3_t*>(pointCloud.get_buffer());

//    int pointCount = 0;
//    for (int ii = 0; ii < width * height; ii++){
//        if (depthData[ii] != 0 && !isnan(xyTableData[ii].xy.x) && !isnan(xyTableData[ii].xy.y)){
//            pointCloudData[ii].xyz.x = xyTableData[ii].xy.x * (float)depthData[ii];
//            pointCloudData[ii].xyz.y = xyTableData[ii].xy.y * (float)depthData[ii];
//            pointCloudData[ii].xyz.z = static_cast<float>(depthData[ii]);
//            ++pointCount;
//        }else{
//            pointCloudData[ii].xyz.x = nanf("");
//            pointCloudData[ii].xyz.y = nanf("");
//            pointCloudData[ii].xyz.z = nanf("");
//        }
//    }
//    return pointCount;
//}



//bool Kinect4::Impl::write_point_cloud(const std::string &fileName, const k4a::image &pointCloud, int pointCount){

//    const int width   = pointCloud.get_width_pixels();
//    const int height  = pointCloud.get_height_pixels();
//    const k4a_float3_t *pointCloudData = reinterpret_cast<const k4a_float3_t*>(pointCloud.get_buffer());

//    // save to the ply file
//    std::ofstream ofs;
//    ofs.open(fileName);
//    if(!ofs.is_open()){
//        return false;
//    }

//    ofs << "ply\n";
//    ofs << "format ascii 1.0\n";
//    ofs << "element vertex " << pointCount << "\n";
//    ofs << "property float x\n";
//    ofs << "property float y\n";
//    ofs << "property float z\n";
//    ofs << "end_header\n";
//    ofs.close();

//    std::stringstream ss;
//    for (int ii = 0; ii < width * height; ii++){

//        if (isnan(pointCloudData[ii].xyz.x) || isnan(pointCloudData[ii].xyz.y) || isnan(pointCloudData[ii].xyz.z)){
//            continue;
//        }

//        ss << static_cast<float>(pointCloudData[ii].xyz.x) << " "
//           << static_cast<float>(pointCloudData[ii].xyz.y) << " "
//           << static_cast<float>(pointCloudData[ii].xyz.z) << "\n";
//    }

//    std::ofstream ofsText(fileName, std::ios::out | std::ios::app);
//    ofsText.write(ss.str().c_str(), (std::streamsize)ss.str().length());

//    return true;
//}

//bool Kinect4::Impl::tranformation_helpers_write_point_cloud(const k4a::image &pointCloudImage, const k4a::image &colorImage, const std::string &fileName){

//    std::vector<color_point_t> points;

//    const int width  = pointCloudImage.get_width_pixels();
//    const int height = pointCloudImage.get_height_pixels();

//    const int16_t *pointCloudImageData = reinterpret_cast<const int16_t*>(pointCloudImage.get_buffer());
//    const uint8_t *colorImageData      = reinterpret_cast<const uint8_t*>(colorImage.get_buffer());

//    for (int ii = 0; ii < width * height; ii++){

//        color_point_t point;
//        point.xyz[0] = pointCloudImageData[3 * ii + 0];
//        point.xyz[1] = pointCloudImageData[3 * ii + 1];
//        point.xyz[2] = pointCloudImageData[3 * ii + 2];
//        if (point.xyz[2] == 0){
//            continue;
//        }

//        point.rgb[0] = colorImageData[4 * ii + 0];
//        point.rgb[1] = colorImageData[4 * ii + 1];
//        point.rgb[2] = colorImageData[4 * ii + 2];
//        uint8_t alpha = colorImageData[4 * ii + 3];

//        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0){
//            continue;
//        }

//        points.push_back(point);
//    }

//    // save to the ply file
//    std::ofstream ofs;
//    ofs.open(fileName);
//    if(!ofs.is_open()){
//        return false;
//    }

//    ofs << "ply\n";
//    ofs << "format ascii 1.0\n";
//    ofs << "element vertex " << points.size() << "\n";
//    ofs << "property float x\n";
//    ofs << "property float y\n";
//    ofs << "property float z\n";
//    ofs << "property uchar red\n";
//    ofs << "property uchar green\n";
//    ofs << "property uchar blue\n";
//    ofs << "end_header\n";
//    ofs.close();

//    std::stringstream ss;
//    for (size_t ii = 0; ii < points.size(); ++ii){

//        // image data is BGR
//        ss << (float)points[ii].xyz[0] << " " << (float)points[ii].xyz[1] << " " << (float)points[ii].xyz[2];
//        ss << " " << (float)points[ii].rgb[2] << " " << (float)points[ii].rgb[1] << " " << (float)points[ii].rgb[0];
//        ss << std::endl;
//    }
//    std::ofstream ofs_text(fileName, std::ios::out | std::ios::app);
//    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());

//    return true;
//}

//bool Kinect4::Impl::point_cloud_depth_to_color(
//    const k4a::transformation &transformationHandle,
//    const k4a::image &depthImage,
//    const k4a::image &colorImage,
//    const std::string &fileName){

//    k4a::image transformedDepthImage = transformationHandle.depth_image_to_color_camera(depthImage);
//    k4a::image pointCloudImage       = transformationHandle.depth_image_to_point_cloud(
//        transformedDepthImage,
//        K4A_CALIBRATION_TYPE_COLOR
//    );
//    return tranformation_helpers_write_point_cloud(pointCloudImage, colorImage, fileName);
//}



//bool Kinect4::Impl::write_color_bgra_image(const std::string &pathImage, const k4a::image &colorImage){

//    std_v1<unsigned char> data;
//    data.resize(colorImage.get_size());

//    auto buffer = colorImage.get_buffer();
//    for(size_t ii = 0; ii < colorImage.get_size()/4; ++ii){
//        const size_t id = ii*4;
//        data[id+0] = buffer[id+2];
//        data[id+1] = buffer[id+1];
//        data[id+2] = buffer[id+0];
//        data[id+3] = buffer[id+3];
//    }

//    tool::graphics::Texture texColor;
//    texColor.copy_2d_data(
//        colorImage.get_width_pixels(),
//        colorImage.get_height_pixels(),
//        4,
//        data
//    );

//    return texColor.write_2d_image_file_data(pathImage);
//}

//bool Kinect4::Impl::write_depth_image(const std::string &pathImage, const k4a::image &depthImage){

//    const size_t w =depthImage.get_width_pixels();
//    const size_t h =depthImage.get_height_pixels();
//    auto buffer = reinterpret_cast<const std::int16_t*>(depthImage.get_buffer());

//    // find min/max
//    std::int16_t min = 10000;
//    std::int16_t max = -10000;
//    for(size_t ii = 0; ii < h*w; ++ii){
//        if(buffer[ii] == 0){
//            continue;
//        }
//        if(buffer[ii] < min){
//            min = buffer[ii];
//        }
//        if(buffer[ii] > max){
//            max = buffer[ii];
//        }
//    }

//    // cap max
//    if(max > 4000){
//        max = 4000;
//    }

//    // resize data
//    std_v1<unsigned char> dData;
//    dData.resize(3*w*h);
//    std::fill(std::begin(dData), std::end(dData), 0);

//    for(size_t ii = 0; ii < h*w; ++ii){
//        if(buffer[ii] == 0){
//            continue;
//        }else{
//            std::int16_t v = buffer[ii];
//            if(v > max){
//                v = max;
//            }
//            v = v-min;
//            float vF = 1.f*v/(max-min);
//            std::int8_t vI  = vF*255.f;
//            dData[ii*3+0] = vI;
//            dData[ii*3+1] = vI;
//            dData[ii*3+2] = vI;
//        }
//    }

//    tool::graphics::Texture texDetph;
//    texDetph.copy_2d_data(
//        depthImage.get_width_pixels(),
//        depthImage.get_height_pixels(),
//        3,
//        dData
//    );
//    return texDetph.write_2d_image_file_data(pathImage);
//}
