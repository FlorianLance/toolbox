

/*******************************************************************************
** exvr-scaner-base                                                           **
** No license (to be defined)                                                 **
** Copyright (c) [2018] [Florian Lance][EPFL-LNCO]                            **
********************************************************************************/

#include "kinect4.hpp"

// std
#include <fstream>
#include <sstream>
#include <thread>
#include <execution>
#include <limits>
#include <format>

// turbojpg
#include <turbojpeg.h>

// kinect4
#include <k4a/k4a.hpp>

// local
#include "geometry/matrix3.hpp"
#include "graphics/texture.hpp"
#include "utility/logger.hpp"
#include "utility/benchmark.hpp"
#include "data/integers_encoder.hpp"
#include "k4a/k4astaticimageproperties.h"


using namespace tool;
using namespace tool::geo;
using namespace tool::camera;
using namespace tool::camera::K4;

auto nanoseconds_since_epoch(){
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
}

struct Kinect4::Impl{

    std::shared_ptr<DisplayDataFrame> f1 = nullptr;
    std::shared_ptr<DisplayDataFrame> f2 = nullptr;

    // device
    uint32_t deviceCount = 0;
    k4a::device device = nullptr;
    k4a::calibration calibration;    
    k4a::transformation transformation;    
    k4a_device_configuration_t k4aConfig;
    K4::Config config;

    // parameters
    Mode mode;
    std::atomic_bool readFramesFromCameras = false;
    Parameters parameters; /**< kinect device parameters */

    // processing data
    // # timestamp
    FrameReadingTimings times;
    // # sizes
    size_t pointCloudImageWidth = 0;
    size_t pointCloudImageHeight = 0;
    size_t pointCloudImageSize = 0;
    size_t validVerticesCount = 0;
    std::atomic<size_t> validDepthValues = 0;
    // # indices
    std_v1<size_t> indicesDepths1D;
    std_v1<geo::Pt3<size_t>> indicesDepths3D;
    std_v1<size_t> indicesColors1D;
    std_v1<int> colorCloudCorr;
//    std::unordered_map<size_t, int> colorCloudCorr;
    // # images
    // ## color
    k4a::image colorImage; /**< captured color image */
    k4a::image uncompressedColorImage; /**< uncompressed captured color image */
    k4a::image resizedUncompressedColorImage; /**< uncompressed captured color image resized to fit captured "depthImage" size */
    // ## depth
    k4a::image depthImage;        /**< captured depth image */
    k4a::image resizedDepthImage; /**< captured depth image resized to fit captured "colorImage" size */
    // ## cloud
    k4a::image pointCloudImage;

    // integer compressor
    IntegersEncoder depthCompressor;

    // thread
    std::mutex parametersM; /**< mutex for reading parameters at beginning of a new frame in thread function */
    std::unique_ptr<std::thread> frameReaderT = nullptr;

    // generate functions
    std::shared_ptr<CompressedDataFrame> generate_compressed_data_frame();
//    std::shared_ptr<K4ColoredCloud> generate_cloud();
//    std::shared_ptr<K4PixelsFrame> generate_original_size_color_frame();
//    std::shared_ptr<K4PixelsFrame> generate_depth_filtered_color_frame();
//    std::shared_ptr<K4PixelsFrame> generate_depth_frame();
//    std::shared_ptr<K4PixelsFrame> generate_infrared_frame();

//    void write_display_data_frame(const Parameters &p, std::shared_ptr<DisplayData> &frame);
    void write_display_data_frame(const Parameters &p,
        k4a::image color,
        k4a::image depth,
        k4a::image cloud,
        std::shared_ptr<DisplayDataFrame> &frame);


    // not used
    static void create_xy_table(
        const k4a::calibration &calibration,
        k4a::image &xyTable
    );
    static int generate_point_cloud(
        k4a::image &depthImage,
        k4a::image &xyTable,
        k4a::image &pointCloud
    );
    static bool write_point_cloud(
        const std::string &fileName,
        const k4a::image &pointCloud,
        int pointCount
    );
    static bool tranformation_helpers_write_point_cloud(
        const k4a::image &pointCloudImage,
        const k4a::image &colorImage,
        const std::string &fileName
    );
    static bool point_cloud_depth_to_color(
        const k4a::transformation &transformationHandle,
        const k4a::image &depthImage,
        const k4a::image &colorImage,
        const std::string &fileName
    );
    static bool write_color_bgra_image(const std::string &pathImage, const k4a::image &colorImage);
    static bool write_depth_image(const std::string &pathImage, const k4a::image &depthImage);
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

    Logger::message("Opened device:\n");
    Logger::message(std::format("  Serialnum: {}\n", i->device.get_serialnum()));
    Logger::message("  Version:\n");
//    Logger::message(std::format("      Firmware build: {}\n", i->device.get_version().firmware_build));
//    Logger::message(std::format("      Firmware signature: {}\n", i->device.get_version().firmware_signature));

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
//        i->frameReaderT = std::make_unique<std::thread>(&Kinect4::read_frames, this);
        i->frameReaderT = std::make_unique<std::thread>(&Kinect4::read_frames2, this);
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

    i->k4aConfig = k4aConfig;
    try {
        // retrieve calibration
        Logger::message("get_calibration\n");
        i->calibration = i->device.get_calibration(i->k4aConfig.depth_mode, i->k4aConfig.color_resolution);

        // start cameras
        Logger::message("start_cameras\n");
        i->device.start_cameras(&i->k4aConfig);

        // start imu
        //        i->device.start_imu();

    }  catch (std::runtime_error error) {
        Logger::error("[Kinect4] start_cameras error: {]\n", error.what());
        i->k4aConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        return false;
    }

    Logger::message("Calibration:\n");
    Logger::message(std::format("  color resolution: {}\n", static_cast<int>(i->calibration.color_resolution)));
    Logger::message("  color camera:\n");
    Logger::message(std::format("      width: {}\n", i->calibration.color_camera_calibration.resolution_width));
    Logger::message(std::format("      height: {}\n", i->calibration.color_camera_calibration.resolution_height));
    Logger::message(std::format("      metric radius: {}\n", i->calibration.color_camera_calibration.metric_radius));

    Logger::message("  depth mode:\n");
    Logger::message(std::format("      width: {}\n", i->calibration.depth_camera_calibration.resolution_width));
    Logger::message(std::format("      height: {}\n", i->calibration.depth_camera_calibration.resolution_height));


    return true;
}

void Kinect4::stop_cameras(){
    i->device.stop_cameras();
//    i->device.stop_imu();
}

void Kinect4::read_frames(){


    const int32_t timeoutMs = 1000;

    // init decompressor
    tjhandle m_jpegUncompressor = tjInitDecompress();


    // init uncompressed color image
    const auto colorDims    = k4a::GetColorDimensions(i->k4aConfig.color_resolution);
    const size_t colorWidth  = std::get<0>(colorDims);
    const size_t colorHeight = std::get<1>(colorDims);
    const size_t colorSize   = colorWidth*colorHeight;
    i->uncompressedColorImage = k4a::image::create(
        K4A_IMAGE_FORMAT_COLOR_BGRA32,
        colorWidth,
        colorHeight,
        colorWidth * 4
    );

    // init future resize image
    const auto depthDims     = k4a::GetDepthDimensions(i->k4aConfig.depth_mode);
    const size_t depthWidth  = std::get<0>(depthDims);
    const size_t depthHeight = std::get<1>(depthDims);
    const size_t depthSize   = depthWidth*depthHeight;
    i->resizedUncompressedColorImage = k4a::image::create(
        K4A_IMAGE_FORMAT_COLOR_BGRA32,
        depthWidth,
        depthHeight,
        static_cast<int32_t>(depthWidth * 4 * sizeof(uint8_t))
    );

    // indices
    i->indicesDepths1D.resize(depthSize);
    std::iota(std::begin(i->indicesDepths1D), std::end(i->indicesDepths1D), 0);

    i->indicesDepths3D.resize(depthSize);
    size_t id = 0;
    for(size_t ii = 0; ii < depthWidth; ++ii){
        for(size_t jj = 0; jj < depthHeight; ++jj){
            i->indicesDepths3D[id] = {id,ii,jj};
            ++id;
        }
    }

    i->indicesColors1D.resize(colorSize);
    std::iota(std::begin(i->indicesColors1D), std::end(i->indicesColors1D), 0);

    i->colorCloudCorr.resize(colorSize);

    // init capture
    k4a::capture capture;

    // init transform
    i->transformation = k4a::transformation(i->calibration);    
    i->resizedDepthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        static_cast<int>(colorWidth),
        static_cast<int>(colorHeight),
        static_cast<int32_t>(colorWidth * sizeof(uint16_t))
    );
    i->pointCloudImage = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        static_cast<int>(colorWidth),
        static_cast<int>(colorHeight),
        static_cast<int32_t>(colorWidth * 3 * sizeof(int16_t))
    );

    // init frames
    i->f1 = std::make_shared<DisplayDataFrame>();
    i->f1->depthFrame = PixelsFrame{
        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
    };
    i->f1->colorFrame = PixelsFrame{
        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
    };
    i->f1->infraredFrame = PixelsFrame{
        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
    };
    i->f1->cloud = ColoredCloudFrame{
        0,
        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
    };
    i->f2 = std::make_shared<DisplayDataFrame>();
    i->f2->depthFrame = PixelsFrame{
        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
    };
    i->f2->colorFrame = PixelsFrame{
        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
    };
    i->f2->infraredFrame = PixelsFrame{
        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
    };
    i->f2->cloud = ColoredCloudFrame{
        0,
        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
    };

    while(i->readFramesFromCameras){

        // get a capture
        i->times.startFrameReadingTS = nanoseconds_since_epoch();
        try {
            tool::BenchGuard("[Kinect4] Device get_capture");
            if(!i->device.get_capture(&capture, std::chrono::milliseconds(timeoutMs))){
                Logger::error("[Kinect4] get_capture timeout\n");
                continue;
            }
        }  catch (std::runtime_error error) {
            Logger::error(std::format("[Kinect4] get_capture error: {}\n", error.what()));
            i->readFramesFromCameras = false;
            break;
        }
        i->times.afterCaptureTS = nanoseconds_since_epoch();

        // get a color image
        i->colorImage = capture.get_color_image();        
        i->times.getColorTS = i->colorImage.get_system_timestamp();
        if (!i->colorImage.is_valid()){
            Logger::error("Failed to get color image from capture\n");
            tool::Bench::stop();
            continue;
        }

        // get a depth image
        i->depthImage = capture.get_depth_image();
        i->times.getDepthTS = i->depthImage.get_system_timestamp();
        if (!i->depthImage.is_valid()){
            Logger::error("Failed to get depth image from capture\n");
            tool::Bench::stop();
            continue;
        }

        // get an infrared image
        // ...

        i->times.afterGetImagesTS = nanoseconds_since_epoch();

        i->parametersM.lock();
        const auto p = i->parameters;
        i->parametersM.unlock();

        if(i->k4aConfig.color_format == K4A_IMAGE_FORMAT_COLOR_MJPG){

            // uncompress jpeg image to bgra32
            const int decompressStatus = tjDecompress2(
                m_jpegUncompressor,
                i->colorImage.get_buffer(),
                static_cast<unsigned long>(i->colorImage.get_size()),
                i->uncompressedColorImage.get_buffer(),
                std::get<0>(colorDims),
                0, // pitch
                std::get<1>(colorDims),
                TJPF_BGRA,
                TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE
            );

            if(decompressStatus != 0){
                Logger::error(std::format("[Kinect4] uncompress jpeg color image error: {}\n", tjGetErrorStr2(m_jpegUncompressor)));
                continue;
            }

        }else{
            Logger::error("Color format not managed.\n");
            i->readFramesFromCameras = false;
            break;
        }

        i->times.afterUncompressTS = nanoseconds_since_epoch();

        // transform color image to depth size
        i->transformation.color_image_to_depth_camera(
            i->depthImage,
            i->uncompressedColorImage,
            &i->resizedUncompressedColorImage
        );

        i->times.afterTransformTS = nanoseconds_since_epoch();

        // apply filtering on depth image
        filter_depth_image(p);

        i->times.afterFilteringTS = nanoseconds_since_epoch();

        // create and send compressed frame
        if(p.sendCompressedDataFrame){
            auto frame = i->generate_compressed_data_frame();
            if(frame){
                new_compressed_data_frame_signal(std::move(frame));
            }
        }

        i->times.afterSendingCompressedTS = nanoseconds_since_epoch();

        // display frames/cloud
        if(p.sendDisplayColorFrame || p.sendDisplayDepthFrame || p.sendDisplayInfraredFrame || p.sendDisplayCloud){

            // process
            process_display_data();

            i->times.afterProcessingDisplayDataTS = nanoseconds_since_epoch();

            i->write_display_data_frame(p,
                i->uncompressedColorImage, i->resizedDepthImage, i->pointCloudImage,
                i->f1);

            i->times.afterGeneratingDisplayDataTS = nanoseconds_since_epoch();

            new_display_frame_signal(i->f1);
            std::swap(i->f1, i->f2);
        }

        i->times.endFrameReadingTS = nanoseconds_since_epoch();

        // send times
        new_times_signal(i->times);
    }
}

void Kinect4::read_frames2(){

    if(!is_opened() || i->readFramesFromCameras){
        Logger::error("[Kinect4] Cannot start reading frames.\n");
        return;
    }

    i->readFramesFromCameras = true;

    const int32_t timeoutMs = 1000;

    // init compressor
    tjhandle m_jpegCompressor = tjInitCompress();

    const auto colorDims    = k4a::GetColorDimensions(i->k4aConfig.color_resolution);
    const size_t colorWidth  = std::get<0>(colorDims);
    const size_t colorHeight = std::get<1>(colorDims);
    const size_t colorSize   = colorWidth*colorHeight;

    const auto depthDims     = k4a::GetDepthDimensions(i->k4aConfig.depth_mode);
    const size_t depthWidth  = std::get<0>(depthDims);
    const size_t depthHeight = std::get<1>(depthDims);
    const size_t depthSize   = depthWidth*depthHeight;

    // init capture
    k4a::capture capture;

    // init transform
    i->transformation = k4a::transformation(i->calibration);

    // indices
    i->indicesColors1D.resize(colorSize);
    std::iota(std::begin(i->indicesColors1D), std::end(i->indicesColors1D), 0);

    // init bgra image
    k4a::image bgraColorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        colorWidth,
        colorHeight,
        static_cast<int32_t>(colorWidth * 4 * sizeof(uint8_t))
    );

    k4a::image resizedColorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        depthWidth,
        depthHeight,
        static_cast<int32_t>(depthWidth * 4 * sizeof(uint8_t))
    );


    // init frames
    auto f1 = std::make_shared<DisplayDataFrame>();
    auto f2 = std::make_shared<DisplayDataFrame>();
//    f1->depthFrame = PixelsFrame{
//        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
//    };
//    f1->colorFrame = PixelsFrame{
//        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
//    };
//    f1->infraredFrame = PixelsFrame{
//        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
//    };
//    f1->cloud = ColoredCloud{
//        0,
//        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
//        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
//    };


//    f2->depthFrame = PixelsFrame{
//        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
//    };
//    f2->colorFrame = PixelsFrame{
//        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
//    };
//    f2->infraredFrame = PixelsFrame{
//        colorWidth, colorHeight, std::vector<geo::Pt3f>(colorSize, geo::Pt3f{})
//    };
//    f2->cloud = ColoredCloud{
//        0,
//        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
//        std::vector<geo::Pt3f>(colorSize, geo::Pt3f{}),
//    };


    while(i->readFramesFromCameras){

        Logger::message("read frame");

        i->parametersM.lock();
        const auto p = i->parameters;
        i->parametersM.unlock();

        // get a capture
        i->times.startFrameReadingTS = nanoseconds_since_epoch();
        try {
            tool::BenchGuard("[Kinect4] Device get_capture");
            if(!i->device.get_capture(&capture, std::chrono::milliseconds(timeoutMs))){
                Logger::error("[Kinect4] get_capture timeout\n");
                continue;
            }
        }  catch (std::runtime_error error) {
            Logger::error(std::format("[Kinect4] get_capture error: {}\n", error.what()));
            i->readFramesFromCameras = false;
            break;
        }
        i->times.afterCaptureTS = nanoseconds_since_epoch();

        // get a color image
        i->colorImage = capture.get_color_image();
        if (!i->colorImage.is_valid()){
            Logger::error("Failed to get color image from capture\n");
            continue;
        }
        i->times.getColorTS = i->colorImage.get_system_timestamp();

        // convert color image and applt yuy2 filtering
        if(image_format(i->config.mode) == ImageFormat::YUY2){

            Logger::message("convert yuy2\n");

            auto colorsYuy2 = reinterpret_cast<geo::Pt4<std::uint8_t>*>(i->colorImage.get_buffer());
            auto colorsBGRA = reinterpret_cast<geo::Pt4<std::uint8_t>*>(bgraColorImage.get_buffer());
            for_each(std::execution::par_unseq, std::begin(i->indicesColors1D), std::end(i->indicesColors1D), [&](size_t &idC){

                if(idC%2 == 0){

                    idC = idC/2;

                    const Pt4<uint8_t> &yuy2 = colorsYuy2[idC];

                    // convert to rgb
                    const int y0 = std::clamp(static_cast<int>(p.yFactor*yuy2.x()), 0, 255);
                    auto ci = 298 * (y0 - 16);

                    const int u0 = std::clamp(static_cast<int>(p.uFactor*yuy2.y()), 0, 255);
                    const auto d = u0 - 128;

                    const int v0 = std::clamp(static_cast<int>(p.vFactor*yuy2.w()), 0, 255);
                    const auto e = v0 - 128;

                    colorsBGRA[idC] = geo::Pt4<uint8_t>{
                        static_cast<uint8_t>(std::clamp((ci + (409 * e) + 128)              >> 8, 0, 255)), // red
                        static_cast<uint8_t>(std::clamp((ci + (-100 * d) - (208 * e) + 128) >> 8, 0, 255)), // green
                        static_cast<uint8_t>(std::clamp((ci + (516 * d) + 128)              >> 8, 0, 255)), // blue
                        255
                    };

                }else{

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

                    colorsBGRA[idC] = geo::Pt4<uint8_t>{
                        static_cast<uint8_t>(std::clamp((ci + v3) >> 8, 0, 255)), // red
                        static_cast<uint8_t>(std::clamp((ci + v2) >> 8, 0, 255)), // green
                        static_cast<uint8_t>(std::clamp((ci + v1) >> 8, 0, 255)), // blue
                        255
                    };
                }
            });           

        }else{
            bgraColorImage = i->colorImage;
        }

        Logger::message(std::format("F {} {} ", bgraColorImage.get_width_pixels(),    bgraColorImage.get_height_pixels()));
        Logger::message(std::format("R {} {} ", resizedColorImage.get_width_pixels(), resizedColorImage.get_height_pixels()));

        if(depth_mode(i->config.mode) != DepthMode::OFF){
            // get a depth image
            i->depthImage = capture.get_depth_image();
            if (!i->depthImage.is_valid()){
                Logger::error("Failed to get depth image from capture\n");
                tool::Bench::stop();
                continue;
            }
            i->times.getDepthTS = i->depthImage.get_system_timestamp();

            // resize color image to fit depth
            i->transformation.color_image_to_depth_camera(i->depthImage, bgraColorImage, &resizedColorImage);


            // apply color filtering to depth
            // ...

            // apply depth filtering
            // ...

        }else{
            resizedColorImage = i->colorImage;
        }


//        i->transformation.depth_image_to_color_camera(i->depthImage, &i->resizedDepthImage);

//        i->transformation.depth_image_to_point_cloud(i->depthImage, K4A_CALIBRATION_TYPE_COLOR, &i->pointCloudImage);

        Logger::message("write_display_data_frame\n");
        i->write_display_data_frame(p,
            bgraColorImage, i->depthImage, i->pointCloudImage,
            f1
        );

        new_display_frame_signal(f1);
        std::swap(f1, f2);

//        if(i->config.mode == Mode::Cloud_640x576 ||){

//        }

        // if send compressed:
        //  if only color
        //  -> compress image using jpeg
        //  if frames
        //  -> compress image using jpeg
        //  -> compress depth using encode_integers
        //  -> compress infrared using encode_integers
        // if cloud
        //  -> compress image using jpeg
        //  -> compress depth using encode_integers


        // if display
        //  if only color
        //  -> copy color to pixel frame
        //  if frames
        //      if display_color
        //          -> copy color to pixel frame
        //      if display_depth
        //          -> copy depth to pixel frame
        //      if display_infrared
        //          -> copy infrared to pixel frame
        //  if cloud
        //      if display_color
        //          -> copy color to pixel frame
        //      if display_depth
        //          -> copy depth to pixel frame
        //      if display_cloud
        //          -> generate cloud and copy



        // std::cout << (i->times.afterCaptureTS-i->times.startFrameReadingTS).count() << " " << (i->times.startFrameReadingTS-i->times.getColorTS).count() << "\n";

//        std::cout << "-> " << resizedColorImage.get_width_pixels() << " " << resizedColorImage.get_height_pixels() << "\n";
//        i->colorImage;

        // if display
        //  -> cloud depth_res
        //  -> depth depth_res
        //  -> infra depth_res
        //  -> color depth_res

//        // display frames/cloud
//        if(p.sendDisplayColorFrame || p.sendDisplayDepthFrame || p.sendDisplayInfraredFrame || p.sendDisplayCloud){

//            // process
//            process_display_data();

//            i->times.afterProcessingDisplayDataTS = nanoseconds_since_epoch();



//            i->times.afterGeneratingDisplayDataTS = nanoseconds_since_epoch();

//            Logger::message("new_display_frame_signal");
//            new_display_frame_signal(i->f1);
//            std::swap(i->f1, i->f2);
//        }
    }
    i->readFramesFromCameras = false;
}

void Kinect4::filter_depth_image(const Parameters &p){

    // retrieve buffers
    auto depthBuffer        = reinterpret_cast<std::int16_t*>(i->depthImage.get_buffer());
    auto colorBuffer        = reinterpret_cast<geo::Pt4<std::uint8_t>*>(i->resizedUncompressedColorImage.get_buffer());

    i->validDepthValues = 0;

    for_each(std::execution::par_unseq, std::begin(i->indicesDepths3D), std::end(i->indicesDepths3D), [&](const Pt3<size_t> &dIndex){

        const size_t id = dIndex.x();
        const size_t ii = dIndex.y();
        const size_t jj = dIndex.z();

        if(depthBuffer[id] == invalid_depth_value){
            return;
        }

        // depth filtering
        if(ii < p.minWidth){
            depthBuffer[id] = invalid_depth_value;
            return;;
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
        const auto color = &colorBuffer[id];

        if(p.filterDepthWithColor){
            auto delta = sqrt(
                (color->x()-p.filterColor.x())*(color->x()-p.filterColor.x()) +
                (color->y()-p.filterColor.y())*(color->y()-p.filterColor.y()) +
                (color->z()-p.filterColor.z())*(color->z()-p.filterColor.z())
            );
            if(delta > p.maxDiffColor.x()){
                depthBuffer[id] = invalid_depth_value;
                return;
            }
        }

        if(color->x() == 0 && color->y() == 0 && color->z() == 0 && color->w() == 0){
            depthBuffer[id] = invalid_depth_value;
            return;
        }

        // increment counter
        i->validDepthValues.fetch_add(1, std::memory_order_relaxed);
    });
}

std::shared_ptr<CompressedDataFrame> Kinect4::Impl::generate_compressed_data_frame(){

    auto frame = std::make_shared<CompressedDataFrame>(CompressedDataFrame{
        static_cast<size_t>(colorImage.get_width_pixels()),
        static_cast<size_t>(colorImage.get_height_pixels()),
        std::vector<std::uint8_t>(colorImage.get_size()),
        static_cast<size_t>(depthImage.get_width_pixels()),
        static_cast<size_t>(depthImage.get_height_pixels()),
        std::vector<std::uint32_t>((depthImage.get_width_pixels()*depthImage.get_height_pixels())/2 + 1024, 0)
    });

    // init compressed depth buffer
    std::fill(std::begin(frame->compressedDepthBuffer), std::end(frame->compressedDepthBuffer), 0);

    // compress depth buffer
    size_t sizeDepthCompressed = depthCompressor.encode(
        reinterpret_cast<uint32_t*>(depthImage.get_buffer()), (depthImage.get_width_pixels()*depthImage.get_height_pixels())/2,
        frame->compressedDepthBuffer.data(), (depthImage.get_width_pixels()*depthImage.get_height_pixels())/2 + 1024
    );
    if(sizeDepthCompressed == 0){
        return nullptr;
    }
    frame->compressedDepthBuffer.resize(sizeDepthCompressed);

    // create compressed color buffer
    std::copy(colorImage.get_buffer(), colorImage.get_buffer() + colorImage.get_size(), frame->compressedColorBuffer.data());

    return frame;
}


void Kinect4::process_display_data(){

//    auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();

    // transforms images              
    i->transformation.depth_image_to_color_camera(i->depthImage, &i->resizedDepthImage);

//    auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();

    i->transformation.depth_image_to_point_cloud(i->resizedDepthImage, K4A_CALIBRATION_TYPE_COLOR, &i->pointCloudImage);

//    auto d3 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();

    // retrieve buffers
    auto pointCloudImageData      = reinterpret_cast<geo::Pt3<int16_t>*>(i->pointCloudImage.get_buffer());

    // get current point cloud image size
    i->pointCloudImageWidth    = i->pointCloudImage.get_width_pixels();
    i->pointCloudImageHeight   = i->pointCloudImage.get_height_pixels();
    i->pointCloudImageSize     = i->pointCloudImageWidth*i->pointCloudImageHeight;

    i->validVerticesCount = 0;
    for (size_t ii = 0; ii < i->pointCloudImageSize; ii++){
        if(pointCloudImageData[ii].z() == 0){
            i->colorCloudCorr[ii] = -1;
        }else{
            i->colorCloudCorr[ii] = i->validVerticesCount++;
        }
    }

//    auto d4 = std::chrono::duration_cast<std::chrono::microseconds>(nanoseconds_since_epoch()).count();
//    std::cout << "d time: " << (d2 - d1) << " " << (d3 - d2) << " " << (d4 - d3) <<  "\n";
}



void Kinect4::Impl::write_display_data_frame(const Parameters &p,
    k4a::image color,
    k4a::image depth,
    k4a::image cloud,
    std::shared_ptr<DisplayDataFrame> &f){




    BenchGuard g("write_display_data_frame");
    std::lock_guard l(f->lock);


    // retrieve buffers
    auto colorBuffer = reinterpret_cast<const geo::Pt4<uint8_t>*>(color.get_buffer());
//    auto depthBuffer = reinterpret_cast<const std::uint16_t*>(depth.get_buffer());
//    auto cloudBuffer = reinterpret_cast<const geo::Pt3<int16_t>*>(cloud.get_buffer());

    // retrieve sizes
    const size_t width  = color.get_width_pixels();
    const size_t height = color.get_height_pixels();
    const size_t size = width * height;



    // resize color frame
    if(p.sendDisplayColorFrame){
        Logger::message(std::format("resize {}\n" , size));
        f->colorFrame.width  = width;
        f->colorFrame.height = height;
        if(f->colorFrame.pixels.size() != size){
            f->colorFrame.pixels.resize(size);
        }
    }
    // resize depth frame
    if(p.sendDisplayDepthFrame || p.sendDisplayCloud){
        //...
    }
    // resize infra frame
    if(p.sendDisplayInfraredFrame){
        // ...
    }
    // resize cloud frame
    if(p.sendDisplayCloud){
        // ...
    }


    // init depth frame
    float min=0.f,max =0.f,diff = 0.f;
    const std_v1<Pt3f> depthGradient ={
        {0.f,0.f,1.f},
        {0.f,1.f,1.f},
        {0.f,1.f,0.f},
        {1.f,1.f,0.f},
        {1.f,0.f,0.f},
    };

//    if(p.sendDisplayDepthFrame){
//        // find min/max
//        const auto [pmin, pmax] = std::minmax_element(depthBuffer, depthBuffer + size);
//        min = *pmin;
//        max = *pmax;
//        // min = 0;
//        // max = 16000;
//        diff = max-min;
//    }

    const geo::Pt3f defaultValue = {};





    // fill frames/cloud
    for_each(std::execution::par_unseq, std::begin(indicesColors1D), std::end(indicesColors1D), [&](size_t id){

//        // depth frame
//        if(p.sendDisplayDepthFrame){

//            if(depthBuffer[id] != 0){

//                float vF = static_cast<float>(depthBuffer[id] - min)/diff;
//                float intPart;
//                float decPart = std::modf((vF*(depthGradient.size()-1)), &intPart);
//                size_t idG = static_cast<size_t>(intPart);
//                f->depthFrame.pixels[id] = depthGradient[idG]*(1.f-decPart) + depthGradient[idG+1]*decPart;

//            }else{
//                f->depthFrame.pixels[id] = defaultValue;
//            }
//        }

//        // infrared frame
//        if(p.sendDisplayInfraredFrame){
//            // ...
//        }

        // color frame / cloud
        if(p.sendDisplayColorFrame || p.sendDisplayCloud){

            const auto col = geo::Pt3f{
                static_cast<float>(colorBuffer[id].z()),
                static_cast<float>(colorBuffer[id].y()),
                static_cast<float>(colorBuffer[id].x())
            }/255.f;

            // color frame
            if(p.sendDisplayColorFrame){
                f->colorFrame.pixels[id] = col;
            }

            // cloud
//            if(p.sendDisplayCloud){
//                const auto cId = colorCloudCorr[id];
//                if(cId != -1){
//                    f->cloud.vertices[cId] = geo::Pt3f{
//                        static_cast<float>(-cloudBuffer[id].x()),
//                        static_cast<float>(-cloudBuffer[id].y()),
//                        static_cast<float>( cloudBuffer[id].z())
//                    }*0.01f;
//                    f->cloud.colors[cId] = col;
//                }
//            }
        }
    });



//    if(p.sendDisplayCloud){
//        f->cloud.validVerticesCount = validVerticesCount;
//    }


}


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

void Kinect4::Impl::create_xy_table(const k4a::calibration &calibration, k4a::image &xyTable){

    k4a_float2_t *tableData = reinterpret_cast<k4a_float2_t*>(xyTable.get_buffer());
    const int width   = calibration.depth_camera_calibration.resolution_width;
    const int height  = calibration.depth_camera_calibration.resolution_height;

    k4a_float2_t point;
    k4a_float3_t ray;
    bool valid;

    for (int y = 0, idx = 0; y < height; y++){

        point.xy.y = static_cast<float>(y);
        for (int x = 0; x < width; x++, idx++){
            point.xy.x = static_cast<float>(x);

            // Transform a 2D pixel coordinate with an associated depth value of the source camera into a 3D point
            // of the target coordinate system.
            valid = calibration.convert_2d_to_3d(
                point,
                1.f,
                K4A_CALIBRATION_TYPE_DEPTH,
                K4A_CALIBRATION_TYPE_DEPTH,
                &ray
                );

            if (valid){
                tableData[idx].xy.x = ray.xyz.x;
                tableData[idx].xy.y = ray.xyz.y;
            }else{
                tableData[idx].xy.x = nanf("");
                tableData[idx].xy.y = nanf("");
            }
        }
    }
}

int Kinect4::Impl::generate_point_cloud(k4a::image &depthImage, k4a::image &xyTable, k4a::image &pointCloud){

    const int width   = depthImage.get_width_pixels();
    const int height  = depthImage.get_height_pixels();

    uint16_t     *depthData      = reinterpret_cast<uint16_t*>(depthImage.get_buffer());
    k4a_float2_t *xyTableData    = reinterpret_cast<k4a_float2_t*>(xyTable.get_buffer());
    k4a_float3_t *pointCloudData = reinterpret_cast<k4a_float3_t*>(pointCloud.get_buffer());

    int pointCount = 0;
    for (int ii = 0; ii < width * height; ii++){
        if (depthData[ii] != 0 && !isnan(xyTableData[ii].xy.x) && !isnan(xyTableData[ii].xy.y)){
            pointCloudData[ii].xyz.x = xyTableData[ii].xy.x * (float)depthData[ii];
            pointCloudData[ii].xyz.y = xyTableData[ii].xy.y * (float)depthData[ii];
            pointCloudData[ii].xyz.z = static_cast<float>(depthData[ii]);
            ++pointCount;
        }else{
            pointCloudData[ii].xyz.x = nanf("");
            pointCloudData[ii].xyz.y = nanf("");
            pointCloudData[ii].xyz.z = nanf("");
        }
    }
    return pointCount;
}



bool Kinect4::Impl::write_point_cloud(const std::string &fileName, const k4a::image &pointCloud, int pointCount){

    const int width   = pointCloud.get_width_pixels();
    const int height  = pointCloud.get_height_pixels();
    const k4a_float3_t *pointCloudData = reinterpret_cast<const k4a_float3_t*>(pointCloud.get_buffer());

    // save to the ply file
    std::ofstream ofs;
    ofs.open(fileName);
    if(!ofs.is_open()){
        return false;
    }

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << pointCount << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";
    ofs.close();

    std::stringstream ss;
    for (int ii = 0; ii < width * height; ii++){

        if (isnan(pointCloudData[ii].xyz.x) || isnan(pointCloudData[ii].xyz.y) || isnan(pointCloudData[ii].xyz.z)){
            continue;
        }

        ss << static_cast<float>(pointCloudData[ii].xyz.x) << " "
           << static_cast<float>(pointCloudData[ii].xyz.y) << " "
           << static_cast<float>(pointCloudData[ii].xyz.z) << "\n";
    }

    std::ofstream ofsText(fileName, std::ios::out | std::ios::app);
    ofsText.write(ss.str().c_str(), (std::streamsize)ss.str().length());

    return true;
}

bool Kinect4::Impl::tranformation_helpers_write_point_cloud(const k4a::image &pointCloudImage, const k4a::image &colorImage, const std::string &fileName){

    std::vector<color_point_t> points;

    const int width  = pointCloudImage.get_width_pixels();
    const int height = pointCloudImage.get_height_pixels();

    const int16_t *pointCloudImageData = reinterpret_cast<const int16_t*>(pointCloudImage.get_buffer());
    const uint8_t *colorImageData      = reinterpret_cast<const uint8_t*>(colorImage.get_buffer());

    for (int ii = 0; ii < width * height; ii++){

        color_point_t point;
        point.xyz[0] = pointCloudImageData[3 * ii + 0];
        point.xyz[1] = pointCloudImageData[3 * ii + 1];
        point.xyz[2] = pointCloudImageData[3 * ii + 2];
        if (point.xyz[2] == 0){
            continue;
        }

        point.rgb[0] = colorImageData[4 * ii + 0];
        point.rgb[1] = colorImageData[4 * ii + 1];
        point.rgb[2] = colorImageData[4 * ii + 2];
        uint8_t alpha = colorImageData[4 * ii + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0){
            continue;
        }

        points.push_back(point);
    }

    // save to the ply file
    std::ofstream ofs;
    ofs.open(fileName);
    if(!ofs.is_open()){
        return false;
    }

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << points.size() << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";
    ofs.close();

    std::stringstream ss;
    for (size_t ii = 0; ii < points.size(); ++ii){

        // image data is BGR
        ss << (float)points[ii].xyz[0] << " " << (float)points[ii].xyz[1] << " " << (float)points[ii].xyz[2];
        ss << " " << (float)points[ii].rgb[2] << " " << (float)points[ii].rgb[1] << " " << (float)points[ii].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(fileName, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());

    return true;
}

bool Kinect4::Impl::point_cloud_depth_to_color(
    const k4a::transformation &transformationHandle,
    const k4a::image &depthImage,
    const k4a::image &colorImage,
    const std::string &fileName){

    k4a::image transformedDepthImage = transformationHandle.depth_image_to_color_camera(depthImage);
    k4a::image pointCloudImage       = transformationHandle.depth_image_to_point_cloud(
        transformedDepthImage,
        K4A_CALIBRATION_TYPE_COLOR
    );
    return tranformation_helpers_write_point_cloud(pointCloudImage, colorImage, fileName);
}



bool Kinect4::Impl::write_color_bgra_image(const std::string &pathImage, const k4a::image &colorImage){

    std_v1<unsigned char> data;
    data.resize(colorImage.get_size());

    auto buffer = colorImage.get_buffer();
    for(size_t ii = 0; ii < colorImage.get_size()/4; ++ii){
        const size_t id = ii*4;
        data[id+0] = buffer[id+2];
        data[id+1] = buffer[id+1];
        data[id+2] = buffer[id+0];
        data[id+3] = buffer[id+3];
    }

    tool::graphics::Texture texColor;
    texColor.copy_2d_data(
        colorImage.get_width_pixels(),
        colorImage.get_height_pixels(),
        4,
        data
        );

    return texColor.write_2d_image_file_data(pathImage);
}

bool Kinect4::Impl::write_depth_image(const std::string &pathImage, const k4a::image &depthImage){

    const size_t w =depthImage.get_width_pixels();
    const size_t h =depthImage.get_height_pixels();
    auto buffer = reinterpret_cast<const std::int16_t*>(depthImage.get_buffer());

    // find min/max
    std::int16_t min = 10000;
    std::int16_t max = -10000;
    for(size_t ii = 0; ii < h*w; ++ii){
        if(buffer[ii] == 0){
            continue;
        }
        if(buffer[ii] < min){
            min = buffer[ii];
        }
        if(buffer[ii] > max){
            max = buffer[ii];
        }
    }

    // cap max
    if(max > 4000){
        max = 4000;
    }

    // resize data
    std_v1<unsigned char> dData;
    dData.resize(3*w*h);
    std::fill(std::begin(dData), std::end(dData), 0);

    for(size_t ii = 0; ii < h*w; ++ii){
        if(buffer[ii] == 0){
            continue;
        }else{
            std::int16_t v = buffer[ii];
            if(v > max){
                v = max;
            }
            v = v-min;
            float vF = 1.f*v/(max-min);
            std::int8_t vI  = vF*255.f;
            dData[ii*3+0] = vI;
            dData[ii*3+1] = vI;
            dData[ii*3+2] = vI;
        }
    }

    tool::graphics::Texture texDetph;
    texDetph.copy_2d_data(
        depthImage.get_width_pixels(),
        depthImage.get_height_pixels(),
        3,
        dData
    );
    return texDetph.write_2d_image_file_data(pathImage);
}
