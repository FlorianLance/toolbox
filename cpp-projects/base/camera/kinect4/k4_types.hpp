
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

#pragma once

// kinect4
#include <k4a/k4atypes.h>

// local
#include "geometry/point2.hpp"
#include "geometry/point3.hpp"
#include "geometry/point4.hpp"
#include "utility/tuple_array.hpp"

namespace tool::camera {

    enum class K4ImageFormat : std::int8_t {
        MJPEG    = K4A_IMAGE_FORMAT_COLOR_MJPG,   // The buffer for each image is encoded as a JPEG and can be decoded by a JPEG decoder.
        NV12     = K4A_IMAGE_FORMAT_COLOR_NV12,   // NV12 images separate the luminance and chroma data such that all the luminance is at the
        // beginning of the buffer, and the chroma lines follow immediately after.
        YUY2     = K4A_IMAGE_FORMAT_COLOR_YUY2,   // YUY2 stores chroma and luminance data in interleaved pixels.
        BGRA32   = K4A_IMAGE_FORMAT_COLOR_BGRA32, // Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
        // and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.
        IR16     = K4A_IMAGE_FORMAT_IR16,         // Each pixel of IR16 data is two bytes of little endian unsigned depth data.
        // The value of the data represents brightness.
        DEPTH16  = K4A_IMAGE_FORMAT_DEPTH16,      // Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data.
        // The unit of the data is in millimeters from the origin of the camera.
        CUSTOM   = K4A_IMAGE_FORMAT_CUSTOM,       // Used in conjunction with user created images or images packing non-standard data.
        CUSTOM8  = K4A_IMAGE_FORMAT_CUSTOM8,      // Each pixel of CUSTOM8 is a single channel one byte of unsigned data.
        CUSTOM16 = K4A_IMAGE_FORMAT_CUSTOM16      // Each pixel of CUSTOM16 is a single channel two bytes of little endian unsigned data.
    };

    enum class K4ColorResolution : std::int8_t {
        OFF     = K4A_COLOR_RESOLUTION_OFF,     // Color camera will be turned off with this setting
        R720P   = K4A_COLOR_RESOLUTION_720P,    // 1280 * 720  16:9
        R1080P  = K4A_COLOR_RESOLUTION_1080P,   // 1920 * 1080 16:9
        R1440P  = K4A_COLOR_RESOLUTION_1440P,   // 2560 * 1440 16:9
        R1536P  = K4A_COLOR_RESOLUTION_1536P,   // 2048 * 1536 4:3
        R2160P  = K4A_COLOR_RESOLUTION_2160P,   // 3840 * 2160 16:9
        R3072P  = K4A_COLOR_RESOLUTION_3072P    // 4096 * 3072 4:3
    };

    enum class K4DepthMode : std::int8_t {
        OFF            = K4A_DEPTH_MODE_OFF ,              // Depth sensor will be turned off with this setting.
        NFOV_2X2BINNED = K4A_DEPTH_MODE_NFOV_2X2BINNED,    // Depth captured at 320x288. Passive IR is also captured at 320x288.
        NFOV_UNBINNED  = K4A_DEPTH_MODE_NFOV_UNBINNED,     // Depth captured at 640x576. Passive IR is also captured at 640x576.
        WFOV_2X2BINNED = K4A_DEPTH_MODE_WFOV_2X2BINNED,    // Depth captured at 512x512. Passive IR is also captured at 512x512.
        WFOV_UNBINNED  = K4A_DEPTH_MODE_WFOV_UNBINNED,     // Depth captured at 1024x1024. Passive IR is also captured at 1024x1024.
        PASSIVE_IR     = K4A_DEPTH_MODE_PASSIVE_IR,        // Passive IR only, captured at 1024x1024.
    };

    enum class K4Framerate : std::int8_t{
        F5  = K4A_FRAMES_PER_SECOND_5,   // 5 FPS
        F15 = K4A_FRAMES_PER_SECOND_15,  // 15 FPS
        F30 = K4A_FRAMES_PER_SECOND_30,  // 30 FPS
    };

    enum class K4SynchronisationMode : std::int8_t{
        Standalone  = K4A_WIRED_SYNC_MODE_STANDALONE, // Neither 'Sync In' or 'Sync Out' connections are used.
        Master      = K4A_WIRED_SYNC_MODE_MASTER,     // The 'Sync Out' jack is enabled and synchronization data it driven out the connected wire.
        // While in master mode the color camera must be enabled as part of the multi device sync signalling logic.
        // Even if the color image is not needed, the color camera must be running.
        Subordinate = K4A_WIRED_SYNC_MODE_SUBORDINATE // The 'Sync In' jack is used for synchronization and 'Sync Out' is driven for the
        // next device in the chain. 'Sync Out' is a mirror of 'Sync In' for this mode.
    };
    // https://docs.microsoft.com/fr-FR/azure/Kinect-dk/multi-camera-sync

    enum class K4Mode : std::int8_t {
        // clouds
        Cloud_320x288,
        Cloud_640x576,
        Cloud_512x512,
        Cloud_1024x1024,
        // frames
        Full_frame_320x288,
        Full_frame_640x576,
        Full_frame_512x512,
        Full_frame_1024x1024,
        // colors
        Only_color_1280x720,
        Only_color_1920x1080,
        Only_color_2560x1440,
        Only_color_2048x1536,
        Only_color_3840x2160,
        Only_color_4096x3072,
        SizeEnum
    };

    enum K4CompressMode : std::int8_t{
        Full= 0,
        Cloud,
        None
    };


    using M   = K4Mode;
    using IF  = K4ImageFormat;
    using CR  = K4ColorResolution;
    using DM  = K4DepthMode;
    using FPS = K4Framerate;
    using CE = bool;
    using ME = bool;
    using IE = bool;
    using Range = geo::Pt2f;
    using Resolution = geo::Pt2<int>;
    using DRes = Resolution;
    using CRes = Resolution;

    using TMode = std::tuple<
        K4Mode,                   IF,         CR,         DM,                 FPS,      Range,          DRes,       CE,    ME,    IE>;
    static constexpr TupleArray<K4Mode::SizeEnum, TMode> k4Modes = {{
        // cloud
        TMode
        {M::Cloud_320x288,        IF::YUY2,   CR::R720P,  DM::NFOV_2X2BINNED, FPS::F30, {0.5f,5.46f},   {320,288},  true,  false, true},
        {M::Cloud_640x576,        IF::YUY2,   CR::R720P,  DM::NFOV_UNBINNED,  FPS::F30, {0.5f,3.86f},   {640,576},  true,  false, true},
        {M::Cloud_512x512,        IF::YUY2,   CR::R720P,  DM::WFOV_2X2BINNED, FPS::F30, {0.25f,2.88f},  {512,512},  true,  false, true},
        {M::Cloud_1024x1024,      IF::YUY2,   CR::R720P,  DM::WFOV_UNBINNED,  FPS::F15, {0.25f,2.21f},  {1024,1024},true,  false, true},
        // frame
        {M::Full_frame_320x288,   IF::YUY2,   CR::R720P,  DM::NFOV_2X2BINNED, FPS::F30, {0.5f,5.46f},   {320,288},  false, false, true},
        {M::Full_frame_640x576,   IF::YUY2,   CR::R720P,  DM::NFOV_UNBINNED,  FPS::F30, {0.5f,3.86f},   {640,576},  false, false, true},
        {M::Full_frame_512x512,   IF::YUY2,   CR::R720P,  DM::WFOV_2X2BINNED, FPS::F30, {0.25f,2.88f},  {512,512},  false, false, true},
        {M::Full_frame_1024x1024, IF::YUY2,   CR::R720P,  DM::WFOV_UNBINNED,  FPS::F15, {0.25f,2.21f},  {1024,1024},false, false, true},
        // only color
        {M::Only_color_1280x720,  IF::YUY2,   CR::R720P,  DM::OFF,            FPS::F30, {0,0},          {0,0},      false, false, false},
        {M::Only_color_1920x1080, IF::BGRA32, CR::R1080P, DM::OFF,            FPS::F30, {0,0},          {0,0},      false, false, false},
        {M::Only_color_2560x1440, IF::BGRA32, CR::R1440P, DM::OFF,            FPS::F30, {0,0},          {0,0},      false, false, false},
        {M::Only_color_2048x1536, IF::BGRA32, CR::R1536P, DM::OFF,            FPS::F30, {0,0},          {0,0},      false, false, false},
        {M::Only_color_3840x2160, IF::BGRA32, CR::R2160P, DM::OFF,            FPS::F30, {0,0},          {0,0},      false, false, false},
        {M::Only_color_4096x3072, IF::BGRA32, CR::R3072P, DM::OFF,            FPS::F15, {0,0},          {0,0},      false, false, false},
    }};


    [[maybe_unused]] static constexpr K4ImageFormat image_format(K4Mode m) {
        return k4Modes.at<0,1>(m);
    }
    [[maybe_unused]] static constexpr K4ColorResolution color_resolution(K4Mode m) {
        return k4Modes.at<0,2>(m);
    }
    [[maybe_unused]] static constexpr K4DepthMode depth_mode(K4Mode m) {
        return k4Modes.at<0,3>(m);
    }
    [[maybe_unused]] static constexpr K4Framerate framerate(K4Mode m) {
        return k4Modes.at<0,4>(m);
    }
    [[maybe_unused]] static constexpr Range range(K4Mode m) {
        return k4Modes.at<0,5>(m);
    }
    [[maybe_unused]] static constexpr Resolution depth_resolution(K4Mode m) {
        return k4Modes.at<0,6>(m);
    }
    [[maybe_unused]] static constexpr bool has_cloud(K4Mode m) {
        return k4Modes.at<0,7>(m);
    }
    [[maybe_unused]] static constexpr bool has_mesh(K4Mode m) {
        return k4Modes.at<0,8>(m);
    }
    [[maybe_unused]] static constexpr bool has_infrared(K4Mode m) {
        return k4Modes.at<0,9>(m);
    }


    [[maybe_unused]] static constexpr std::int16_t k4_invalid_depth_value = 0;
    [[maybe_unused]] static constexpr std::int16_t k4_invalid_infra_value = 0;
    [[maybe_unused]] static constexpr geo::Pt4<std::uint8_t> k4_invalid_color_value = {0,0,0,0};
    [[maybe_unused]] static constexpr K4Mode k4DefaultMode = K4Mode::Cloud_640x576;

    struct K4ImuSample{
        float temperature;     /**< Temperature reading of this sample (Celsius). */
        geo::Pt3f acc;         /**< Accelerometer sample in meters per second squared. */
        std::int64_t accTsMs;  /**< Timestamp of the accelerometer in microseconds. */
        geo::Pt3f gyr;         /**< Gyro sample in radians per second. */
        std::int64_t gyrTsMs;  /**< Timestamp of the gyroscope in microseconds */
    };

    struct K4DisplaySettings{
        bool cloudVisible = true;
        bool forceCloudColor = false;
        geo::Pt4f cloudColor = {1.f,0.f,0.f, 1.f};
        bool useVoxels = false;
        float sizePoints = 5.f;
        float sizeVoxels = 0.002f;
    };

    struct K4Config{
        std::uint32_t idDevice = 0;
        K4Mode mode = K4Mode::Cloud_640x576;
        bool synchronizeColorAndDepth = true;
        int delayBetweenColorAndDepthUsec = 0;
        K4SynchronisationMode synchMode = K4SynchronisationMode::Standalone;
        int subordinateDelayUsec = 0;
        bool disableLED = false;
    };

    struct K4FiltersSettings{

        // # width / height
        unsigned int minWidth  = 0;
        unsigned int maxWidth  = depth_resolution(k4DefaultMode).x();
        unsigned int minHeight = 0;
        unsigned int maxHeight = depth_resolution(k4DefaultMode).y();

        // color
        float yFactor = 1.f;
        float uFactor = 1.f;
        float vFactor = 1.f;
        geo::Pt3<std::uint8_t> filterColor  = geo::Pt3<std::uint8_t>(255,0,0);
        geo::Pt3<std::uint8_t> maxDiffColor = geo::Pt3<std::uint8_t>(10,40,40);

        // # depth
        std::int16_t minDepthValue = static_cast<std::int16_t>(range(k4DefaultMode).x()*1000.f);
        std::int16_t maxDepthValue = static_cast<std::int16_t>(range(k4DefaultMode).y()*1000.f);

        // compression
        unsigned char jpegCompressionRate = 80;

        // # neigbhours
        float maxLocalDiff = 10.f;
        unsigned char nbMinNeighboursNb = 1;
        unsigned char minNeighboursLoops = 1;

        // flogs
        bool filterDepthWithColor       = false;
        bool invalidateColorFromDepth   = false;
        bool invalidateInfraFromDepth   = false;
    };

    struct K4DeviceSettings{

        // capture
        int deviceId        = 0;
        bool captureAudio   = true;
        bool captureIMU     = true;
        camera::K4CompressMode compressMode  = camera::K4CompressMode::Cloud;

        // display
        bool displayRGB    = false;
        bool displayDepth  = false;
        bool displayInfra  = false;
        bool displayCloud  = false;

        static K4DeviceSettings init_for_grabber(){
            K4DeviceSettings device;
            device.displayRGB    = true;
            device.displayDepth  = true;
            device.displayInfra  = true;
            device.displayCloud  = true;
            return device;
        }
    };

    struct K4ActionsSettings{
        // device
        bool startDevice   = true;
        bool openCamera    = true;
        // network
        bool sendData      = true;
        // record
        bool record        = false;

        static K4ActionsSettings init_for_grabber(){
            K4ActionsSettings actions;
            actions.sendData      = false;
            actions.startDevice   = false;
            actions.openCamera    = false;
            return actions;
        }
    };

    struct K4GrabberSettings{
        K4Config config;
        K4DeviceSettings device = camera::K4DeviceSettings::init_for_grabber();
        K4ActionsSettings actions = camera::K4ActionsSettings::init_for_grabber();
        K4FiltersSettings filters;
    };

}
