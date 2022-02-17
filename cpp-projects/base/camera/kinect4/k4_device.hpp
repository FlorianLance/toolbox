
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

// signals
#include "lsignal.h"

// local
#include "k4_frame.hpp"

namespace tool::camera {

class K4Device {

public:

    static k4a_device_configuration_t generate_config(const K4Config &config);

    static k4a_device_configuration_t generate_config(
        K4ImageFormat colFormat,
        K4ColorResolution colResolution,
        K4DepthMode depthMode = K4DepthMode::NFOV_UNBINNED,
        K4Framerate fps = K4Framerate::F30,
        bool synchronizeColorAndDepth = true,
        int delayBetweenColorAndDepthUsec = 0,
        K4SynchronisationMode synchMode = K4SynchronisationMode::Standalone,
        int subordinateDelayUsec = 0,
        bool disableLED = false);

    K4Device();
    ~K4Device();

    // devices
    std::uint32_t nb_devices() const noexcept;
    bool open(std::uint32_t deviceId);
    void close();
    void clean();

    // getters
    bool is_opened() const;
    bool is_reading_frames()const;

    // cameras
    bool start_cameras(const K4Config &config);
    bool start_cameras(const k4a_device_configuration_t &k4aConfig); // private
    void stop_cameras();

    // reading
    bool start_reading();
    void stop_reading();

    // settings
    void set_parameters(const K4Parameters &parameters);

// signals
    lsignal::signal<void(std::shared_ptr<K4DisplayDataFrame> cloud)> new_display_frame_signal;
    lsignal::signal<void(std::shared_ptr<K4CompressedFullFrame> frame)> new_compressed_full_frame_signal;
    lsignal::signal<void(std::shared_ptr<K4CompressedCloudFrame> frame)> new_compressed_cloud_frame_signal;

private:

    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};
}
