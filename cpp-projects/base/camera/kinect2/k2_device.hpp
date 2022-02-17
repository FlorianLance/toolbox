
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

// std
#include <atomic>

// local
#include "k2_types.hpp"

namespace tool::camera {

/**
 * @brief Check validity of input depth value (defined by kinect2_invalid_value)
 */
constexpr bool is_kinect2_depth_valid(float v){
    return v > -5.f;
}



/**
 * @brief Kinect frames grabber. Can request data using several modes (Kinect2FrameRequest)
 */
class K2Device {

public :

    K2Device();
    ~K2Device();

    /**
     * @brief Will try to open kinect2 camera using input mode, return true if success.
     */
    bool open(K2FrameRequest mode);

    /**
     * @brief Close camera sensors
     */
    void close();

    /**
     * @brief Will fetch a new frame for the camera using current mode.
     */
    std::optional<K2Frame> get_kinect_data();

    /**
     * @brief Return current camera mode
     */
    K2FrameRequest mode() const;

    /**
     * @brief Save camera intrinsics to input file path. Return true if succes.
     */
    bool save_camera_intrisics(std::string filePath);

    /**
     * @brief Save camera depth space table to input file path. Return true if succes.
     */
    bool save_camera_space_depth_table(std::string filePath);


    K2Settings parameters; // Kinect2 current camera parameters


private:

    /**
     * @brief Return camera intrinsics values (Focal length {x,y}, Principal point {x,y}, Radial distortion {second,fourth, sixth} order.
     */
    std_v1<float> intrinsics() const;

    bool acquire_multi_sources_frame();
    bool acquire_color_frame();
    bool acquire_depth_frame();
    bool acquire_infra_frame();
    bool acquire_long_exposure_infra_frame();
    bool acquire_body_frame();

    void get_color_data();
    bool get_depth_data();
    void get_infra_data();
    void get_long_exposure_infra_data();
    void get_body_data();

    void post_computing_color_data();
    void post_computing_depth_data();
    void post_computing_infra_data();
    void post_computing_body_data();

    void process_compressed_color_1920x1080();
    void process_compressed_color_512x424();
    void process_compressed_color_cloud();
    void process_compressed_color_mesh();
    void process_depth_512x424();
    void process_infra_512x424();

    void release_color_data();
    void release_depth_data();
    void release_infra_data();
    void release_long_exposure_infra_data();
    void release_body_data();
    void release_multi_source_data();
    void clean_frame();

    struct Impl;
    std::unique_ptr<Impl> m_p = nullptr;
};
}
