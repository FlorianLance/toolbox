
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

// base
#include "geometry/matrix4.hpp"

// local
#include "k2_network.hpp"

namespace tool::camera {

class K2ConfigFiles{

public:

    // camera
    // # grabber
    static std::pair<bool, std::string> save_grabber_settings_config_file(const K2Settings &p, std::string path);
    static std::pair<std::optional<K2Settings>, std::string> read_grabber_settings_config_file(std::string path = "");

    // network
    // # grabber
    static std::pair<bool, std::string> read_grabber_network_config_file(int *readingPort, std::string path = "");
    // # manager
    static std::pair<std::vector<K2GrabberTargetInfo>, std::string> read_manager_network_config_file(std::string path = "");

    // calibration
    // # manager
    static std::pair<std::vector<tool::geo::Mat4d>, std::string> read_manager_calibration_file(std::string path = "");
};

}
