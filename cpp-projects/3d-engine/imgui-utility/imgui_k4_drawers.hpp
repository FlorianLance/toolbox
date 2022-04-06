
/*******************************************************************************
** Toolbox-3d-engine                                                          **
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
#include "camera/kinect4/k4_types.hpp"

[[maybe_unused]] static const char* modeItems[] = {
    "Cloud_320x288", "Cloud_640x576", "Cloud_512x512", "Cloud_1024x1024",
    "Full_frame_320x288", "Full_frame_640x576", "Full_frame_512x512", "Full_frame_1024x1024",
    "Only_color_1280x720", "Only_color_1920x1080", "Only_color_2560x1440", "Only_color_2048x1536", "Only_color_3840x2160", "Only_color_4096x3072"
};

[[maybe_unused]] static const char* compressModeItems[] = {
    "Full", "Cloud", "None"
};

namespace tool::graphics {
class K4FiltersTabItem{
public:
    static bool draw(const std::string &tabItemName, camera::K4Mode mode, camera::K4Filters &filters, bool &updateP);    
};
class K4DeviceTabItem{
public:
    static bool draw(const std::string &tabItemName, camera::K4DeviceSettings &device, bool &updateP);
};
class K4DisplaySettingsTabItem{
public:
    static bool draw(const std::string &tabItemName, camera::K4DisplaySettings &display, bool &updateP);
};


}



