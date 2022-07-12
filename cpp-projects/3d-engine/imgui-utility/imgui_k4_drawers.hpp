
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
#include "camera/kinect4/k4_frame.hpp"

// opengl
#include "opengl/draw/drawer.hpp"
#include "opengl/gl_texture.hpp"

// local
#include "imgui-utility/imgui_fbo.hpp"


namespace tool::graphics {

[[maybe_unused]] static const char* modeItems[] = {
    "Cloud_320x288", "Cloud_640x576", "Cloud_512x512", "Cloud_1024x1024",
    "Full_frame_320x288", "Full_frame_640x576", "Full_frame_512x512", "Full_frame_1024x1024",
    "Only_color_1280x720", "Only_color_1920x1080", "Only_color_2560x1440", "Only_color_2048x1536", "Only_color_3840x2160", "Only_color_4096x3072"
};

[[maybe_unused]] static const char* compressModeItems[] = {
    "Full", "Cloud", "None"
};

class K4SettingsDrawer{
public:
    static void draw_config(const std::vector<std::string> &devicesName, camera::K4ConfigSettings &config,  bool &updateP);
    static void draw_device_settings(camera::K4DeviceSettings &device, bool &updateP);
    static void draw_actions_settings(camera::K4ActionsSettings &actions,  bool &updateP);
    static std::tuple<bool,bool,bool> draw_all_settings_tab_item(
        const std::string &tabItemName,const std::vector<std::string> &devicesName,
        camera::K4ConfigSettings &config,
        camera::K4DeviceSettings &device,
        camera::K4ActionsSettings &actions,
        bool &autoUpdate
    );
    static bool draw_filters_tab_item(const std::string &tabItemName, camera::K4Mode mode, camera::K4Filters &filters, bool &autoUpdate);
    static bool draw_display_setings_tab_item(const std::string &tabItemName, camera::K4DisplaySettings &display, bool &autoUpdate);
    static bool draw_model_tab_item(const std::string &tabItemName, geo::Mat4f &model, bool &autoUpdate);
};


struct K4CloudDrawer{
    // last frame info
    std::int32_t lastDisplayFrameId = -1;
    std::int32_t lastCloudFrameId = -1;
    std::int32_t lastFullFrameId = -1;
    // cloud
    geo::Mat4f model = geo::Mat4f::identity();
    gl::CloudPointsDrawer drawer;
    // textures
    gl::Texture2D colorT;
    gl::Texture2D depthT;
    gl::Texture2D infraT;
    // settings
    camera::K4DisplaySettings displaySettings;
};

struct K4CloudsDrawer{

    void populate(size_t nbConnections);
    void update_from_display_frame(size_t idCloud, std::shared_ptr<camera::K4DisplayFrame> frame);
    void update_from_cloud_frame(size_t idCloud, camera::K4CloudFrame *frame);
    void update_from_full_frame(size_t idCloud, camera::K4FullFrame *frame);
    void draw_clouds_to_fbo(const geo::Pt4f &backgroundColor, ImguiFboUiDrawer &fboD);

    // clouds
    std::vector<K4CloudDrawer> cloudsD;
    gl::ShaderProgram *cloudShader = nullptr;
    gl::ShaderProgram *voxelShader = nullptr;
};


struct CloudPointsFrameDrawer{
    int currentFrameId = 0;
    int startFrameId = 0;
    int endFrameId = 0;
    gl::CloudPointsDrawer cloudD;
};



}



