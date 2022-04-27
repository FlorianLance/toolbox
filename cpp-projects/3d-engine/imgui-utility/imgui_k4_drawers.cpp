

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

#include "imgui_k4_drawers.hpp"

// std
#include <format>

// base
#include "utility/logger.hpp"

// local
#include "imgui-utility/imgui_std.hpp"

using namespace tool::graphics;

bool K4SettingsDrawer::draw_filters_tab_item(const std::string &tabItemName, camera::K4Mode mode, camera::K4Filters &filters, bool &autoUpdate){

    if (!ImGui::BeginTabItem(tabItemName.c_str())){
        return false;
    }

    ImGui::Spacing();
    ImGui::TextCenter("Thresholds");
    ImGui::Separator();


    bool update = false;

    int minMaxD[2] = {filters.minDepthValue, filters.maxDepthValue};
    auto range = (camera::range(mode)*1000.f).conv<int>();
    if(minMaxD[0] < range.x()){
        minMaxD[0] = range.x();
    }
    if(minMaxD[1] > range.y()){
        minMaxD[1] = range.y();
    }
    ImGui::Text("Depth (mm):");
    if(ImGui::SliderInt2("###settings_depth_min_max_sliderint2", minMaxD, range.x(), range.y())){
        filters.minDepthValue = static_cast<std::int16_t>(minMaxD[0]);
        filters.maxDepthValue = static_cast<std::int16_t>(minMaxD[1]);
        update = true;
    }

    int minMaxWidth[2] = {static_cast<int>(filters.minWidth), static_cast<int>(filters.maxWidth)};
    auto depthRes = camera::depth_resolution(mode);
    if(minMaxWidth[1] > depthRes.x()){
        minMaxWidth[1] = depthRes.x();
    }
    ImGui::Text("Width (pixels):");
    if(ImGui::SliderInt2("###settings_width_min_max_sliderint2", minMaxWidth, 0, depthRes.x())){
        filters.minWidth = static_cast<unsigned int>(minMaxWidth[0]);
        filters.maxWidth = static_cast<unsigned int>(minMaxWidth[1]);
        update = true;
    }

    int minMaxHeight[2] = {static_cast<int>(filters.minHeight), static_cast<int>(filters.maxHeight)};
    if(minMaxHeight[1] > depthRes.y()){
        minMaxHeight[1] = depthRes.y();
    }

    ImGui::Text("Height (pixels):");
    if(ImGui::SliderInt2("###settings_height_min_max_sliderint2", minMaxHeight, 0, depthRes.y())){
        filters.minHeight = static_cast<unsigned int>(minMaxHeight[0]);
        filters.maxHeight = static_cast<unsigned int>(minMaxHeight[1]);
        update = true;
    }

    ImGui::Spacing();
    ImGui::TextCenter("Geometry filtering");
    ImGui::Separator();

    ImGui::Text("Max local depth diff");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100.f);
    if(ImGui::DragFloat("###settings_local_diff_dragfloat", &filters.maxLocalDiff, 0.1f, 0.f, 50.f)){
        update = true;
    }

    ImGui::Spacing();
    ImGui::TextCenter("Color filtering");
    ImGui::Separator();

    if(ImGui::Checkbox("Filter depth with color###settings_filter_depth_with_color_checkbox", &filters.filterDepthWithColor)){
        update = true;
    }
    float filteredColor[3] = {
        filters.filterColor.x()/255.f,
        filters.filterColor.y()/255.f,
        filters.filterColor.z()/255.f
    };
    ImGui::Text("Filtered color");
    if(ImGui::ColorEdit3("###settings_filtered_color_coloredit3", filteredColor)){
        filters.filterColor = {
            static_cast<std::uint8_t>(255*filteredColor[0]),
            static_cast<std::uint8_t>(255*filteredColor[1]),
            static_cast<std::uint8_t>(255*filteredColor[2])
        };
        update = true;
    }

    int maxDiffColor[3] = {
        static_cast<int>(filters.maxDiffColor.x()),
        static_cast<int>(filters.maxDiffColor.y()),
        static_cast<int>(filters.maxDiffColor.z())
    };
    ImGui::Text("Max diff color");
    if(ImGui::SliderInt3("###settings_max_diff_color_sliderint3", maxDiffColor, 0, 255)){
        filters.maxDiffColor = {
            static_cast<std::uint8_t>(maxDiffColor[0]),
            static_cast<std::uint8_t>(maxDiffColor[1]),
            static_cast<std::uint8_t>(maxDiffColor[2])
        };
        update = true;
    }

    ImGui::Spacing();
    ImGui::TextCenter("Invalidate");
    ImGui::Separator();

    if(ImGui::Checkbox("Invalidate color from depth###settings_invalidate_color_from_depth_checkbox", &filters.invalidateColorFromDepth)){
        update = true;
    }

    if(ImGui::Checkbox("Invalidate infra from depth###settings_invalidate_infra_from_depth_checkbox", &filters.invalidateInfraFromDepth)){
        update = true;
    }


    ImGui::Spacing();
    ImGui::Separator();

    bool manualUpdate = false;
    if(ImGui::Button("Update###filters_update_button")){
        manualUpdate = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Auto update###filters_auto_update_cb", &autoUpdate)){}

    ImGui::EndTabItem();

    return (update && autoUpdate) || manualUpdate;
}

bool K4SettingsDrawer::draw_display_setings_tab_item(const std::string &tabItemName, camera::K4DisplaySettings &display, bool &autoUpdate){

    if (!ImGui::BeginTabItem(tabItemName.c_str())){
        return false;
    }

    bool update = false;

    if(ImGui::Checkbox("Cloud visible###display_cloud_visible", &display.cloudVisible)){
        update = true;
    }
    if(ImGui::Checkbox("Force cloud color###display_force_cloud_color", &display.forceCloudColor)){
        update = true;
    }
    if(ImGui::ColorEdit4("Cloud color###display_cloud_color", display.cloudColor.v.data())){
        update = true;
    }

    ImGui::Separator();
    if(ImGui::Checkbox("Use voxels###display_use_voxels", &display.useVoxels)){
        update = true;
    }

    ImGui::SetNextItemWidth(100.f);
    if(ImGui::DragFloat("Size points###display_size_points", &display.sizePoints, 0.1f, 0.1f, 30.f, "%.1f")){
        update = true;
    }
    ImGui::SetNextItemWidth(100.f);
    float sizeV = display.sizeVoxels*1000.f;
    if(ImGui::DragFloat("Size voxel###display_size_voxels", &sizeV, 1.f, 10.f, 0.001f, "%.3f")){
        update = true;
        display.sizeVoxels = sizeV*0.001f;
    }

    ImGui::Spacing();
    ImGui::Separator();

    bool manualUpdate = false;
    if(ImGui::Button("Update###filters_update_button")){
        manualUpdate = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Auto update###filters_auto_update_cb", &autoUpdate)){}

    ImGui::EndTabItem();

    return (update && autoUpdate) || manualUpdate;
}



std::tuple<bool,bool,bool> K4SettingsDrawer::draw_all_settings_tab_item(const std::string &tabItemName, const std::vector<std::string> &devicesName, camera::K4ConfigSettings &config, camera::K4DeviceSettings &device, camera::K4ActionsSettings &action, bool &autoUpdate){

    if (!ImGui::BeginTabItem(tabItemName.c_str())){
        return {false,false,false};
    }
    bool updateC = false, updateD = false, updateA = false;

    draw_config(devicesName, config, updateC);
    draw_device_settings(device, updateD);
    draw_action_settings(action, updateA);

    ImGui::Spacing();
    ImGui::Separator();

    bool manualUpdate = false;
    if(ImGui::Button("Update###settings_update_button")){
        manualUpdate = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Auto update###settings_auto_update_cb", &autoUpdate)){}

    ImGui::EndTabItem();

    return {
        (updateC && autoUpdate) || manualUpdate,
        (updateD && autoUpdate) || manualUpdate,
        (updateA && autoUpdate) || manualUpdate
    };
}

void K4SettingsDrawer::draw_config(const std::vector<std::string> &devicesName, camera::K4ConfigSettings &config, bool &updateP){

    ImGui::Spacing();
    ImGui::TextCenter("Config");
    ImGui::Separator();

    ImGui::Spacing();
    ImGui::Text("Device id:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(130.f);
    if(ImGui::BeginCombo("###settings_device_id", devicesName[config.idDevice].c_str())){
        for(size_t ii = 0; ii < devicesName.size(); ++ii){
            bool selected = ii == config.idDevice;
            if (ImGui::Selectable(devicesName[ii].c_str(),selected)){
                config.idDevice = ii;
                updateP = true;
            }
            if(selected){
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();
    ImGui::Text("Mode:");
    int guiCurrentModeSelection = static_cast<int>(config.mode);
    if(ImGui::Combo("###settings_mode_combo", &guiCurrentModeSelection, modeItems, IM_ARRAYSIZE(modeItems))){
        updateP       = true;
        config.mode  = static_cast<camera::K4Mode>(guiCurrentModeSelection);
    }
    ImGui::Spacing();
}

void K4SettingsDrawer::draw_device_settings(camera::K4DeviceSettings &device, bool &updateP){

    ImGui::Spacing();
    ImGui::TextCenter("Device");
    ImGui::Separator();

    ImGui::Spacing();
    ImGui::Text("Capture:");
    ImGui::SameLine();
    if(ImGui::Checkbox("audio###settings_capture_audio", &device.captureAudio)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("IMU###settings_capture_imu", &device.captureIMU)){
        updateP = true;
    }

    ImGui::Spacing();
    ImGui::Text("Compression:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(75.f);
    int guiCurrentCompressSelection = static_cast<int>(device.compressMode);
    if(ImGui::Combo("###settings_compress_mode", &guiCurrentCompressSelection, compressModeItems, IM_ARRAYSIZE(compressModeItems))){
        updateP = true;
        device.compressMode = static_cast<camera::K4CompressMode>(guiCurrentCompressSelection);
    }

    ImGui::Spacing();
    ImGui::Text("Display on grabber:");
    if(ImGui::Checkbox("RGB###settings_display_rgb", &device.displayRGB)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Depth###settings_display_depth", &device.displayDepth)){
        updateP = true;
    }
    if(ImGui::Checkbox("Infra###settings_display_infra", &device.displayInfra)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Cloud###settings_display_cloud", &device.displayCloud)){
        updateP = true;
    }
}

void K4SettingsDrawer::draw_action_settings(camera::K4ActionsSettings &action, bool &updateP){

    ImGui::Spacing();
    ImGui::TextCenter("Action");
    ImGui::Separator();
    ImGui::Spacing();

    if(ImGui::Checkbox("Start device###settings_start_device", &action.startDevice)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Open camera###settings_open_camera", &action.openCamera)){
        updateP = true;
    }

    ImGui::Spacing();
    ImGui::Text("Data:");
    ImGui::SameLine();
    if(ImGui::Checkbox("Record###settings_record_data_cb", &action.record)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Send###settings_send_data_cb", &action.sendData)){
        updateP = true;
    }

    ImGui::Spacing();
}





void K4CloudsDrawer::populate(size_t nbConnections){
    cloudsD.resize(nbConnections);
}

void K4CloudsDrawer::update_from_display_frame(size_t idCloud, std::shared_ptr<camera::K4DisplayFrame> frame){

    if(idCloud >= cloudsD.size()){
        // error
        return;
    }

    if(cloudsD[idCloud].lastDisplayFrameId == frame->idCapture){
        return;
    }

    if(frame->colorFrame.width > 0){
        cloudsD[idCloud].colorT.init_or_update_8ui(
            static_cast<GLsizei>(frame->colorFrame.width),
            static_cast<GLsizei>(frame->colorFrame.height), 3, frame->colorFrame.pixels.data());
    }
    if(frame->depthFrame.width > 0){
        cloudsD[idCloud].depthT.init_or_update_8ui(
            static_cast<GLsizei>(frame->depthFrame.width),
            static_cast<GLsizei>(frame->depthFrame.height), 3, frame->depthFrame.pixels.data());
    }
    if(frame->infraredFrame.width > 0){
        cloudsD[idCloud].infraT.init_or_update_8ui(
            static_cast<GLsizei>(frame->infraredFrame.width),
            static_cast<GLsizei>(frame->infraredFrame.height), 3, frame->infraredFrame.pixels.data());
    }

    if(frame->cloud.validVerticesCount > 0){
        cloudsD[idCloud].drawer.init(frame->cloud.validVerticesCount, frame->cloud.vertices.data(), frame->cloud.colors.data());
    }

    cloudsD[idCloud].lastDisplayFrameId = frame->idCapture;

//    if(k4M->parameters.captureAudio){
        //        for(size_t idFrame = 0; idFrame < currentData->audioFramesCount; ++idFrame){
        //            for(size_t idChannel = 0; idChannel < channelsData2.size(); ++idChannel){
        //                //                        if(idChannel == 0){
        //                //                            Logger::message(std::to_string(currentData->audioChannelsData[idChannel][idFrame]) + " ");
        //                //                        }
        //                channelsData2[idChannel].push_back(currentData->audioChannelsData[idChannel][idFrame]);
        //            }
        //        }

        //        for(size_t idChannel = 0; idChannel < channelsData2.size(); ++idChannel){
        //            if(channelsData2[idChannel].size() > 50000){
        //                channelsData2[idChannel].erase(channelsData2[idChannel].begin(), channelsData2[idChannel].begin() + (channelsData2[idChannel].size() - 50000));
        //            }
        //        }
//    }
}

void K4CloudsDrawer::update_from_cloud_frame(size_t idCloud, camera::K4CloudFrame *frame){

    if(idCloud >= cloudsD.size()){
        // error
        return;
    }

    if(cloudsD[idCloud].lastCloudFrameId == frame->idCapture){
        return;
    }

    if(frame->cloud.validVerticesCount > 0){
        cloudsD[idCloud].drawer.init(frame->cloud.validVerticesCount, frame->cloud.vertices.data(), frame->cloud.colors.data());
    }

    cloudsD[idCloud].lastCloudFrameId = frame->idCapture;
}

void K4CloudsDrawer::update_from_full_frame(size_t idCloud, camera::K4FullFrame *frame){

    if(idCloud >= cloudsD.size()){
        // error
        return;
    }

    if(cloudsD[idCloud].lastFullFrameId == frame->idCapture){
        return;
    }

    if(frame->colorWidth > 0){
        cloudsD[idCloud].colorT.init_or_update_8ui(
            static_cast<GLsizei>(frame->colorWidth),
            static_cast<GLsizei>(frame->colorHeight), 3, frame->imageColorData.data());
    }
    if(frame->depthWidth > 0){
        cloudsD[idCloud].depthT.init_or_update_8ui(
            static_cast<GLsizei>(frame->depthWidth),
            static_cast<GLsizei>(frame->depthHeight), 3, frame->imageDepthData.data());
    }
    if(frame->infraWidth > 0){
        cloudsD[idCloud].infraT.init_or_update_8ui(
            static_cast<GLsizei>(frame->infraWidth),
            static_cast<GLsizei>(frame->infraHeight), 3, frame->imageInfraData.data());
    }

    if(frame->cloud.validVerticesCount > 0){
        cloudsD[idCloud].drawer.init(frame->cloud.validVerticesCount, frame->cloud.vertices.data(), frame->cloud.colors.data());
    }

    cloudsD[idCloud].lastFullFrameId = frame->idCapture;
}


void K4CloudsDrawer::draw_clouds_to_fbo(const geo::Pt4f &backgroundColor, ImguiFboUiDrawer &fboD){

    if(fboD.texture_id() == 0){
        return;
    }

    fboD.bind();
    fboD.update_viewport();
    fboD.set_gl_states(backgroundColor);

    for(auto &cloudD : cloudsD){

        if(!cloudD.displaySettings.cloudVisible){
            continue;
        }

        auto shader = cloudD.displaySettings.useVoxels ? voxelShader : cloudShader;
        shader->use();
        shader->set_uniform("view", fboD.camera()->view().conv<float>());
        shader->set_uniform("projection", fboD.camera()->projection().conv<float>());
        shader->set_uniform("model", cloudD.model);
        shader->set_uniform("enable_unicolor", cloudD.displaySettings.forceCloudColor);
        shader->set_uniform("unicolor", cloudD.displaySettings.cloudColor);

        if(cloudD.displaySettings.useVoxels){
            shader->set_uniform("hSize", cloudD.displaySettings.sizeVoxels);
        }else{
            shader->set_uniform("size_pt", cloudD.displaySettings.sizePoints);
            shader->set_uniform("camera_position", fboD.camera()->position().conv<float>());
        }
        cloudD.drawer.draw();
    }

    fboD.unbind();
}



