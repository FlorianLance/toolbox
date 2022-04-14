

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

// local
#include "imgui-utility/imgui_std.hpp"

using namespace tool::graphics;

bool K4SettingsDrawer::draw_filters_settings_tab_item(const std::string &tabItemName, camera::K4Mode mode, camera::K4FiltersSettings &filters, bool &updateP){

    if (!ImGui::BeginTabItem(tabItemName.c_str())){
        return false;
    }

    int minMaxD[2] = {filters.minDepthValue, filters.maxDepthValue};
    auto range = (camera::range(mode)*1000.f).conv<int>();
    if(minMaxD[0] < range.x()){
        minMaxD[0] = range.x();
    }
    if(minMaxD[1] > range.y()){
        minMaxD[1] = range.y();
    }
    if(ImGui::SliderInt2("Depth (mm)###settings_depth_min_max_sliderint2", minMaxD, range.x(), range.y())){
        filters.minDepthValue = static_cast<std::int16_t>(minMaxD[0]);
        filters.maxDepthValue = static_cast<std::int16_t>(minMaxD[1]);
        updateP = true;
    }

    int minMaxWidth[2] = {static_cast<int>(filters.minWidth), static_cast<int>(filters.maxWidth)};
    auto depthRes = camera::depth_resolution(mode);
    if(minMaxWidth[1] > depthRes.x()){
        minMaxWidth[1] = depthRes.x();
    }
    if(ImGui::SliderInt2("Width (pixels)###settings_width_min_max_sliderint2", minMaxWidth, 0, depthRes.x())){
        filters.minWidth = static_cast<unsigned int>(minMaxWidth[0]);
        filters.maxWidth = static_cast<unsigned int>(minMaxWidth[1]);
        updateP = true;
    }

    int minMaxHeight[2] = {static_cast<int>(filters.minHeight), static_cast<int>(filters.maxHeight)};
    if(minMaxHeight[1] > depthRes.y()){
        minMaxHeight[1] = depthRes.y();
    }
    if(ImGui::SliderInt2("Height (pixels)###settings_height_min_max_sliderint2", minMaxHeight, 0, depthRes.y())){
        filters.minHeight = static_cast<unsigned int>(minMaxHeight[0]);
        filters.maxHeight = static_cast<unsigned int>(minMaxHeight[1]);
        updateP = true;
    }

    if(ImGui::Checkbox("Filter depth with color###settings_filter_depth_with_color_checkbox", &filters.filterDepthWithColor)){
        updateP = true;
    }

    if(ImGui::Checkbox("Invalidate color from depth###settings_invalidate_color_from_depth_checkbox", &filters.invalidateColorFromDepth)){
        updateP = true;
    }

    if(ImGui::Checkbox("Invalidate infra from depth###settings_invalidate_infra_from_depth_checkbox", &filters.invalidateInfraFromDepth)){
        updateP = true;
    }
    if(ImGui::DragFloat("Local diff###settings_local_diff_dragfloat", &filters.maxLocalDiff, 0.1f, 0.f, 50.f)){
        updateP = true;
    }

    float filteredColor[3] = {
        filters.filterColor.x()/255.f,
        filters.filterColor.y()/255.f,
        filters.filterColor.z()/255.f
    };
    if(ImGui::ColorEdit3("Filtered color###settings_filtered_color_coloredit3", filteredColor)){
        filters.filterColor = {
            static_cast<std::uint8_t>(255*filteredColor[0]),
            static_cast<std::uint8_t>(255*filteredColor[1]),
            static_cast<std::uint8_t>(255*filteredColor[2])
        };
        updateP = true;
    }

    int maxDiffColor[3] = {
        static_cast<int>(filters.maxDiffColor.x()),
        static_cast<int>(filters.maxDiffColor.y()),
        static_cast<int>(filters.maxDiffColor.z())
    };
    if(ImGui::SliderInt3("Max diff color###settings_max_diff_color_sliderint3", maxDiffColor, 0, 255)){
        filters.maxDiffColor = {
            static_cast<std::uint8_t>(maxDiffColor[0]),
            static_cast<std::uint8_t>(maxDiffColor[1]),
            static_cast<std::uint8_t>(maxDiffColor[2])
        };
        updateP = true;
    }
    ImGui::EndTabItem();

    return true;
}

bool K4SettingsDrawer::draw_display_setings_tab_item(const std::string &tabItemName, camera::K4DisplaySettings &display, bool &updateP){

    if (!ImGui::BeginTabItem(tabItemName.c_str())){
        return false;
    }

    if(ImGui::Checkbox("Cloud visible", &display.cloudVisible)){
        updateP = true;
    }
    if(ImGui::Checkbox("Force cloud color", &display.forceCloudColor)){
        updateP = true;
    }
    if(ImGui::ColorEdit4("Cloud color", display.cloudColor.v.data())){
        updateP = true;
    }

    ImGui::Separator();
    if(ImGui::Checkbox("Use voxels", &display.useVoxels)){
        updateP = true;
    }

    ImGui::SetNextItemWidth(100.f);
    if(ImGui::DragFloat("Size points", &display.sizePoints, 0.1f, 0.1f, 30.f, "%.1f")){
        updateP = true;
    }
    ImGui::SetNextItemWidth(100.f);
    if(ImGui::DragFloat("Size voxel", &display.sizeVoxels, 0.001f, 0.001f, 0.015f, "%.3f")){
        updateP = true;
    }

    ImGui::EndTabItem();
    return true;
}



bool K4SettingsDrawer::draw_all_settings_tab_item(const std::string &tabItemName, camera::K4Config &config, camera::K4DeviceSettings &device, camera::K4ActionsSettings &action, bool &updateC, bool &updateD, bool &updateA){

    if (!ImGui::BeginTabItem(tabItemName.c_str())){
        return false;
    }

    draw_config(config, updateC);
    draw_device_settings(device, updateC);
    draw_action_settings(action, updateC);

    ImGui::EndTabItem();
    return true;
}


void K4SettingsDrawer::draw_config(camera::K4Config &config, bool &updateP){

    ImGui::Text("Mode:");
    int guiCurrentModeSelection = static_cast<int>(config.mode);
    if(ImGui::Combo("###settings_mode_combo", &guiCurrentModeSelection, modeItems, IM_ARRAYSIZE(modeItems))){
        updateP       = true;
        config.mode  = static_cast<camera::K4Mode>(guiCurrentModeSelection);
    }

    //    if(devices.size() > 0){
    //        ImGui::Text("Device id:");
    //        if(ImGui::BeginCombo("###settings_device_id_combo", devices[device.deviceId].c_str())){
    //            for(size_t ii = 0; ii < devices.size(); ++ii){

    //                bool selected = ii == device.deviceId;
    //                if (ImGui::Selectable(devices[ii].c_str(),selected)){
    //                    device.deviceId = ii;
    //                    updateP = true;
    //                }
    //                if(selected){
    //                    ImGui::SetItemDefaultFocus();
    //                }
    //            }
    //            ImGui::EndCombo();
    //        }
    //    }
}


void K4SettingsDrawer::draw_device_settings(camera::K4DeviceSettings &device, bool &updateP){

    ImGui::Text("Capture:");
    if(ImGui::Checkbox("audio###settings_capture_audio_cb", &device.captureAudio)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("IMU###settings_capture_imu_cb", &device.captureIMU)){
        updateP = true;
    }
    ImGui::Separator();

    ImGui::Text("Compression:");
    int guiCurrentCompressSelection = static_cast<int>(device.compressMode);
    if(ImGui::Combo("###settings_compress_combo", &guiCurrentCompressSelection, compressModeItems, IM_ARRAYSIZE(compressModeItems))){
        updateP = true;
        device.compressMode = static_cast<camera::K4CompressMode>(guiCurrentCompressSelection);
    }
    ImGui::Separator();

    ImGui::Text("Display on grabber:");
    if(ImGui::Checkbox("RGB###settings_display_rgb_cb", &device.displayRGB)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Depth###settings_display_depth_cb", &device.displayDepth)){
        updateP = true;
    }
    if(ImGui::Checkbox("Infra###settings_display_infra_cb", &device.displayInfra)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Cloud###settings_display_cloud_cb", &device.displayCloud)){
        updateP = true;
    }
}

void K4SettingsDrawer::draw_action_settings(camera::K4ActionsSettings &action, bool &updateP){

    if(ImGui::Checkbox("Start device", &action.startDevice)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Open camera", &action.openCamera)){
        updateP = true;
    }
    ImGui::Separator();

    ImGui::Text("Data:");
    if(ImGui::Checkbox("Record###settings_record_data_cb", &action.record)){
        updateP = true;
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Send###settings_send_data_cb", &action.sendData)){
        updateP = true;
    }

    ImGui::Separator();
}





void K4CloudsDrawer::populate(size_t nbConnections){
    cloudsD.resize(nbConnections);
}

void K4CloudsDrawer::update_from_display_frame(size_t idCloud, std::unique_ptr<camera::K4DisplayFrame> frame){

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
    fboD.reset_states(backgroundColor);

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


