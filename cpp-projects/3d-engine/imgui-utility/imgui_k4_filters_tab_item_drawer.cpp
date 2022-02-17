

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

#include "imgui_k4_filters_tab_item_drawer.hpp"

// std
#include <format>

// local
#include "imgui-utility/imgui_std.hpp"

using namespace tool::graphics;

bool K4FiltersTabItem::draw(const std::string &tabItemName, camera::K4Mode mode, camera::K4Filters &filters, bool &updateP){

    if (ImGui::BeginTabItem(tabItemName.c_str())){

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
    }else{
        return false;
    }

    return true;
}
