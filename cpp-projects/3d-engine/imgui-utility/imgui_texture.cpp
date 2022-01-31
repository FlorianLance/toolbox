

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

#include "imgui_texture.hpp"

// std
#include <format>

// local
#include "imgui-utility/imgui_std.hpp"

using namespace tool::graphics;

void ImguiTextureDrawer::draw_texture_tab_child(const std::string &windowName, geo::Pt2<int> sizeWindow, gl::TBO *texture, bool invert){

    if(ImGui::BeginChild(std::format("{}Window",windowName).c_str(), ImVec2(sizeWindow.x(), sizeWindow.y()),false,ImGuiWindowFlags_NoScrollWithMouse)){

        auto vMin    = ImGui::GetWindowContentRegionMin();
        auto vMax    = ImGui::GetWindowContentRegionMax();
        auto sizeW   = ImVec2(vMax.x-vMin.x, vMax.y-vMin.y);
        float scale  = std::min(1.f*sizeW.y / texture->height(),  1.f*sizeW.x / texture->width());
        auto sizeI   = ImVec2(static_cast<int>(texture->width() * scale),static_cast<int>(texture->height() * scale));
        //        sizeI.y -= 70;

        auto uv1   = ImVec2(0,0);
        auto uv2   = ImVec2(1,1);

        auto uv3   = ImVec2(0,1);
        auto uv4   = ImVec2(1,0);

        if(ImGui::BeginTabBar(std::format("{}Tab",windowName).c_str(), ImGuiTabBarFlags_None)){
            if(ImGui::BeginTabItem(windowName.c_str())){

                if(texture->id() == 0){
                    ImGui::Text("Texture not initialized.");
                }else{
                    if(invert){
                        ImGui::Image(texture->id(), sizeI,  uv3, uv4);
                    }else{
                        ImGui::Image(texture->id(), sizeI,  uv1, uv2);
                    }
                }

                ImGui::EndTabItem();
            }
            if(ImGui::BeginTabItem("Infos")){
                ImGui::Text("pointer = %d", texture->id());
                ImGui::Text("original size = %d x %d", texture->width(), texture->height());
                ImGui::Text("window size = %d x %d", static_cast<int>(sizeI.x), static_cast<int>(sizeI.y));
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::EndChild();
}
