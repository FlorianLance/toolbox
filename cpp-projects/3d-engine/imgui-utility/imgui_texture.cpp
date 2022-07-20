

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

// base
#include "utility/format.hpp"

// local
#include "imgui-utility/imgui_std.hpp"

using namespace tool::graphics;

void ImguiTextureDrawer::draw_texture(const std::string &name, geo::Pt2f screenPos, geo::Pt2f sizeTexture, const gl::TBO *texture, bool invert){

    auto pos = ImGui::GetCursorScreenPos();

    ImGui::SetCursorScreenPos(ImGui::to_iv2(screenPos));

    if(texture->id() == 0){
        ImGui::Text(std::format("{}: texture not initialized.", name));
    }else{
        if(invert){
            ImGui::Image(texture->id(), ImGui::to_iv2(sizeTexture),  ImVec2(0,1), ImVec2(1,0));
        }else{
            ImGui::Image(texture->id(), ImGui::to_iv2(sizeTexture),  ImVec2(0,0), ImVec2(1,1));
        }
        ImGui::SetCursorScreenPos(ImGui::to_iv2(screenPos));
        ImGui::TextColored(ImVec4(1,0,0,1),name);
    }

    ImGui::SetCursorScreenPos(pos);
}


void ImguiTextureDrawer::draw_texture_tab_child(const std::string &windowName, geo::Pt2f sizeWindow, const gl::TBO *texture, bool invert){

    if(ImGui::BeginChild(std::format("{}Window",windowName).c_str(), ImGui::to_iv2(sizeWindow),false,ImGuiWindowFlags_NoScrollWithMouse)){

        if(ImGui::BeginTabBar(std::format("{}Tab",windowName).c_str(), ImGuiTabBarFlags_None)){
            if(ImGui::BeginTabItem(windowName.c_str())){

                auto size    = ImGui::content_region_size_available();
                float scale  = std::min(1.f*size.y() / texture->height(),  1.f*size.x() / texture->width());
                auto sizeI   = ImVec2(texture->width() * scale, texture->height() * scale);

                if(texture->id() == 0){
                    ImGui::Text("Texture not initialized.");
                }else{
                    if(invert){
                        ImGui::Image(texture->id(), sizeI,  ImVec2(0,1), ImVec2(1,0));
                    }else{
                        ImGui::Image(texture->id(), sizeI,  ImVec2(0,0), ImVec2(1,1));
                    }
                }
                ImGui::EndTabItem();
            }
            if(ImGui::BeginTabItem("Infos")){
                auto size    = ImGui::content_region_size_available();
                ImGui::Text("pointer = %d", texture->id());
                ImGui::Text("original size = %d x %d", texture->width(), texture->height());
                ImGui::Text("window size = %d x %d", size.x(), size.y());
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::EndChild();
}
