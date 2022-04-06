

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

// std
#include <vector>
#include <string>

// imgui
#include "imgui/imgui.h"

// base
#include "geometry/point2.hpp"

namespace ImGui{

[[maybe_unused]] static ImVec2 to_iv2(tool::geo::Pt2<int> p){
    return ImVec2(static_cast<float>(p.x()),static_cast<float>(p.y()));
}
[[maybe_unused]] static ImVec2 to_iv2(tool::geo::Pt2f p){
    return ImVec2(p.x(),p.y());
}
[[maybe_unused]] static tool::geo::Pt2f to_pt2(ImVec2 v){
    return {v.x,v.y};
}

/**
 * @brief get current window position in screen space
 */
[[maybe_unused]] static tool::geo::Pt2f window_screen_pos(){
    return to_pt2(ImGui::GetWindowPos());
}

/**
 * @brief get current window size
 */
[[maybe_unused]] static tool::geo::Pt2f window_size(){
    return to_pt2(ImGui::GetWindowSize());
}

[[maybe_unused]] static tool::geo::Pt2f item_size(){
    return to_pt2(ImGui::GetItemRectSize());
}

[[maybe_unused]] static tool::geo::Pt2f last_item_top_left_screen_pos(){
    return to_pt2(ImGui::GetItemRectMin());
}

[[maybe_unused]] static tool::geo::Pt2f last_item_bottom_right_screen_pos(){
    return to_pt2(ImGui::GetItemRectMax());
}

[[maybe_unused]] static tool::geo::Pt2f last_item_bottom_left_screen_pos(){
    return last_item_top_left_screen_pos() + tool::geo::Pt2f{0, item_size().y()};
}

[[maybe_unused]] static float last_item_top_screen_value(){
    return last_item_top_left_screen_pos().y();
}

[[maybe_unused]] static float last_item_bottom_screen_value(){
    return last_item_bottom_right_screen_pos().y();
}

[[maybe_unused]] static float last_item_left_screen_value(){
    return last_item_top_left_screen_pos().x();
}

[[maybe_unused]] static float last_item_right_screen_value(){
    return last_item_bottom_right_screen_pos().x();
}

[[maybe_unused]] static tool::geo::Pt2f content_region_size_available(){
    return to_pt2(ImGui::GetContentRegionAvail());
}

[[maybe_unused]] static tool::geo::Pt2f cursor_window_position(){
    return to_pt2(ImGui::GetCursorPos());
}

[[maybe_unused]] static tool::geo::Pt2f cursor_screen_position(){
    return to_pt2(ImGui::GetCursorScreenPos());
}

static auto vector_getter = [](void* vec, int idx, const char** out_text){
    auto& vector = *static_cast<std::vector<std::string>*>(vec);
    if (idx < 0 || idx >= static_cast<int>(vector.size())) {
        return false;
    }
    *out_text = vector.at(idx).c_str();
    return true;
};

[[maybe_unused]] static bool Combo(const char* label, int* currIndex, std::vector<std::string>& values){
    if (values.empty()) {
        return false;
    }
    return Combo(label, currIndex, vector_getter,static_cast<void*>(&values), static_cast<int>(values.size()));
}

[[maybe_unused]] static bool ListBox(const char* label, int* currIndex, std::vector<std::string>& values){
    if (values.empty()) {
        return false;
    }
    return ListBox(label, currIndex, vector_getter,static_cast<void*>(&values), static_cast<int>(values.size()));
}

[[maybe_unused]] static void Text(const std::string &text){
    auto d = text.c_str();
    ImGui::Text(d, d + text.size());
}

[[maybe_unused]] static void TextColored(const ImVec4& col, const std::string &text){
    auto d = text.c_str();
    ImGui::TextColored(col, d, d + text.size());
}

[[maybe_unused]] static void TextCenter(const char* text, ...) {

    va_list vaList = nullptr;
    va_start(vaList, text);

    float font_size = ImGui::GetFontSize() * strlen(text) / 2;
    ImGui::SameLine(
        ImGui::GetWindowSize().x / 2 -
        font_size + (font_size / 2)
    );

    ImGui::TextV(text, vaList);

    va_end(vaList);
}


}
