
/*******************************************************************************
** Toolbox-qt-utility                                                         **
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
#include "utility/tuple_array.hpp"

namespace tool::ex {

    using namespace std::literals::string_view_literals;

    enum class UiType : int {
        // simple ui
        // # bool
        Check_box, Radio_button,
        // # reals
        Spin_box, Float_spin_box, Double_spin_box, Slider_integer, Slider_double,
        // # text
        Label, List_labels, Line_edit, Text_edit,
        // # combo
        Combo_box_index,Combo_box_text,
        // # color
        Color_pick,
        // # vectors
        Vector2D, Vector3D,
        // # action
        PushButton, // Button,
        // complex ui
        Transformation, Curve,
        Notepad, Code_editor,
        Color_frame, Time, KeyboardButtonTrigger, Custom,
        // generative ui
        Tab, Generator,
        // ex_data
        Resource, ResourcesList, Component, ComponentsList, Component_config, Routine_condition,
        // others
        Non_ui_labels, Unspecified, // Camera_position,
        SizeEnum};

    using UiT  = UiType;
    using Name = std::string_view;
    using Tooltip = std::string_view;
    using Generative = bool;

    using TUiType = std::tuple<
        UiType,                 Name,                    Tooltip,                          Generative>;
    static constexpr TupleArray<UiType::SizeEnum, TUiType> uiTypes{{
        TUiType
        // simple ui
        // # bool
        {UiT::Check_box,        "Check box"sv,           "bool"sv,                         true},
        {UiT::Radio_button,     "Radio buttons"sv,       "bool"sv,                         false},
        // # reals
        {UiT::Spin_box,         "Integer spin box"sv,    "int"sv,                          true},
        {UiT::Float_spin_box,   "Float spin box"sv,      "float"sv,                        true},
        {UiT::Double_spin_box,  "Double spin box"sv,     "double"sv,                       false},
        {UiT::Slider_integer,   "Integer slider"sv,      "int"sv,                          true},
        {UiT::Slider_double,    "Float values slider"sv, "float"sv,                        true},
        // # text
        {UiT::Label,            "Label"sv,               "string"sv,                       true},        
        {UiT::List_labels,      "List of lines"sv,       "List<string>"sv,                 false},
        {UiT::Line_edit,        "Line text editor"sv,    "string"sv,                       true},
        {UiT::Text_edit,        "Block text editor"sv,   "string"sv,                       true},
        // # combo
        {UiT::Combo_box_index,  "Combo box index"sv,     "int"sv,                          true},
        {UiT::Combo_box_text,   "Combo box text"sv,      "string"sv,                       true},
        // #
        {UiT::Vector2D,         "Vector 2D"sv,           "Vector2 or List<float>"sv,       true},
        {UiT::Vector3D,         "Vector 3D"sv,           "Vector3 or List<float>"sv,       true},
        // # colors
        {UiT::Color_pick,       "Color pick"sv,          "get_color"sv,                    true},
        // # action
        {UiT::PushButton,       "Action button"sv,       "string"sv,                       false},
        //{UiT::Button,           "Action button"sv,       "string"sv,                       false},
        // complex ui
        {UiT::Transformation,   "Transformation"sv,      "Transform or List<Vector3>"sv,   true},
        {UiT::Curve,            "Curve"sv,               "List<float>"sv,                  true},
        {UiT::Notepad,          "Notepad"sv,             ""sv,                             true},
        {UiT::Code_editor,      "Code editor"sv,         "string"sv,                       true},
        {UiT::Color_frame,      "Color Frame"sv,         "bool"sv,                         false},
        {UiT::Time,             "Time"sv,                ""sv,                             false},
        {UiT::KeyboardButtonTrigger,   "KeyboardButtonTrigger"sv,      ""sv,                             false},
        {UiT::Custom,           "Custom"sv,              "string"sv,                       false},
        // # ex data
        {UiT::Resource,         "Resource"sv,            "get_resource"sv,                 true},
        {UiT::ResourcesList,    "Resources list"sv,      "get_resource_list"sv,            true},
        {UiT::Component,        "Component"sv,           "get_component"sv,                true},
        {UiT::ComponentsList,   "Components list"sv,     "get_component_list"sv,           true},
        {UiT::Component_config, "Component config"sv,    ""sv,                             true},
        {UiT::Routine_condition,"Routine condition"sv,   ""sv,                             true},
        // # generative ui
        {UiT::Tab,              "Tab"sv,                 "string"sv,                       false},        
        {UiT::Generator,        "Generator"sv,           ""sv,                             false},
        // # other
        {UiT::Non_ui_labels,    "Non ui labels"sv,       "List<string>"sv,                 false},
        {UiT::Unspecified,      "No UI"sv,               ""sv,                             false}
    }};

    constexpr std::optional<UiType> get_ui_type(Name name) {
        return uiTypes.optional_at<1,0>(name);
    }

    constexpr Name get_name(UiType t) {
        return uiTypes.at<0,1>(t);
    }

    constexpr auto get_ui_types() {
        return uiTypes.tuple_column<0>();
    }

    constexpr auto get_names() {
        return uiTypes.tuple_column<1>();
    }

    constexpr Name get_tooltip_string(UiType t) {
        return uiTypes.at<0,2>(t);
    }

    constexpr bool is_generative(UiType t){
        return uiTypes.at<0,3>(t);
    }
}

