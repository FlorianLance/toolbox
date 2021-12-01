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

// std
#include <array>
#include <tuple>
#include <string_view>

// base
#include "utility/tuple_array.hpp"

namespace tool::input {

using namespace std::literals::string_view_literals;

class Joypad{

public:
    enum class Button : int {
        // P1
        A_P1 = 0, B_P1, X_P1, Y_P1,
        RightBumper_P1, LeftBumper_P1, Start_P1, Back_P1,
        LeftStickClick_P1, RightStickClick_P1,
        // P2
        A_P2, B_P2, X_P2, Y_P2,
        RightBumper_P2, LeftBumper_P2, Start_P2, Back_P2,
        LeftStickClick_P2, RightStickClick_P2,
        // counter
        SizeEnum
    };

    enum class Axis : int {
        // P1
        LeftJoystickX_P1 = 0, RightJoystickX_P1,
        LeftJoystickY_P1, RightJoystickY_P1,
        LeftTrigger_P1, RightTrigger_P1,
        DPadX_P1, DPadY_P1,
        // P2
        LeftJoystickX_P2, RightJoystickX_P2,
        LeftJoystickY_P2, RightJoystickY_P2,
        LeftTrigger_P2, RightTrigger_P2,
        DPadX_P2, DPadY_P2,
        // counter
        SizeEnum
    };

    using Code = int;
    using Player = int;
    using Name = std::string_view;
    using TButton = std::tuple<
        Button,                         Name,                          Code,    Player>;
    static constexpr TupleArray<Button::SizeEnum, TButton> buttons ={{
        TButton
        {Button::A_P1                   ,"A_P1"sv                     ,0        , 1},
        {Button::B_P1                   ,"B_P1"sv                     ,1        , 1},
        {Button::X_P1                   ,"X_P1"sv                     ,2        , 1},
        {Button::Y_P1                   ,"Y_P1"sv                     ,3        , 1},
        {Button::RightBumper_P1         ,"RightBumper_P1"sv           ,4        , 1},
        {Button::LeftBumper_P1          ,"LeftBumper_P1"sv            ,5        , 1},
        {Button::Start_P1               ,"Start_P1"sv                 ,6        , 1},
        {Button::Back_P1                ,"Back_P1"sv                  ,7        , 1},
        {Button::LeftStickClick_P1      ,"LeftStickClick_P1"sv        ,8        , 1},
        {Button::RightStickClick_P1     ,"RightStickClick_P1"sv       ,9        , 1},
        {Button::A_P2                   ,"A_P2"sv                     ,10       , 2},
        {Button::B_P2                   ,"B_P2"sv                     ,11       , 2},
        {Button::X_P2                   ,"X_P2"sv                     ,12       , 2},
        {Button::Y_P2                   ,"Y_P2"sv                     ,13       , 2},
        {Button::RightBumper_P2         ,"RightBumper_P2"sv           ,14       , 2},
        {Button::LeftBumper_P2          ,"LeftBumper_P2"sv            ,15       , 2},
        {Button::Start_P2               ,"Start_P2"sv                 ,16       , 2},
        {Button::Back_P2                ,"Back_P2"sv                  ,17       , 2},
        {Button::LeftStickClick_P2      ,"LeftStickClick_P2"sv        ,18       , 2},
        {Button::RightStickClick_P2     ,"RightStickClick_P2"sv       ,19       , 2},
    }};

    static constexpr Name get_name(Button button) {
        return buttons.at<0,1>(button);
    }

    static constexpr Code get_code(Button button) {
        return buttons.at<0,2>(button);
    }

    static constexpr Player get_player(Button button) {
        return buttons.at<0,3>(button);
    }

    static constexpr std::optional<Name> get_button_name(Code code) {
        return buttons.optional_at<2,1>(code);
    }

    static constexpr std::optional<Code> get_button_code(Name name) {
        return buttons.optional_at<1,2>(name);
    }

    static constexpr std::optional<Button> get_button(Code code) {
        return buttons.optional_at<2,0>(code);
    }

    static constexpr std::optional<Button> get_button(Name name) {
        return buttons.optional_at<1,0>(name);
    }

    static std::vector<Button> buttons_from_player(Player player) {
        return buttons.elements_matching_columns_values<3, 0>(player);
    }

    using TAxes = std::tuple<
        Axis,                         Name,                    Code,    Player>;
    static constexpr TupleArray<Axis::SizeEnum, TAxes> axes ={{
        TAxes
        {Axis::LeftJoystickX_P1     ,"LeftJoystickX_P1"sv     ,100,     1},
        {Axis::RightJoystickX_P1    ,"RightJoystickX_P1"sv    ,101,     1},
        {Axis::LeftJoystickY_P1     ,"LeftJoystickY_P1"sv     ,102,     1},
        {Axis::RightJoystickY_P1    ,"RightJoystickY_P1"sv    ,103,     1},
        {Axis::LeftTrigger_P1       ,"LeftTrigger_P1"sv       ,104,     1},
        {Axis::RightTrigger_P1      ,"RightTrigger_P1"sv      ,105,     1},
        {Axis::DPadX_P1             ,"DPadX_P1"sv             ,106,     1},
        {Axis::DPadY_P1             ,"DPadY_P1"sv             ,107,     1},
        {Axis::LeftJoystickX_P2     ,"LeftJoystickX_P2"sv     ,108,     2},
        {Axis::RightJoystickX_P2    ,"RightJoystickX_P2"sv    ,109,     2},
        {Axis::LeftJoystickY_P2     ,"LeftJoystickY_P2"sv     ,110,     2},
        {Axis::RightJoystickY_P2    ,"RightJoystickY_P2"sv    ,111,     2},
        {Axis::LeftTrigger_P2       ,"LeftTrigger_P2"sv       ,112,     2},
        {Axis::RightTrigger_P2      ,"RightTrigger_P2"sv      ,113,     2},
        {Axis::DPadX_P2             ,"DPadX_P2"sv             ,114,     2},
        {Axis::DPadY_P2             ,"DPadY_P2"sv             ,115,     2},
    }};

    static constexpr Name get_name(Axis axis) {
        return axes.at<0,1>(axis);
    }

    static constexpr Code get_code(Axis axis) {
        return axes.at<0,2>(axis);
    }

    static constexpr Player get_player(Axis axis) {
        return axes.at<0,3>(axis);
    }

    static constexpr std::optional<Name> get_axis_name(Code code) {
        return axes.optional_at<2,1>(code);
    }

    static constexpr std::optional<Code> get_axis_code(Name name) {
        return axes.optional_at<1,2>(name);
    }

    static constexpr std::optional<Axis> get_axis(Code code) {
        return axes.optional_at<2,0>(code);
    }

    static constexpr std::optional<Axis> get_axis(Name name) {
        return axes.optional_at<1,0>(name);
    }

    static std::vector<Axis> axes_from_player(Player player) {
        return axes.elements_matching_columns_values<3, 0>(player);
    }
};
}
