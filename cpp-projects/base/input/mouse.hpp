
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
#include <string_view>

// base
#include "utility/tuple_array.hpp"

namespace tool::input {

using namespace std::literals::string_view_literals;

class Mouse{


public:
    enum class Button : int {
        Mouse0=0,
        Mouse1,
        Mouse2,
        Mouse3,
        Mouse4,
        Mouse5,
        Mouse6,
        SizeEnum
    };

    enum class Axis : int {
        Horizonal = 0, Vertical,
        // counter
        SizeEnum
    };


    using Code = int;
    using Name = std::string_view;
    using TButton = std::tuple<Button, Name, Code>;
    static constexpr TupleArray<Button::SizeEnum, TButton> buttons{{
        TButton{Button::Mouse0, "Mouse0"sv, 323},
        TButton{Button::Mouse1, "Mouse1"sv, 324},
        TButton{Button::Mouse2, "Mouse2"sv, 325},
        TButton{Button::Mouse3, "Mouse3"sv, 326},
        TButton{Button::Mouse4, "Mouse4"sv, 327},
        TButton{Button::Mouse5, "Mouse5"sv, 328},
        TButton{Button::Mouse6, "Mouse6"sv, 329}
    }};


    static constexpr Name get_name(Button button) {
        return buttons.at<0,1>(button);
    }

    static constexpr Code get_code(Button button) {
        return buttons.at<0,2>(button);
    }

    static constexpr std::optional<Name> get_name(Code code) {
        return buttons.optional_at<2,1>(code);
    }

    static constexpr std::optional<Code> get_code(Name name) {
        return buttons.optional_at<1,2>(name);
    }

    static constexpr std::optional<Button> get_button(Code code) {
        return buttons.optional_at<2,0>(code);
    }

    static constexpr std::optional<Button> get_button(Name name) {
        return buttons.optional_at<1,0>(name);
    }

    using TAxes = std::tuple<
            Axis,               Name>;
    static constexpr TupleArray<Axis::SizeEnum, TAxes> axes ={{
        TAxes
        {Axis::Horizonal,       "Horizontal"sv},
        {Axis::Vertical,        "Vertical"sv},
    }};

    static constexpr Name get_name(Axis axis) {
        return axes.at<0,1>(axis);
    }

    static constexpr std::optional<Axis> get_axis(Name name) {
        return axes.optional_at<1,0>(name);
    }

};
}
