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

class Keyboard{

public:

    enum class Button : int {
        Backspace = 0,
        Tab,
        Clear,
        Return,
        Pause,
        Escape,
        Space,
        Exclaim,
        DoubleQuote,
        Hash,
        Dollar,
        Percent,
        Ampersand,
        Quote,
        LeftParen,
        RightParen,
        Asterisk,
        Plus,
        Comma,
        Minus,
        Period,
        Slash,
        Alpha0,
        Alpha1,
        Alpha2,
        Alpha3,
        Alpha4,
        Alpha5,
        Alpha6,
        Alpha7,
        Alpha8,
        Alpha9,
        Colon,
        Semicolon,
        Less,
        Equals,
        Greater,
        Question,
        At,
        LeftBracket,
        Backslash,
        RightBracket,
        Caret,
        Underscore,
        BackQuote,
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        M,
        N,
        O,
        P,
        Q,
        R,
        S,
        T,
        U,
        V,
        W,
        X,
        Y,
        Z,
        LeftCurlyBracket,
        Pipe,
        RightCurlyBracket,
        Tilde,
        Delete,
        Keypad0,
        Keypad1,
        Keypad2,
        Keypad3,
        Keypad4,
        Keypad5,
        Keypad6,
        Keypad7,
        Keypad8,
        Keypad9,
        KeypadPeriod,
        KeypadDivide,
        KeypadMultiply,
        KeypadMinus,
        KeypadPlus,
        KeypadEnter,
        KeypadEquals,
        UpArrow,
        DownArrow,
        RightArrow,
        LeftArrow,
        Insert,
        Home,
        End,
        PageUp,
        PageDown,
        F1,
        F2,
        F3,
        F4,
        F5,
        F6,
        F7,
        F8,
        F9,
        F10,
        F11,
        F12,
        F13,
        F14,
        F15,
        Numlock,
        CapsLock,
        ScrollLock,
        RightShift,
        LeftShift,
        RightControl,
        LeftControl,
        RightAlt,
        LeftAlt,
        RightCommand,
        RightApple,
        LeftCommand,
        LeftApple,
        LeftWindows,
        RightWindows,
        AltGr,
        Help,
        Print,
        SysReq,
        Break,
        Menu,
        SizeEnum
    };


    using Code = int;
    using Name = std::string_view;
    using TButton = std::tuple<
                Button,                                  Name,                 Code>;
    static constexpr TupleArray<Button::SizeEnum,TButton> buttons ={{
        TButton{Button::Backspace                       ,"Backspace"sv        ,8},
        TButton{Button::Tab                             ,"Tab"sv              ,9},
        TButton{Button::Clear                           ,"Clear"sv            ,12},
        TButton{Button::Return                          ,"Return"sv           ,13},
        TButton{Button::Pause                           ,"Pause"sv            ,19},
        TButton{Button::Escape                          ,"Escape"sv           ,27},
        TButton{Button::Space                           ,"Space"sv            ,32},
        TButton{Button::Alpha0                          ,"Alpha0"sv           ,48},
        TButton{Button::Alpha1                          ,"Alpha1"sv           ,49},
        TButton{Button::Alpha2                          ,"Alpha2"sv           ,50},
        TButton{Button::Alpha3                          ,"Alpha3"sv           ,51},
        TButton{Button::Alpha4                          ,"Alpha4"sv           ,52},
        TButton{Button::Alpha5                          ,"Alpha5"sv           ,53},
        TButton{Button::Alpha6                          ,"Alpha6"sv           ,54},
        TButton{Button::Alpha7                          ,"Alpha7"sv           ,55},
        TButton{Button::Alpha8                          ,"Alpha8"sv           ,56},
        TButton{Button::Alpha9                          ,"Alpha9"sv           ,57},
        TButton{Button::Exclaim                         ,"Exclaim"sv          ,33},
        TButton{Button::DoubleQuote                     ,"DoubleQuote"sv      ,34},
        TButton{Button::Hash                            ,"Hash"sv             ,35},
        TButton{Button::Dollar                          ,"Dollar"sv           ,36},
        TButton{Button::Percent                         ,"Percent"sv          ,37},
        TButton{Button::Ampersand                       ,"Ampersand"sv        ,38},
        TButton{Button::Quote                           ,"Quote"sv            ,39},
        TButton{Button::LeftParen                       ,"LeftParen"sv        ,40},
        TButton{Button::RightParen                      ,"RightParen"sv       ,41},
        TButton{Button::Asterisk                        ,"Asterisk"sv         ,42},
        TButton{Button::Plus                            ,"Plus"sv             ,43},
        TButton{Button::Comma                           ,"Comma"sv            ,44},
        TButton{Button::Minus                           ,"Minus"sv            ,45},
        TButton{Button::Period                          ,"Period"sv           ,46},
        TButton{Button::Slash                           ,"Slash"sv            ,47},
        TButton{Button::Colon                           ,"Colon"sv            ,58},
        TButton{Button::Semicolon                       ,"Semicolon"sv        ,59},
        TButton{Button::Less                            ,"Less"sv             ,60},
        TButton{Button::Equals                          ,"Equals"sv           ,61},
        TButton{Button::Greater                         ,"Greater"sv          ,62},
        TButton{Button::Question                        ,"Question"sv         ,63},
        TButton{Button::At                              ,"At"sv               ,64},
        TButton{Button::LeftBracket                     ,"LeftBracket"sv      ,91},
        TButton{Button::Backslash                       ,"Backslash"sv        ,92},
        TButton{Button::RightBracket                    ,"RightBracket"sv     ,93},
        TButton{Button::Caret                           ,"Caret"sv            ,94},
        TButton{Button::Underscore                      ,"Underscore"sv       ,95},
        TButton{Button::BackQuote                       ,"BackQuote"sv        ,96},
        TButton{Button::A                               ,"A"sv                ,97},
        TButton{Button::B                               ,"B"sv                ,98},
        TButton{Button::C                               ,"C"sv                ,99},
        TButton{Button::D                               ,"D"sv                ,100},
        TButton{Button::E                               ,"E"sv                ,101},
        TButton{Button::F                               ,"F"sv                ,102},
        TButton{Button::G                               ,"G"sv                ,103},
        TButton{Button::H                               ,"H"sv                ,104},
        TButton{Button::I                               ,"I"sv                ,105},
        TButton{Button::J                               ,"J"sv                ,106},
        TButton{Button::K                               ,"K"sv                ,107},
        TButton{Button::L                               ,"L"sv                ,108},
        TButton{Button::M                               ,"M"sv                ,109},
        TButton{Button::N                               ,"N"sv                ,110},
        TButton{Button::O                               ,"O"sv                ,111},
        TButton{Button::P                               ,"P"sv                ,112},
        TButton{Button::Q                               ,"Q"sv                ,113},
        TButton{Button::R                               ,"R"sv                ,114},
        TButton{Button::S                               ,"S"sv                ,115},
        TButton{Button::T                               ,"T"sv                ,116},
        TButton{Button::U                               ,"U"sv                ,117},
        TButton{Button::V                               ,"V"sv                ,118},
        TButton{Button::W                               ,"W"sv                ,119},
        TButton{Button::X                               ,"X"sv                ,120},
        TButton{Button::Y                               ,"Y"sv                ,121},
        TButton{Button::Z                               ,"Z"sv                ,122},
        TButton{Button::LeftCurlyBracket                ,"LeftCurlyBracket"sv ,123},
        TButton{Button::Pipe                            ,"Pipe"sv             ,124},
        TButton{Button::RightCurlyBracket               ,"RightCurlyBracket"sv,125},
        TButton{Button::Tilde                           ,"Tilde"sv            ,126},
        TButton{Button::Delete                          ,"Delete"sv           ,127},
        TButton{Button::Keypad0                         ,"Keypad0"sv          ,256},
        TButton{Button::Keypad1                         ,"Keypad1"sv          ,257},
        TButton{Button::Keypad2                         ,"Keypad2"sv          ,258},
        TButton{Button::Keypad3                         ,"Keypad3"sv          ,259},
        TButton{Button::Keypad4                         ,"Keypad4"sv          ,260},
        TButton{Button::Keypad5                         ,"Keypad5"sv          ,261},
        TButton{Button::Keypad6                         ,"Keypad6"sv          ,262},
        TButton{Button::Keypad7                         ,"Keypad7"sv          ,263},
        TButton{Button::Keypad8                         ,"Keypad8"sv          ,264},
        TButton{Button::Keypad9                         ,"Keypad9"sv          ,265},
        TButton{Button::KeypadPeriod                    ,"KeypadPeriod"sv     ,266},
        TButton{Button::KeypadDivide                    ,"KeypadDivide"sv     ,267},
        TButton{Button::KeypadMultiply                  ,"KeypadMultiply"sv   ,268},
        TButton{Button::KeypadMinus                     ,"KeypadMinus"sv      ,269},
        TButton{Button::KeypadPlus                      ,"KeypadPlus"sv       ,270},
        TButton{Button::KeypadEnter                     ,"KeypadEnter"sv      ,271},
        TButton{Button::KeypadEquals                    ,"KeypadEquals"sv     ,272},
        TButton{Button::UpArrow                         ,"UpArrow"sv          ,273},
        TButton{Button::DownArrow                       ,"DownArrow"sv        ,274},
        TButton{Button::RightArrow                      ,"RightArrow"sv       ,275},
        TButton{Button::LeftArrow                       ,"LeftArrow"sv        ,276},
        TButton{Button::Insert                          ,"Insert"sv           ,277},
        TButton{Button::Home                            ,"Home"sv             ,278},
        TButton{Button::End                             ,"End"sv              ,279},
        TButton{Button::PageUp                          ,"PageUp"sv           ,280},
        TButton{Button::PageDown                        ,"PageDown"sv         ,281},
        TButton{Button::F1                              ,"F1"sv               ,282},
        TButton{Button::F2                              ,"F2"sv               ,283},
        TButton{Button::F3                              ,"F3"sv               ,284},
        TButton{Button::F4                              ,"F4"sv               ,285},
        TButton{Button::F5                              ,"F5"sv               ,286},
        TButton{Button::F6                              ,"F6"sv               ,287},
        TButton{Button::F7                              ,"F7"sv               ,288},
        TButton{Button::F8                              ,"F8"sv               ,289},
        TButton{Button::F9                              ,"F9"sv               ,290},
        TButton{Button::F10                             ,"F10"sv              ,291},
        TButton{Button::F11                             ,"F11"sv              ,292},
        TButton{Button::F12                             ,"F12"sv              ,293},
        TButton{Button::F13                             ,"F13"sv              ,294},
        TButton{Button::F14                             ,"F14"sv              ,295},
        TButton{Button::F15                             ,"F15"sv              ,296},
        TButton{Button::Numlock                         ,"Numlock"sv          ,300},
        TButton{Button::CapsLock                        ,"CapsLock"sv         ,301},
        TButton{Button::ScrollLock                      ,"ScrollLock"sv       ,302},
        TButton{Button::RightShift                      ,"RightShift"sv       ,303},
        TButton{Button::LeftShift                       ,"LeftShift"sv        ,304},
        TButton{Button::RightControl                    ,"RightControl"sv     ,305},
        TButton{Button::LeftControl                     ,"LeftControl"sv      ,306},
        TButton{Button::RightAlt                        ,"RightAlt"sv         ,307},
        TButton{Button::LeftAlt                         ,"LeftAlt"sv          ,308},
        TButton{Button::LeftCommand                     ,"LeftCommand"sv      ,309},
        TButton{Button::LeftApple                       ,"LeftApple"sv        ,309},
        TButton{Button::LeftWindows                     ,"LeftWindows"sv      ,310},
        TButton{Button::RightCommand                    ,"RightCommand"sv     ,310},
        TButton{Button::RightApple                      ,"RightApple"sv       ,311},
        TButton{Button::RightWindows                    ,"RightWindows"sv     ,312},
        TButton{Button::AltGr                           ,"AltGr"sv            ,313},
        TButton{Button::Help                            ,"Help"sv             ,315},
        TButton{Button::Print                           ,"Print"sv            ,316},
        TButton{Button::SysReq                          ,"SysReq"sv           ,317},
        TButton{Button::Break                           ,"Break"sv            ,318},
        TButton{Button::Menu                            ,"Menu"sv             ,319}
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
};
}
