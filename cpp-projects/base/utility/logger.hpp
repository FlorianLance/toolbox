

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
#include <memory>
#include <string>

// lsignal
#include "lsignal.h"

namespace tool {

class Logger{

public:

    enum class MessageT : int{
        normal, warning, error, unknow,
    };

    enum class SenderT : int {
        GUI, Component, Resource
    };

    Logger();

    static Logger *get();

    static bool init(std::string_view logDirectoryPath = "", std::string_view logFileName = "default_log.html", bool doFormat = false);
    static void no_file_init(bool doFormat = false);

    static void message(std::string_view message, bool htmlFormat = true, bool triggersSignal = true, bool saveToFile = true);
    static void error(std::string_view error, bool htmlFormat = true, bool triggersSignal = true, bool saveToFile = true);
    static void warning(std::string_view warning, bool htmlFormat = true, bool triggersSignal = true, bool saveToFile = true);

    static void message_id(std::string_view message, SenderT sType, int sKey, bool htmlFormat = true, bool triggersSignal = true, bool saveToFile = true);
    static void error_id(std::string_view error, SenderT sType, int sKey, bool htmlFormat = true, bool triggersSignal = true, bool saveToFile = true);
    static void warning_id(std::string_view warning, SenderT sType, int sKey, bool htmlFormat = true, bool triggersSignal = true, bool saveToFile = true);

    static void status(std::string_view status, int ms = 0);
    static void progress(int state);

    static void clean();

// signals
    lsignal::signal<void(std::string message)> message_signal;    
    lsignal::signal<void(std::string warning)> warning_signal;
    lsignal::signal<void(std::string error)> error_signal;

    lsignal::signal<void(std::string message, SenderT sType, int sKey)> message_id_signal;
    lsignal::signal<void(std::string warning, SenderT sType, int sKey)> warning_id_signal;
    lsignal::signal<void(std::string error, SenderT sType, int sKey)> error_id_signal;

    lsignal::signal<void(std::string status, int ms)> status_signal;
    lsignal::signal<void(int state)> progress_signal;

private:

    static void trigger_message(std::string_view message, bool htmlFormat = true);
    static void trigger_error(std::string_view error, bool htmlFormat = true);
    static void trigger_warning(std::string_view warning, bool htmlFormat = true);

    static void trigger_message_id(std::string_view message, SenderT sType, int sKey, bool htmlFormat = true);
    static void trigger_error_id(std::string_view error, SenderT sType, int sKey, bool htmlFormat = true);
    static void trigger_warning_id(std::string_view warning, SenderT sType, int sKey, bool htmlFormat = true);

    static std::string to_html_line(MessageT type, std::string_view text, bool addTimestamp = false);
    static void insert_line_to_log_file(MessageT type, std::string_view message);

    struct Impl;
    std::unique_ptr<Impl> m_p = nullptr;
};
}

