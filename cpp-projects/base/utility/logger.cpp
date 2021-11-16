
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

#include "logger.hpp"

// std
#include <mutex>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <format>

// local
#include "utility/time.hpp"

using namespace tool;
namespace fs = std::filesystem;

struct Logger::Impl{

    static inline std::mutex locker;
    static inline std::unique_ptr<Logger> logger = nullptr;
    static inline std::unique_ptr<std::ofstream> out = nullptr;
    static inline std::chrono::nanoseconds epochStart;
    static inline bool doFormat = false;

    static inline constexpr std::string_view startHtmlBalise           = "<p>";
    static inline constexpr std::string_view startTimestampHtmlBalise  = "<p> [";
    static inline constexpr std::string_view midTimestampHtmlBalise    = "] ";
    static inline constexpr std::string_view endHtmlBalise             = "</font></p>\n";

    static inline constexpr std::string_view htmlDarkBlueColorBalise   = "<font color=\"DarkBlue\">";
    static inline constexpr std::string_view htmlDarkOrangeColorBalise = "<font color=\"Orange\">";
    static inline constexpr std::string_view htmlDarkRedColorBalise    = "<font color=\"DarkRed\">";
    static inline constexpr std::string_view htmlDarkBlackColorBalise  = "<font color=\"Black\">";

    Impl(){
    }
};

Logger::Logger() : m_p(std::make_unique<Impl>()){
}

Logger *Logger::get(){
    if(Logger::Impl::logger != nullptr){
        return Logger::Impl::logger.get();
    }
    return nullptr;
}

void Logger::init(std::string_view logDirectoryPath, std::string_view logFileName, bool doFormat){

    Logger::Impl::doFormat = doFormat;

    if(logDirectoryPath.length() == 0 || logFileName.length() == 0){
        std::cerr << "[LOGGER-ERROR] Empty path or filename.\n";
        return;
    }

    fs::path logDirPath       = logDirectoryPath;
    fs::path logName          = logFileName;
    fs::path absoluteFilePath = logDirPath / logName;

    if(!fs::exists(logDirPath)){
        if(!fs::create_directory(logDirPath)){
            std::cerr << "[LOGGER-ERROR] Cannot create logging directory path.\n";
            return;
        }
    }

    std::unique_lock<std::mutex> lock(Logger::Impl::locker);
    if(Logger::Impl::logger == nullptr){

        if(fs::exists(absoluteFilePath)){

            auto extension = absoluteFilePath.extension();
            auto newFileName = absoluteFilePath.filename().replace_extension("");
            newFileName.concat("_previous").concat(extension.string());

            auto previousPath = absoluteFilePath;
            previousPath.replace_filename(newFileName);

            bool copyPrevious = true;
            if(fs::exists(previousPath)){
                if(!fs::remove(previousPath)){
                    std::cerr << "[LOGGER-ERROR] Cannot remove previous log file.\n";
                    copyPrevious = false;
                }
            }

            if(copyPrevious){
                fs::copy(absoluteFilePath, previousPath);
            }
        }

        // init log file
        Logger::Impl::epochStart = nanoseconds_since_epoch();
        Logger::Impl::logger     = std::make_unique<Logger>();
        Logger::Impl::out        = std::make_unique<std::ofstream>();

        Logger::Impl::out->open(absoluteFilePath);
        if(!Logger::Impl::out->is_open()){
            std::cerr << "[LOGGER-ERROR] Cannot write to log file: " << absoluteFilePath  << "\n";
        }
    }
}

void Logger::no_file_init(bool doFormat){
    Logger::Impl::epochStart = nanoseconds_since_epoch();
    Logger::Impl::logger     = std::make_unique<Logger>();
    Logger::Impl::out        = nullptr;
    Logger::Impl::doFormat   = doFormat;
}

void Logger::message(std::string_view message, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::logger == nullptr){
        std::cout << message;
        return;
    }

    if(triggersSignal){
        trigger_message(message, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageType::normal, message);
    }
}


void Logger::error(std::string_view error, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::logger == nullptr){
        std::cerr << error;
        return;
    }

    if(triggersSignal){
        trigger_error(error, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageType::error, error);
    }
}


void Logger::warning(std::string_view warning, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::logger == nullptr){
        std::cerr << warning;
        return;
    }

    if(triggersSignal){
        trigger_warning(warning, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageType::warning, warning);
    }
}

void Logger::trigger_message(std::string_view message, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::logger->message_signal(to_html_line(MessageType::normal,message));
    }else{
        Logger::Impl::logger->message_signal(std::string(message));
    }
}

void Logger::trigger_error(std::string_view error, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::logger->error_signal(to_html_line(MessageType::error, error));
    }else{
        Logger::Impl::logger->error_signal(std::string(error));
    }
}

void Logger::trigger_warning(std::string_view warning, bool htmlFormat){

    if(htmlFormat){
        Logger::Impl::logger->warning_signal(to_html_line(MessageType::warning, warning));
    }else{
        Logger::Impl::logger->warning_signal(std::string(warning));
    }
}

void Logger::status(std::string_view status, int ms){
    Logger::Impl::logger->status_signal(std::string(status), ms);
}

void Logger::progress(int state){
    Logger::Impl::logger->progress_signal(state);
}


void Logger::clean(){

    std::unique_lock<std::mutex> lock(Logger::Impl::locker);
    if(Logger::Impl::out){
        Logger::Impl::out->close();
        Logger::Impl::out = nullptr;
    }
    Logger::Impl::logger = nullptr;
}

void Logger::insert_line_to_log_file(MessageType type, std::string_view message){

    std::unique_lock<std::mutex> lock(Logger::Impl::locker);
    if(Logger::Impl::out){
        (*Logger::Impl::out) << to_html_line(type, message, true);
    }
}

std::string Logger::to_html_line(MessageType type, std::string_view text, bool addTimestamp){

    std::string_view colorCode;
    switch (type) {
    case MessageType::normal:
        colorCode = Logger::Impl::htmlDarkBlueColorBalise;
        break;
    case MessageType::warning:
        colorCode = Logger::Impl::htmlDarkOrangeColorBalise;
        break;
    case MessageType::error:
        colorCode = Logger::Impl::htmlDarkRedColorBalise;
        break;
    case MessageType::unknow:
        colorCode = Logger::Impl::htmlDarkBlackColorBalise;
        break;
    }    

    if(!addTimestamp){
        return std::format("{}{}{}{}", Logger::Impl::startHtmlBalise, colorCode, text, Logger::Impl::endHtmlBalise);
    }else{
        auto diff = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(nanoseconds_since_epoch() - Logger::Impl::epochStart).count());
        return std::format("{}{}{}{}{}{}", Logger::Impl::startTimestampHtmlBalise, diff, Logger::Impl::midTimestampHtmlBalise, colorCode, text, Logger::Impl::endHtmlBalise);
    }
}
