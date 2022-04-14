
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

// local
#include "utility/format.hpp"
#include "utility/time.hpp"

using namespace tool;
namespace fs = std::filesystem;

struct Logger::Impl{

    static inline std::mutex locker;
    static inline std::unique_ptr<Logger> logger = nullptr;
    static inline Logger *loggerPtr = nullptr;
    static inline std::unique_ptr<std::ofstream> out = nullptr;
    static inline std::chrono::nanoseconds epochStart;
    static inline bool doFormat = false;

    static inline constexpr std::string_view braces4                   = "{}{}{}{}";
    static inline constexpr std::string_view braces6                   = "{}{}{}{}{}{}";

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
    return Logger::Impl::loggerPtr;
}


std::unique_ptr<Logger, LoggerCleaner> Logger::no_file_generate(bool doFormat){

    auto l = std::unique_ptr<Logger,LoggerCleaner>(new Logger(), LoggerCleaner());

    Logger::Impl::epochStart = nanoseconds_since_epoch();
    Logger::Impl::out        = nullptr;
    Logger::Impl::doFormat   = doFormat;

    Logger::Impl::logger     = nullptr;
    Logger::Impl::loggerPtr  = l->get();

    return l;
}

bool Logger::init(std::string_view logDirectoryPath, std::string_view logFileName, bool doFormat){

    Logger::Impl::doFormat = doFormat;

    if(logDirectoryPath.length() == 0 || logFileName.length() == 0){
        std::cerr << "[LOGGER-ERROR] Empty path or filename." << std::endl;
        return false;
    }

    fs::path logDirPath       = logDirectoryPath;
    fs::path logName          = logFileName;
    fs::path absoluteFilePath = logDirPath / logName;

    if(!fs::exists(logDirPath)){
        if(!fs::create_directory(logDirPath)){
            std::cerr << "[LOGGER-ERROR] Cannot create logging directory path." << std::endl;
            return false;
        }
    }

    std::unique_lock<std::mutex> lock(Logger::Impl::locker);
    if(Logger::Impl::loggerPtr == nullptr){

        if(fs::exists(absoluteFilePath)){

            auto extension = absoluteFilePath.extension();
            auto newFileName = absoluteFilePath.filename().replace_extension("");
            newFileName.concat("_previous").concat(extension.string());

            auto previousPath = absoluteFilePath;
            previousPath.replace_filename(newFileName);

            bool copyPrevious = true;
            if(fs::exists(previousPath)){
                if(!fs::remove(previousPath)){
                    std::cerr << "[LOGGER-ERROR] Cannot remove previous log file." << std::endl;
                    copyPrevious = false;
                }
            }

            if(copyPrevious){
                fs::copy(absoluteFilePath, previousPath);
            }
        }

        // init log file
        Logger::Impl::logger     = std::make_unique<Logger>();
        Logger::Impl::loggerPtr  = Logger::Impl::logger.get();
        Logger::Impl::epochStart = nanoseconds_since_epoch();
        Logger::Impl::out        = std::make_unique<std::ofstream>();

        Logger::Impl::out->open(absoluteFilePath);
        if(!Logger::Impl::out->is_open()){
            std::cerr << "[LOGGER-ERROR] Cannot write to log file: " << absoluteFilePath  << std::endl;
            return false;
        }
    }
    return true;
}

void Logger::no_file_init(bool doFormat){
    Logger::Impl::epochStart = nanoseconds_since_epoch();
    Logger::Impl::logger     = std::make_unique<Logger>();
    Logger::Impl::loggerPtr  = Logger::Impl::logger.get();
    Logger::Impl::out        = nullptr;
    Logger::Impl::doFormat   = doFormat;
}

void Logger::message(std::string_view message, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::loggerPtr == nullptr){
        std::cout << message;
        return;
    }

    if(triggersSignal){
        trigger_message(message, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageT::normal, message);
    }
}

void Logger::warning(std::string_view warning, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::loggerPtr == nullptr){
        std::cerr << warning;
        return;
    }

    if(triggersSignal){
        trigger_warning(warning, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageT::warning, warning);
    }
}

void Logger::error(std::string_view error, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::loggerPtr == nullptr){
        std::cerr << error;
        return;
    }

    if(triggersSignal){
        trigger_error(error, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageT::error, error);
    }
}




void Logger::message_id(std::string_view message, SenderT sType, int sKey, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::loggerPtr == nullptr){
        std::cout << message;
        return;
    }

    if(triggersSignal){
        trigger_message_id(message, sType, sKey, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageT::normal, message);
    }
}


void Logger::warning_id(std::string_view warning, SenderT sType, int sKey, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::loggerPtr == nullptr){
        std::cerr << warning;
        return;
    }

    if(triggersSignal){
        trigger_warning_id(warning, sType, sKey, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageT::warning, warning);
    }
}


void Logger::error_id(std::string_view error, SenderT sType, int sKey, bool htmlFormat, bool triggersSignal, bool saveToFile){

    if(Logger::Impl::loggerPtr == nullptr){
        std::cerr << error;
        return;
    }

    if(triggersSignal){
        trigger_error_id(error, sType, sKey, htmlFormat && Logger::Impl::doFormat);
    }

    if(saveToFile && (Logger::Impl::out != nullptr)){
        insert_line_to_log_file(MessageT::error, error);
    }
}


void Logger::trigger_message(std::string_view message, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::loggerPtr->message_signal(to_html_line(MessageT::normal,message));
    }else{
        Logger::Impl::loggerPtr->message_signal(std::string(message));
    }
}

void Logger::trigger_warning(std::string_view warning, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::loggerPtr->warning_signal(to_html_line(MessageT::warning, warning));
    }else{
        Logger::Impl::loggerPtr->warning_signal(std::string(warning));
    }
}

void Logger::trigger_error(std::string_view error, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::loggerPtr->error_signal(to_html_line(MessageT::error, error));
    }else{
        Logger::Impl::loggerPtr->error_signal(std::string(error));
    }
}

void Logger::trigger_message_id(std::string_view message, SenderT sType, int sKey, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::loggerPtr->message_id_signal(to_html_line(MessageT::normal,message), sType, sKey);
    }else{
        Logger::Impl::loggerPtr->message_id_signal(std::string(message), sType, sKey);
    }
}

void Logger::trigger_warning_id(std::string_view warning, SenderT sType, int sKey, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::loggerPtr->warning_id_signal(to_html_line(MessageT::warning, warning), sType, sKey);
    }else{
        Logger::Impl::loggerPtr->warning_id_signal(std::string(warning), sType, sKey);
    }
}

void Logger::trigger_error_id(std::string_view error, SenderT sType, int sKey, bool htmlFormat){
    if(htmlFormat){
        Logger::Impl::loggerPtr->error_id_signal(to_html_line(MessageT::error, error), sType, sKey);
    }else{
        Logger::Impl::loggerPtr->error_id_signal(std::string(error), sType, sKey);
    }
}

void Logger::status(std::string_view status, int ms){
    Logger::Impl::loggerPtr->status_signal(std::string(status), ms);
}

void Logger::progress(int state){
    Logger::Impl::loggerPtr->progress_signal(state);
}


void Logger::clean(){

    std::unique_lock<std::mutex> lock(Logger::Impl::locker);
    if(Logger::Impl::out){
        Logger::Impl::out->close();
        Logger::Impl::out = nullptr;
    }
    Logger::Impl::logger = nullptr;
}

void Logger::insert_line_to_log_file(MessageT type, std::string_view message){

    std::unique_lock<std::mutex> lock(Logger::Impl::locker);
    if(Logger::Impl::out){
        (*Logger::Impl::out) << to_html_line(type, message, true) << std::flush;
    }
}

std::string Logger::to_html_line(MessageT type, std::string_view text, bool addTimestamp){

    std::string_view colorCode;
    switch (type) {
    case MessageT::normal:
        colorCode = Logger::Impl::htmlDarkBlueColorBalise;
        break;
    case MessageT::warning:
        colorCode = Logger::Impl::htmlDarkOrangeColorBalise;
        break;
    case MessageT::error:
        colorCode = Logger::Impl::htmlDarkRedColorBalise;
        break;
    case MessageT::unknow:
        colorCode = Logger::Impl::htmlDarkBlackColorBalise;
        break;
    }    

    if(!addTimestamp){
        return std::format(Logger::Impl::braces4, Logger::Impl::startHtmlBalise, colorCode, text, Logger::Impl::endHtmlBalise);
    }else{
        auto diff = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(nanoseconds_since_epoch() - Logger::Impl::epochStart).count());
        return std::format(Logger::Impl::braces6, Logger::Impl::startTimestampHtmlBalise, diff, Logger::Impl::midTimestampHtmlBalise, colorCode, text, Logger::Impl::endHtmlBalise);
    }
}

void LoggerCleaner::operator()(Logger *logger) {
    logger->clean();
}
