


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

#include "ex_element.hpp"


// local
#include "utility/logger.hpp"

using namespace tool;
using namespace ex;

int ExElement::get_array_size(ParametersContainer pc, const std::string &name){
    if(contains_array(pc, name)){
        return std::get<1>(arrayContainers[pc][name]);
    }
    Logger::get()->error(fmt("get_array_size: no value with name {}", name));
    return 0;
}

void ExElement::log_message(std::string message){
    Logger::message_id(message, sender_type(), key());
}

void ExElement::log_warning(std::string warningMessage){
    Logger::warning_id(warningMessage, sender_type(), key());
}

void ExElement::log_error(std::string errorMessage){
    Logger::error_id(errorMessage, sender_type(), key());
}

void ExElement::stack_trace_log(std::string stackTraceMessage){
    (*stackTraceCBP)(stackTraceMessage.c_str());
}

void ExElement::next(){
    (*nextCBP)();
}

void ExElement::previous(){
    (*previousCBP)();
}

void ExElement::close(int key){
    (*closeCBP)(key);
}

int ExElement::component_key(std::string componentName){
    return (*getCBP)(componentName.c_str());
}

void ExElement::init_callbacks(
    LogMessageCB logMessageCB,
    LogWarningCB logWarningCB,
    LogErrorCB logErrorCB,
    LogMessageIdCB logMessageIdCB,
    LogWarningIdCB logWarningIdCB,
    LogErrorIdCB logErrorIdCB,
    StackTraceCB stackTraceCB,
    EllapsedTimeExpMsCB ellapsedTimeExpMsCB,
    EllapsedTimeRoutineMsCB ellapsedTimeRoutineMsCB,
    GetCB getCB,
    IsVisibleCB isVisibleCB,
    IsUpdatingCB isUpdatingCB,
    IsClosedCB isClosedCB,
    NextCB nextCB,
    PreviousCB previousCB,
    CloseCB closeCB,
    SignalBoolCB signalBoolCB,
    SignalIntCB signalIntCB,
    SignalFloatCB signalFloatCB,
    SignalDoubleCB signalDoubleCB,
    SignalStringCB signalStringCB){

    logMessageCBP             = std::make_unique<LogMessageCB>(logMessageCB);
    logWarningCBP             = std::make_unique<LogWarningCB>(logWarningCB);
    logErrorCBP               = std::make_unique<LogErrorCB>(logErrorCB);

    logMessageIdCBP           = std::make_unique<LogMessageIdCB>(logMessageIdCB);
    logWarningIdCBP           = std::make_unique<LogWarningIdCB>(logWarningIdCB);
    logErrorIdCBP             = std::make_unique<LogErrorIdCB>(logErrorIdCB);
    stackTraceCBP             = std::make_unique<StackTraceCB>(stackTraceCB);

    ellapsedTimeExpMsCBP      = std::make_unique<EllapsedTimeExpMsCB>(ellapsedTimeExpMsCB);
    ellapsedTimeRoutineMsCBP  = std::make_unique<EllapsedTimeRoutineMsCB>(ellapsedTimeRoutineMsCB);

    getCBP                    = std::make_unique<GetCB>(getCB);
    isVisibleCBP              = std::make_unique<IsVisibleCB>(isVisibleCB);
    isUpdatingCBP             = std::make_unique<IsUpdatingCB>(isUpdatingCB);
    isClosedCBP               = std::make_unique<IsClosedCB>(isClosedCB);

    nextCBP                   = std::make_unique<NextCB>(nextCB);
    previousCBP               = std::make_unique<PreviousCB>(previousCB);
    closeCBP                  = std::make_unique<CloseCB>(closeCB);

    signalBoolCBP             = std::make_unique<SignalBoolCB>(signalBoolCB);
    signalIntCBP              = std::make_unique<SignalIntCB>(signalIntCB);
    signalFloatCBP            = std::make_unique<SignalFloatCB>(signalFloatCB);
    signalDoubleCBP           = std::make_unique<SignalDoubleCB>(signalDoubleCB);
    signalStringCBP           = std::make_unique<SignalStringCB>(signalStringCB);


    Logger::get()->message_signal.connect([&](std::string message){
        (*logMessageCBP)(message.c_str());
    });
    Logger::get()->warning_signal.connect([&](std::string warning){
        (*logWarningCBP)(warning.c_str());
    });
    Logger::get()->error_signal.connect([&](std::string error){
        (*logErrorCBP)(error.c_str());
    });

    Logger::get()->message_id_signal.connect([&](std::string message, Logger::SenderT sType, int sKey){
        (*logMessageIdCBP)(message.c_str(), static_cast<int>(sType), sKey);
    });
    Logger::get()->warning_id_signal.connect([&](std::string warning, Logger::SenderT sType, int sKey){
        (*logWarningIdCBP)(warning.c_str(), static_cast<int>(sType), sKey);
    });
    Logger::get()->error_id_signal.connect([&](std::string error, Logger::SenderT sType, int sKey){
        (*logErrorIdCBP)(error.c_str(), static_cast<int>(sType), sKey);
    });

    Logger::get()->message("test1");
    Logger::message("test2");
    Logger::get()->warning("wtest1");
    Logger::warning("wtest2");
    Logger::get()->error("etest1");
    Logger::error("etest2");
}
