


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



using namespace tool;
using namespace ex;

int ExElement::get_array_size(ParametersContainer pc, const std::string &name){
    if(contains_array(pc, name)){
        return std::get<1>(arrayContainers[pc][name]);
    }
    Logger::get()->error(std::format("get_array_size: no value with name {}", name));
    return 0;
}

void ExElement::log_message(std::string message){
    (*exp->logMessageIdCBP)(message.c_str(), (int)sender_type(), key());
    exp->logger->message_id(message, sender_type(), key());
}

void ExElement::log_warning(std::string warningMessage){
    (*exp->logWarningIdCBP)(warningMessage.c_str(), (int)sender_type(), key());
    exp->logger->warning_id(warningMessage, sender_type(), key());
}

void ExElement::log_error(std::string errorMessage){
    (*exp->logErrorIdCBP)(errorMessage.c_str(), (int)sender_type(), key());
    exp->logger->error_id(errorMessage, sender_type(), key());
}

void ExElement::stack_trace_log(std::string stackTraceMessage){
    (*exp->stackTraceCBP)(stackTraceMessage.c_str());
}

void ExElement::next(){
    (*exp->nextCBP)();
}

void ExElement::previous(){
    (*exp->previousCBP)();
}

void ExElement::close(int key){
    (*exp->closeCBP)(key);
}

int ExElement::component_key(std::string componentName){
    return (*exp->getCBP)(componentName.c_str());
}


