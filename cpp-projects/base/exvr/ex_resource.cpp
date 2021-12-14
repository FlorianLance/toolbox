


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

#include "ex_resource.hpp"

using namespace tool::ex;


int ExResource::get_array_size(const std::string &name){
    std::unique_lock<std::mutex> lock(containerLocker);
    if(dynamicArray.count(name) != 0){
        return std::get<1>(dynamicArray[name]);
    }
    return 0;
}

void ExResource::log_warning(std::string warningMessage){
    if(logWarningCB){
        (*logWarningCB)(warningMessage.c_str());
    }else{
        std::cerr << warningMessage << "\n";
    }
}

void ExResource::log_error(std::string errorMessage){
    if(logErrorCB){
        (*logErrorCB)(errorMessage.c_str());
    }else{
        std::cerr << errorMessage << "\n";
    }
}

void ExResource::log(std::string message){
    if(logCB){
        (*logCB)(message.c_str());
    }else{
        std::cout << message << "\n";
    }
}

void ExResource::stack_trace_log(std::string stackTraceMessage){
    if(stackTraceCB){
        (*stackTraceCB)(stackTraceMessage.c_str());
    }
}
