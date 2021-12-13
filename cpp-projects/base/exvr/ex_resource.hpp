

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
#include <iostream>

// local
#include "ex_utility.hpp"

namespace tool::ex {

class ExResource{

public:

    void log_warning(std::string warningMessage){
        if(logWarningCB){
            (*logWarningCB)(warningMessage.c_str());
        }else{
            std::cerr << warningMessage << "\n";
        }
    }

    void log_error(std::string errorMessage){
        if(logErrorCB){
            (*logErrorCB)(errorMessage.c_str());
        }else{
            std::cerr << errorMessage << "\n";
        }
    }

    void log(std::string message){
        if(logCB){
            (*logCB)(message.c_str());
        }else{
            std::cout << message << "\n";
        }
    }

    void stack_trace_log(std::string stackTraceMessage){
        if(strackTraceCB){
            (*strackTraceCB)(stackTraceMessage.c_str());
        }
    }


    std::unique_ptr<StrackTraceCB> strackTraceCB = nullptr;
    std::unique_ptr<LogCB> logCB = nullptr;
    std::unique_ptr<LogWarningCB> logWarningCB = nullptr;
    std::unique_ptr<LogErrorCB> logErrorCB= nullptr;
};
}
