

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
#include <mutex>
#include <map>
#include <any>
#include <format>

// local
#include "ex_utility.hpp"

namespace tool::ex {

class ExResource{

public:

    virtual ~ExResource(){}

    virtual bool initialize(){return true;}
    virtual void clean(){}

    template<typename T>
    void update_parameter(const std::string &name, T value){
        std::lock_guard<std::mutex> lock(containerLocker);
        dynamic[name] = std::make_any<T>(value);
    }

    template<typename T>
    void update_parameter_array(const std::string &name, std::vector<T> values){
        std::lock_guard<std::mutex> lock(containerLocker);
        dynamicArray[name] = std::make_tuple(std::make_any<std::vector<T>>(values), static_cast<int>(values.size()));
    }

    bool contains(const std::string &name){
        std::lock_guard<std::mutex> lock(containerLocker);
        return dynamic.count(name) > 0;
    }

    bool contains_array(const std::string &name){
        std::lock_guard<std::mutex> lock(containerLocker);
        return dynamicArray.count(name) > 0;
    }

    template<typename T>
    T get(const std::string &name){

        std::lock_guard<std::mutex> lock(containerLocker);
        if(dynamic.count(name) == 0){
            return T{};
        }

        try{
            return std::any_cast<T>(dynamic[name]);
        }catch (const std::bad_any_cast& e){
            log_error(std::string("Get: ") + e.what());
        }
        return T{};
    }

    template<typename T>
    T *get_ptr(const std::string &name){

        std::lock_guard<std::mutex> lock(containerLocker);
        if(dynamic.count(name) == 0){
            return nullptr;
        }

        T *value = std::any_cast<T>(&dynamic[name]);
        if(value == nullptr){
            log_error("Invalid any cast.");
        }

        return value;
    }

    template<typename T>
    void get(const std::string &name, T &value){

        std::lock_guard<std::mutex> lock(containerLocker);
        if(dynamic.count(name) == 0){
            return T{};
        }

        try{
            value = std::any_cast<T>(dynamic[name]);
        }catch (const std::bad_any_cast& e){
            log_error(std::string("Get: ") + e.what());
        }
        return;
    }

    template<typename T>
    std::vector<T> get_array(const std::string &name){

        std::lock_guard<std::mutex> lock(containerLocker);
        if(dynamicArray.count(name) == 0){
            return std::vector<T>{};
        }

        try{
            return std::any_cast<std::vector<T>>(std::get<0>(dynamicArray[name]));
        }catch (const std::bad_any_cast& e){
            log_error(std::string("GetArray: ") + e.what());
        }

        return std::vector<T>{};
    }

    int get_array_size(const std::string &name);

    void log_warning(std::string warningMessage);
    void log_error(std::string errorMessage);
    void log(std::string message);
    void stack_trace_log(std::string stackTraceMessage);

    std::unique_ptr<StackTraceCB> stackTraceCB = nullptr;
    std::unique_ptr<LogCB> logCB = nullptr;
    std::unique_ptr<LogWarningCB> logWarningCB = nullptr;
    std::unique_ptr<LogErrorCB> logErrorCB= nullptr;

    std::mutex containerLocker;
    std::map<std::string, std::any> dynamic;
    std::map<std::string, std::tuple<std::any,int>> dynamicArray;

protected:   

};
}
