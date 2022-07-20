

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

// local
#include <any>

// local
#include "ex_experiment.hpp"

namespace tool::ex {

using namespace std::literals::string_view_literals;

enum class ParametersContainer : int {
    InitConfig=0, CurrentConfig=1, Dynamic=2, Global=3,SizeEnum
};

using PC  = ParametersContainer;
using Name = std::string_view;
using TParametersContainer = std::tuple<PC, Name>;

static constexpr TupleArray<ParametersContainer::SizeEnum,TParametersContainer> parametersContainers ={{
    TParametersContainer
    {PC::InitConfig,     "init"sv  },
    {PC::CurrentConfig,  "current"sv },
    {PC::Dynamic,        "dynamic"sv },
    {PC::Global,         "global"sv },
}};

[[maybe_unused]] constexpr static Name get_name(ParametersContainer pc) {
    return parametersContainers.at<0,1>(pc);
}

class ExElement{
public:

    void set_exp(tool::ex::ExExperiment *e){
        exp = e;
    }


    // callbacks
    void log_warning(std::string warningMessage);
    void log_error(std::string errorMessage);
    void log_message(std::string message);
    void stack_trace_log(std::string stackTraceMessage);
    void close(int cKey);
    void next();
    void previous();
    int component_key(std::string componentName);

    // containers
    bool contains(ParametersContainer pc, const std::string &name){
        if(containers.contains(pc)){
            return containers[pc].contains(name);
        }
        return false;
    }

    template<typename T>
    T get(ParametersContainer pc, const std::string &name){
        if(contains(pc,name)){
            try{
                return std::any_cast<T>(containers[pc][name]);
            }catch (const std::bad_any_cast& e){
                log_error(std::format("get: cast error: {}", e.what()));
            }
        }
        return T{};
    }

    template<typename T>
    T* get_ptr(ParametersContainer pc, const std::string &name){
        if(contains(pc,name)){
            try{
                return std::any_cast<T>(&containers[pc][name]);
            }catch (const std::bad_any_cast& e){
                log_error(std::format("get_ptr: cast error: {}", e.what()));
            }
        }
        return nullptr;
    }


    template<typename T>
    void set(ParametersContainer pc, const std::string &name, T value){
        if(!containers.contains(pc)){
            containers[pc] = {};
        }
        containers[pc][name] = value;
    }

    // array
    bool contains_array(ParametersContainer pc, const std::string &name){
        if(arrayContainers.contains(pc)){
            return arrayContainers[pc].contains(name);
        }
        return false;
    }

    template<typename T>
    void set_array(ParametersContainer pc, const std::string &name, std::vector<T> values){
        if(!containers.contains(pc)){
            arrayContainers[pc] = {};
        }
        arrayContainers[pc][name] = std::make_tuple(std::move(values), static_cast<int>(values.size()));
    }

    template<typename T>
    std::vector<T> get_array(ParametersContainer pc, const std::string &name){
        if(contains_array(pc,name)){
            try{
                return std::any_cast<std::vector<T>>(std::get<0>(arrayContainers[pc][name]));
            }catch (const std::bad_any_cast& e){
                log_error(std::format("get_array: cast error: {}", e.what()));
            }
        }
        return std::vector<T>{};
    }

    int get_array_size(ParametersContainer pc, const std::string &name);


    virtual int key() = 0;
    virtual Logger::SenderT sender_type() = 0;

    tool::ex::ExExperiment *exp = nullptr;

protected:

    std::unordered_map<ParametersContainer, std::unordered_map<std::string, std::any>> containers = {
        {ParametersContainer::Global,        {}}
    };
    std::unordered_map<ParametersContainer, std::unordered_map<std::string, std::tuple<std::any,int>>> arrayContainers{
        {ParametersContainer::Global,        {}}
    };
};
}

