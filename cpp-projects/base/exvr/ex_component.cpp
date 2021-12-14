

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

#include "ex_component.hpp"

using namespace tool::ex;

constexpr const char *ExComponent::get_name(ParametersContainer c) {
    for (auto& p : mapping) {
        if (c == std::get<0>(p)) {
            return std::get<1>(p);
        }
    }
    return "";
}

int ExComponent::get_array_size(ParametersContainer container, const std::string &name){

    std::unique_lock<std::mutex> lock(containerLocker);
    auto m = get_array_container(container);
    if(m->count(name) != 0){
        return std::get<1>((*m)[name]);
    }

    //        log_error("GetArraySize: "  + std::string(get_name(container)) + std::string( " does not contain: ") + name);
    return 0;
}

bool ExComponent::is_initialized(int key){
    if(isInitializedCB){
        return (*isInitializedCB)(key);
    }
    return false;
}

bool ExComponent::is_visible(int key){
    if(isVisibleCB){
        return (*isVisibleCB)(key);
    }
    return false;
}

bool ExComponent::is_updating(int key){
    if(isUpdatingCB){
        return (*isUpdatingCB)(key);
    }
    return false;
}

bool ExComponent::is_closed(int key){
    if(isClosedCB){
        return (*isClosedCB)(key);
    }
    return false;
}

void ExComponent::next(){
    if(nextCB){
        (*nextCB)();
    }
}

void ExComponent::previous(){
    if(previousCB){
        (*previousCB)();
    }
}

void ExComponent::close(int key){
    if(closeCB){
        (*closeCB)(key);
    }
}

void ExComponent::log_warning(std::string warningMessage){
    if(logWarningCB){
        (*logWarningCB)(warningMessage.c_str());
    }else{
        std::cerr << warningMessage << "\n";
    }
}

void ExComponent::log_error(std::string errorMessage){
    if(logErrorCB){
        (*logErrorCB)(errorMessage.c_str());
    }else{
        std::cerr << errorMessage << "\n";
    }
}

void ExComponent::log(std::string message){
    if(logCB){
        (*logCB)(message.c_str());
    }else{
        std::cerr << message << "\n";
    }
}

void ExComponent::stack_trace_log(std::string stackTraceMessage){
    if(stackTraceCB){
        (*stackTraceCB)(stackTraceMessage.c_str());
    }
}

long ExComponent::ellapsed_time_exp_ms(){
    if(ellapsedTimeExpMsCB){
        return (*ellapsedTimeExpMsCB)();
    }
    return 0;
}

long ExComponent::ellapsed_time_routine_ms(){
    if(ellapsedTimeRoutineMsCB){
        return (*ellapsedTimeRoutineMsCB)();
    }
    return 0;
}

int ExComponent::component_key(std::string componentName){
    if(getCB){
        return  (*getCB)(componentName.c_str());
    }
    return -1;
}

void ExComponent::signal_bool(int key, int index, bool value){
    if(signalBoolCB){
        (*signalBoolCB)(key, index, value ? 1 : 0);
    }
}

void ExComponent::signal_int(int key, int index, int value){
    if(signalIntCB){
        (*signalIntCB)(key, index, value);
    }
}

void ExComponent::signal_float(int key, int index, float value){
    if(signalFloatCB){
        (*signalFloatCB)(key, index, value);
    }
}

void ExComponent::signal_double(int key, int index, double value){
    if(signalDoubleCB){
        (*signalDoubleCB)(key, index, value);
    }
}

void ExComponent::signal_string(int key, int index, std::string value){
    if(signalStringCB){
        (*signalStringCB)(key, index, value.c_str());
    }
}

std::map<std::string, std::any> *ExComponent::get_container(ParametersContainer container){

    std::map<std::string, std::any> *m = nullptr;
    switch (container) {
    case ParametersContainer::InitConfig:
        m = &initC;
        break;
    case ParametersContainer::CurrentConfig:
        m = &currentC;
        break;
    case ParametersContainer::Dynamic:
        m = &dynamic;
        break;
    }
    return m;
}

std::map<std::string, std::tuple<std::any, int> > *ExComponent::get_array_container(ParametersContainer container){

    std::map<std::string, std::tuple<std::any,int>> *m = nullptr;
    switch (container) {
    case ParametersContainer::InitConfig:
        m = &initCArray;
        break;
    case ParametersContainer::CurrentConfig:
        m = &currentCArray;
        break;
    case ParametersContainer::Dynamic:
        m = &dynamicArray;
        break;
    }
    return m;
}
