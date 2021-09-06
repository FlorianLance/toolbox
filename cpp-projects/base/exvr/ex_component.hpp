

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
#include <any>
#include <map>
#include <iostream>
#include <vector>
#include <string>
#include <array>
#include <mutex>

typedef void (__stdcall * StrackTraceCB)(const char*);
typedef void (__stdcall * LogCB)(const char*);
typedef void (__stdcall * LogWarningCB)(const char*);
typedef void (__stdcall * LogErrorCB)(const char*);
typedef long (__stdcall * EllapsedTimeExpMsCB)();
typedef long (__stdcall * EllapsedTimeRoutineMsCB)();
typedef int (__stdcall * GetCB)(const char*);
typedef int (__stdcall * IsInitializedCB)(int);
typedef int (__stdcall * IsVisibleCB)(int);
typedef int (__stdcall * IsUpdatingCB)(int);
typedef int (__stdcall * IsClosedCB)(int);
typedef void (__stdcall * NextCB)();
typedef void (__stdcall * PreviousCB)();
typedef void (__stdcall * CloseCB)(int);
typedef void (__stdcall * SignalBoolCB)(int, int,int);
typedef void (__stdcall * SignalIntCB)(int, int,int);
typedef void (__stdcall * SignalFloatCB)(int, int,float);
typedef void (__stdcall * SignalDoubleCB)(int, int,double);
typedef void (__stdcall * SignalStringCB)(int, int,const char*);

// local
#include "utility/vector_utility.hpp"

namespace tool::ex {

class ExComponent{

public:

    enum class ParametersContainer : int {
        InitConfig=0, CurrentConfig=1, Dynamic=2
    };

    static constexpr std::array<std::tuple<ParametersContainer, const char*>,static_cast<size_t>(3)> mapping ={{
        {ParametersContainer::InitConfig,       "init"},
        {ParametersContainer::CurrentConfig,    "current"},
        {ParametersContainer::Dynamic,          "dynamic"}
    }};

    static constexpr const char *get_name(ParametersContainer c) {
        for (auto& p : mapping) {
            if (c == std::get<0>(p)) {
                return std::get<1>(p);
            }
        }
        return "";
    }

    virtual ~ExComponent(){}

    virtual bool initialize(){return true;}
    virtual void clean(){}

    virtual void start_experiment(){}
    virtual void stop_experiment(){}

    virtual void set_current_config(const std::string &configName){static_cast<void>(configName);}
    virtual void update_from_current_config(){}
    virtual void pre_start_routine(){}
    virtual void start_routine(){}
    virtual void post_start_routine(){}
    virtual void stop_routine(){}

    virtual void on_gui(){}
    virtual void pre_update(){}
    virtual void update(){}
    virtual void post_update(){}

    virtual void set_visibility(bool visible){static_cast<void>(visible);}
    virtual void set_update_state(bool doUpdate){static_cast<void>(doUpdate);}    
    virtual void play(){}
    virtual void pause(){}

    virtual void update_parameter_from_gui(const std::string &updatedParameter){static_cast<void>(updatedParameter);}
    virtual void action_from_gui(bool initConfig, const std::string &action){static_cast<void>(initConfig);static_cast<void>(action);}

    virtual void slot(int index){static_cast<void>(index);}

    template<typename T>
    void update_parameter(ParametersContainer container, const std::string &name, T value){
        std::unique_lock<std::mutex> lock(containerLocker);
        auto m = get_container(container);
        (*m)[name] = value;
    }

    template<typename T>
    void update_parameter_array(ParametersContainer container, const std::string &name, std_v1<T> values){
        std::unique_lock<std::mutex> lock(containerLocker);
        auto m = get_array_container(container);
        (*m)[name] = std::make_tuple(values, static_cast<int>(values.size()));
    }

    bool contains(ParametersContainer container, const std::string &name){
        std::unique_lock<std::mutex> lock(containerLocker);
        return get_container(container)->count(name) > 0;
    }

    template<typename T>
    T get(ParametersContainer container, const std::string &name){

        std::unique_lock<std::mutex> lock(containerLocker);
        auto m = get_container(container);
        if(m->count(name) == 0){
//            log_error("Get: "  + std::string(get_name(container)) + std::string( " does not contain: ") + name);
            return T{};
        }

        try{
            return std::any_cast<T>((*m)[name]);
        }catch (const std::bad_any_cast& e){
            log_error(std::string("Get: ") + e.what());
        }
        return T{};
    }

    template<typename T>
    const T &const_get(ParametersContainer container, const std::string &name){

        std::unique_lock<std::mutex> lock(containerLocker);
        auto m = get_container(container);
        if(m->count(name) == 0){
            return T{};
        }

        try{
            return std::any_cast<T>((*m)[name]);
        }catch (const std::bad_any_cast& e){
            log_error(std::string("Get: ") + e.what());
        }
        return T{};
    }


    template<typename T>
    std_v1<T> get_array(ParametersContainer container, const std::string &name){

        std::unique_lock<std::mutex> lock(containerLocker);
        auto m = get_array_container(container);
        if(m->count(name) == 0){
//            log_error("GetArray: "  + std::string(get_name(container)) + std::string( " does not contain: ") + name);
            return std_v1<T>{};
        }

        try{
            return std::any_cast<std_v1<T>>(std::get<0>((*m)[name]));
        }catch (const std::bad_any_cast& e){
            log_error(std::string("GetArray: ") + e.what());
        }

        return std_v1<T>{};
    }

    int get_array_size(ParametersContainer container, const std::string &name){

        std::unique_lock<std::mutex> lock(containerLocker);
        auto m = get_array_container(container);
        if(m->count(name) != 0){
            return std::get<1>((*m)[name]);
        }

//        log_error("GetArraySize: "  + std::string(get_name(container)) + std::string( " does not contain: ") + name);
        return 0;
    }

    // callbacks
    static bool is_initialized(int key){
        if(isInitializedCB){
            return (*isInitializedCB)(key);
        }
        return false;
    }

    static bool is_visible(int key){
        if(isVisibleCB){
            return (*isVisibleCB)(key);
        }
        return false;
    }

    static bool is_updating(int key){
        if(isUpdatingCB){
            return (*isUpdatingCB)(key);
        }
        return false;
    }

    static bool is_closed(int key){
        if(isClosedCB){
            return (*isClosedCB)(key);
        }
        return false;
    }

    static void next(){
        if(nextCB){
            (*nextCB)();
        }
    }

    static void previous(){
        if(previousCB){
            (*previousCB)();
        }
    }

    static void close(int key){
        if(closeCB){
            (*closeCB)(key);
        }
    }

    static void log_warning(std::string warningMessage){
        if(logWarningCB){
            (*logWarningCB)(warningMessage.c_str());
        }else{
            std::cerr << warningMessage << "\n";
        }
    }

    static void log_error(std::string errorMessage){
        if(logErrorCB){
            (*logErrorCB)(errorMessage.c_str());
        }else{
            std::cerr << errorMessage << "\n";
        }
    }

    static void log(std::string message){
        if(logCB){
            (*logCB)(message.c_str());
        }else{
            std::cerr << message << "\n";
        }
    }

    static void stack_trace_log(std::string stackTraceMessage){
        if(strackTraceCB){
            (*strackTraceCB)(stackTraceMessage.c_str());
        }
    }


    static long ellapsed_time_exp_ms(){
        if(ellapsedTimeExpMsCB){
            return (*ellapsedTimeExpMsCB)();
        }
        return 0;
    }

    static long ellapsed_time_routine_ms(){
        if(ellapsedTimeRoutineMsCB){
            return (*ellapsedTimeRoutineMsCB)();
        }
        return 0;
    }

    static int component_key(std::string componentName){
        if(getCB){
            return  (*getCB)(componentName.c_str());
        }
        return -1;
    }

    static void signal_bool(int key, int index, bool value){
        if(signalBoolCB){
            (*signalBoolCB)(key, index, value ? 1 : 0);
        }
    }
    static void signal_int(int key, int index, int value){
        if(signalIntCB){
            (*signalIntCB)(key, index, value);
        }
    }
    static void signal_float(int key, int index, float value){
        if(signalFloatCB){
            (*signalFloatCB)(key, index, value);
        }
    }
    static void signal_double(int key, int index, double value){
        if(signalDoubleCB){
            (*signalDoubleCB)(key, index, value);
        }
    }
    static void signal_string(int key, int index, std::string value){
        if(signalStringCB){
            (*signalStringCB)(key, index, value.c_str());
        }
    }


    inline static std::unique_ptr<StrackTraceCB> strackTraceCB = nullptr;
    inline static std::unique_ptr<LogCB> logCB = nullptr;
    inline static std::unique_ptr<LogWarningCB> logWarningCB = nullptr;
    inline static std::unique_ptr<LogErrorCB> logErrorCB= nullptr;
    inline static std::unique_ptr<EllapsedTimeExpMsCB> ellapsedTimeExpMsCB= nullptr;
    inline static std::unique_ptr<EllapsedTimeRoutineMsCB> ellapsedTimeRoutineMsCB= nullptr;
    inline static std::unique_ptr<GetCB> getCB= nullptr;
    inline static std::unique_ptr<IsInitializedCB> isInitializedCB= nullptr;
    inline static std::unique_ptr<IsVisibleCB> isVisibleCB= nullptr;
    inline static std::unique_ptr<IsUpdatingCB> isUpdatingCB= nullptr;
    inline static std::unique_ptr<IsClosedCB> isClosedCB= nullptr;
    inline static std::unique_ptr<NextCB> nextCB= nullptr;
    inline static std::unique_ptr<PreviousCB> previousCB= nullptr;
    inline static std::unique_ptr<CloseCB> closeCB= nullptr;

    inline static std::unique_ptr<SignalBoolCB> signalBoolCB= nullptr;
    inline static std::unique_ptr<SignalIntCB> signalIntCB= nullptr;
    inline static std::unique_ptr<SignalFloatCB> signalFloatCB= nullptr;
    inline static std::unique_ptr<SignalDoubleCB> signalDoubleCB= nullptr;
    inline static std::unique_ptr<SignalStringCB> signalStringCB= nullptr;

private:

    std::map<std::string, std::any> *get_container(ParametersContainer container){

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

    std::map<std::string, std::tuple<std::any,int>> *get_array_container(ParametersContainer container){

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

protected:

    std::mutex containerLocker;

    std::map<std::string, std::any> initC;
    std::map<std::string, std::any> currentC;
    std::map<std::string, std::any> dynamic;

    std::map<std::string, std::tuple<std::any,int>> initCArray;
    std::map<std::string, std::tuple<std::any,int>> currentCArray;
    std::map<std::string, std::tuple<std::any,int>> dynamicArray;
};
}

