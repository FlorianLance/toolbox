

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

// local
#include "ex_utility.hpp"
#include "utility/vector.hpp"

namespace tool::ex {

class ExComponent{

public:

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

    static constexpr const char *get_name(ParametersContainer c);

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
            return std_v1<T>{};
        }

        try{
            return std::any_cast<std_v1<T>>(std::get<0>((*m)[name]));
        }catch (const std::bad_any_cast& e){
            log_error(std::string("GetArray: ") + e.what());
        }

        return std_v1<T>{};
    }

    int get_array_size(ParametersContainer container, const std::string &name);

    // callbacks
    bool is_initialized(int key);
    bool is_visible(int key);
    bool is_updating(int key);
    bool is_closed(int key);
    void next();
    void previous();
    void close(int key);

    void log_warning(std::string warningMessage);
    void log_error(std::string errorMessage);
    void log(std::string message);
    void stack_trace_log(std::string stackTraceMessage);

    long ellapsed_time_exp_ms();
    long ellapsed_time_routine_ms();

    int component_key(std::string componentName);

    void signal_bool(int key, int index, bool value);
    void signal_int(int key, int index, int value);
    void signal_float(int key, int index, float value);
    void signal_double(int key, int index, double value);
    void signal_string(int key, int index, std::string value);


    std::unique_ptr<StackTraceCB> stackTraceCB = nullptr;
    std::unique_ptr<LogCB> logCB = nullptr;
    std::unique_ptr<LogWarningCB> logWarningCB = nullptr;
    std::unique_ptr<LogErrorCB> logErrorCB= nullptr;
    std::unique_ptr<EllapsedTimeExpMsCB> ellapsedTimeExpMsCB= nullptr;
    std::unique_ptr<EllapsedTimeRoutineMsCB> ellapsedTimeRoutineMsCB= nullptr;
    std::unique_ptr<GetCB> getCB= nullptr;
    std::unique_ptr<IsInitializedCB> isInitializedCB= nullptr;
    std::unique_ptr<IsVisibleCB> isVisibleCB= nullptr;
    std::unique_ptr<IsUpdatingCB> isUpdatingCB= nullptr;
    std::unique_ptr<IsClosedCB> isClosedCB= nullptr;
    std::unique_ptr<NextCB> nextCB= nullptr;
    std::unique_ptr<PreviousCB> previousCB= nullptr;
    std::unique_ptr<CloseCB> closeCB= nullptr;

    std::unique_ptr<SignalBoolCB> signalBoolCB= nullptr;
    std::unique_ptr<SignalIntCB> signalIntCB= nullptr;
    std::unique_ptr<SignalFloatCB> signalFloatCB= nullptr;
    std::unique_ptr<SignalDoubleCB> signalDoubleCB= nullptr;
    std::unique_ptr<SignalStringCB> signalStringCB= nullptr;

private:

    std::map<std::string, std::any> *get_container(ParametersContainer container);
    std::map<std::string, std::tuple<std::any,int>> *get_array_container(ParametersContainer container);

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

