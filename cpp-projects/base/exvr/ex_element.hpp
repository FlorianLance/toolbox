

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
#include <unordered_map>
#include <any>

// local
#include "utility/format.hpp"
#include "utility/tuple_array.hpp"
#include "utility/logger.hpp"

typedef void (__stdcall * LogMessageCB)(const char*);
typedef void (__stdcall * LogWarningCB)(const char*);
typedef void (__stdcall * LogErrorCB)(const char*);

typedef void (__stdcall * LogMessageIdCB)(const char*, int, int);
typedef void (__stdcall * LogWarningIdCB)(const char*, int, int);
typedef void (__stdcall * LogErrorIdCB)(const char*, int, int);

typedef void (__stdcall * StackTraceCB)(const char*);

typedef int (__stdcall * GetCB)(const char*);
typedef int (__stdcall * IsVisibleCB)(int);
typedef int (__stdcall * IsUpdatingCB)(int);
typedef int (__stdcall * IsClosedCB)(int);

typedef void (__stdcall * SignalBoolCB)(int, int,int);
typedef void (__stdcall * SignalIntCB)(int, int,int);
typedef void (__stdcall * SignalFloatCB)(int, int,float);
typedef void (__stdcall * SignalDoubleCB)(int, int,double);
typedef void (__stdcall * SignalStringCB)(int, int,const char*);

typedef void (__stdcall * NextCB)();
typedef void (__stdcall * PreviousCB)();
typedef void (__stdcall * CloseCB)(int);

typedef long (__stdcall * EllapsedTimeExpMsCB)();
typedef long (__stdcall * EllapsedTimeRoutineMsCB)();


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
                log_error(fmt("get: cast error: {}", e.what()));
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
                log_error(fmt("get_ptr: cast error: {}", e.what()));
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
                log_error(fmt("get_array: cast error: {}", e.what()));
            }
        }
        return std::vector<T>{};
    }

    int get_array_size(ParametersContainer pc, const std::string &name);


    virtual int key() = 0;
    virtual Logger::SenderT sender_type() = 0;


protected:

    std::unordered_map<ParametersContainer, std::unordered_map<std::string, std::any>> containers = {
        {ParametersContainer::Global,        {}}
    };
    std::unordered_map<ParametersContainer, std::unordered_map<std::string, std::tuple<std::any,int>>> arrayContainers{
        {ParametersContainer::Global,        {}}
    };

public:

    static inline std::unique_ptr<LogMessageCB> logMessageCBP = nullptr;
    static inline std::unique_ptr<LogWarningCB> logWarningCBP = nullptr;
    static inline std::unique_ptr<LogErrorCB> logErrorCBP     = nullptr;
    static inline std::unique_ptr<LogMessageIdCB> logMessageIdCBP = nullptr;
    static inline std::unique_ptr<LogWarningIdCB> logWarningIdCBP = nullptr;
    static inline std::unique_ptr<LogErrorIdCB> logErrorIdCBP     = nullptr;
    static inline std::unique_ptr<StackTraceCB> stackTraceCBP     = nullptr;
    static inline std::unique_ptr<EllapsedTimeExpMsCB> ellapsedTimeExpMsCBP = nullptr;
    static inline std::unique_ptr<EllapsedTimeRoutineMsCB> ellapsedTimeRoutineMsCBP = nullptr;
    static inline std::unique_ptr<GetCB> getCBP = nullptr;
    static inline std::unique_ptr<IsVisibleCB> isVisibleCBP= nullptr;
    static inline std::unique_ptr<IsUpdatingCB> isUpdatingCBP= nullptr;
    static inline std::unique_ptr<IsClosedCB> isClosedCBP= nullptr;
    static inline std::unique_ptr<NextCB> nextCBP = nullptr;
    static inline std::unique_ptr<PreviousCB> previousCBP = nullptr;
    static inline std::unique_ptr<CloseCB> closeCBP = nullptr;
    static inline std::unique_ptr<SignalBoolCB> signalBoolCBP= nullptr;
    static inline std::unique_ptr<SignalIntCB> signalIntCBP= nullptr;
    static inline std::unique_ptr<SignalFloatCB> signalFloatCBP= nullptr;
    static inline std::unique_ptr<SignalDoubleCB> signalDoubleCBP= nullptr;
    static inline std::unique_ptr<SignalStringCB> signalStringCBP= nullptr;

    static void init_callbacks(
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
        SignalStringCB signalStringCB
    );
};

}

