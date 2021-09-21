
/*******************************************************************************
** Toolbox-qt-utility                                                         **
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
#include <unordered_set>
#include <unordered_map>

// base
#include "utility/tuple_array.hpp"

namespace tool::ex{

using namespace std::literals::string_view_literals;


struct RowId{int v;};
struct UiElementKey{int v;};
struct SetKey{int v;};
struct ActionKey{int v;};
struct ConfigKey{int v;};
struct IntervalKey{int v;};
struct TimelineKey{int v;};
struct ConditionKey{int v;};
struct ElementKey{int v;};
struct ComponentKey{int v;};
struct ConnectionKey{int v;};
struct ConnectorKey{int v;};

[[maybe_unused]] static bool operator==(const UiElementKey &l, const UiElementKey &r){
    return  (l.v == r.v);
}
[[maybe_unused]] static bool operator==(const ComponentKey &l, const ComponentKey &r){
    return  (l.v == r.v);
}
[[maybe_unused]] static bool operator==(const SetKey &l, const SetKey &r){
    return  (l.v == r.v);
}
[[maybe_unused]] static bool operator==(const ElementKey &l, const ElementKey &r){
    return  (l.v == r.v);
}
[[maybe_unused]] static bool operator==(const ConditionKey &l, const ConditionKey &r){
    return  (l.v == r.v);
}

class IdKey{

public :

    enum class Type : int {
        UiItemArgument, Action, Component, Condition, Config, Connection, Element, Interval, Timeline, ButtonElement, Connector, Resource, Set,
        SizeEnum
    };

    IdKey() = delete;
    IdKey(Type type, int id =-1);

    constexpr int operator()() const {
        return m_id;
    }    


    using TypeStr = std::string_view;
    using TType = std::tuple<Type, TypeStr>;


    constexpr TypeStr type_name() const;
    static constexpr TypeStr to_string(IdKey::Type t) {
        return types.at<0,1>(t);
    }

    static void reset();

private:


    static constexpr TupleArray<Type::SizeEnum,TType> types ={{
        TType
        {Type::UiItemArgument,     "UI item argument"sv},
        {Type::Action,             "Action"sv},
        {Type::Component,          "Component"sv},
        {Type::Condition,          "Condition"sv},
        {Type::Config,             "Config"sv},
        {Type::Connection,         "Connection"sv},
        {Type::Element,            "Element"sv},
        {Type::Interval,           "Interval"sv},
        {Type::Timeline,           "Timeline"sv},
        {Type::ButtonElement,      "ButtonElement"sv},
        {Type::Connector,          "Connector"sv},
        {Type::Resource,           "Resource"sv},
        {Type::Set,                "Set"sv},

    }};


    int &current_id(){
        return currentId[m_type];
    }

    std::unordered_set<int> &current_set(){        
        return keys[m_type];
    }

    static inline std::unordered_map<Type,int> currentId = {};
    static inline std::unordered_map<Type,std::unordered_set<int>> keys = {};

    Type m_type;
    int m_id = -1;
};
}

namespace std{

template<>
class hash<tool::ex::UiElementKey>{
public:
    size_t operator()(tool::ex::UiElementKey const& k) const{
         return std::hash<int>{}(k.v);
    }
};
}

