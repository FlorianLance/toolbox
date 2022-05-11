
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

struct Key{
    int v;
    bool operator< (const Key& k) const{return this->v < k.v;}
};
[[maybe_unused]] static bool operator==(const Key &l, const Key &r){return  (l.v == r.v);}

struct UiElementKey     : public Key{};
struct SetKey           : public Key{};
struct ActionKey        : public Key{};
struct ConfigKey        : public Key{};
struct IntervalKey      : public Key{};
struct TimelineKey      : public Key{};
struct ConditionKey     : public Key{};
struct ElementKey       : public Key{};
struct ComponentKey     : public Key{};
struct ConnectionKey    : public Key{};
struct ConnectorKey     : public Key{};

//[[maybe_unused]] static bool operator==(const UiElementKey &l, const UiElementKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const SetKey &l, const SetKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const ActionKey &l, const ActionKey &r){return  (l.v == r.v);};
//[[maybe_unused]] static bool operator==(const ConfigKey &l, const ConfigKey &r){return  (l.v == r.v);};
//[[maybe_unused]] static bool operator==(const IntervalKey &l, const IntervalKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const TimelineKey &l, const TimelineKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const ConditionKey &l, const ConditionKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const ElementKey &l, const ElementKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const ComponentKey &l, const ComponentKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const ConnectionKey &l, const ConnectionKey &r){return  (l.v == r.v);}
//[[maybe_unused]] static bool operator==(const ConnectorKey &l, const ConnectorKey &r){return  (l.v == r.v);}


class IdKey{

public :

    enum class Type : int {
        UiItemArgument, Action, Component, Condition, Config, Connection, Element, Interval, Timeline, ButtonElement, Connector, Resource, Set,
        SizeEnum
    };

    using TypeStr = std::string_view;
    using TType = std::tuple<Type, TypeStr>;

    IdKey() = delete;
    IdKey(Type type, int id =-1);

    constexpr int operator()() const noexcept {return m_id;}
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
template<> class hash<tool::ex::Key>{public:size_t operator()(tool::ex::Key const& k) const{return std::hash<int>{}(k.v);}};
template<> class hash<tool::ex::UiElementKey>{public:size_t operator()(tool::ex::UiElementKey const& k) const{return std::hash<int>{}(k.v);}};
template<> class hash<tool::ex::ComponentKey>{public:size_t operator()(tool::ex::ComponentKey const& k) const{return std::hash<int>{}(k.v);}};
template<> class hash<tool::ex::ConfigKey>{public:size_t operator()(tool::ex::ConfigKey const& k) const{return std::hash<int>{}(k.v);}};
}


