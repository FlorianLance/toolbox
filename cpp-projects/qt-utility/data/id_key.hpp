
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
};
[[maybe_unused]] static bool operator<(const Key &l, const Key &r){return  (l.v < r.v);}
[[maybe_unused]] static bool operator==(const Key &l, const Key &r){return  (l.v == r.v);}


// OK
struct ActionKey        : public Key{};
struct ConfigKey        : public Key{};
struct ConditionKey     : public Key{};
struct ElementKey       : public Key{};
struct ComponentKey     : public Key{};
struct ConnectionKey    : public Key{};
struct ConnectorKey     : public Key{};
struct ResourceKey      : public Key{};
struct SetKey           : public Key{};


class IdKey{

public :

    enum class Source : int{
        Current=0, Imported
    };

    enum class Type : int {
        Action, Component, Condition, Config, Connection, FlowElement, Connector, Resource, Set, ButtonFlowElement, NotUsed,
        SizeEnum
    };

    using TypeStr = std::string_view;
    using TType = std::tuple<Type, TypeStr>;

    IdKey() = default;
    IdKey(Type type, int id = -1);
    IdKey(const IdKey &) = delete;
    IdKey& operator=(const IdKey&) = delete;

    ~IdKey();

    constexpr int operator()() const noexcept {return m_id;}
    constexpr TypeStr type_name() const;
    static constexpr TypeStr to_string(IdKey::Type t) {return m_types.at<0,1>(t);}

    static void reset();
    static void set_source(Source source){m_source = source;}
    static Source current_source(){return m_source;}

private:

    static constexpr TupleArray<Type::SizeEnum,TType> m_types ={{
        TType
        {Type::Action,             "Action"sv},
        {Type::Component,          "Component"sv},
        {Type::Condition,          "Condition"sv},
        {Type::Config,             "Config"sv},
        {Type::Connection,         "Connection"sv},
        {Type::FlowElement,            "Element"sv},
        {Type::Connector,          "Connector"sv},
        {Type::Resource,           "Resource"sv},
        {Type::Set,                "Resource"sv},
        {Type::ButtonFlowElement,  "Button flow element"sv},
    }};

    static inline std::unordered_map<Type, std::array<int,2>> m_currentId = {
        {Type::Action,             {0,0}},
        {Type::Component,          {0,0}},
        {Type::Condition,          {0,0}},
        {Type::Config,             {0,0}},
        {Type::Connection,         {0,0}},
        {Type::FlowElement,            {0,0}},
        {Type::Connector,          {0,0}},
        {Type::Resource,           {0,0}},
        {Type::Set,                {0,0}},
        {Type::ButtonFlowElement,  {0,0}},
    };

    static inline std::unordered_map<Type, std::array<std::unordered_set<int>,2>> m_keys = {
        {Type::Action,             {}},
        {Type::Component,          {}},
        {Type::Condition,          {}},
        {Type::Config,             {}},
        {Type::Connection,         {}},
        {Type::FlowElement,            {}},
        {Type::Connector,          {}},
        {Type::Resource,           {}},
        {Type::Set,                {}},
        {Type::ButtonFlowElement,  {}},
    };

    Type m_type = Type::NotUsed;
    int m_id = -1;
    static inline Source m_source = Source::Current;
};

}

namespace std{
template<> class hash<tool::ex::Key>{public:size_t operator()(tool::ex::Key const& k) const{return std::hash<int>{}(k.v);}};
//template<> class hash<tool::ex::UiElementKey>{public:size_t operator()(tool::ex::UiElementKey const& k) const{return std::hash<int>{}(k.v);}};
template<> class hash<tool::ex::ComponentKey>{public:size_t operator()(tool::ex::ComponentKey const& k) const{return std::hash<int>{}(k.v);}};
template<> class hash<tool::ex::ConfigKey>{public:size_t operator()(tool::ex::ConfigKey const& k) const{return std::hash<int>{}(k.v);}};
}


