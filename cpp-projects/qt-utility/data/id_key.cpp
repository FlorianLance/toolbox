
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

#include "id_key.hpp"

// std
#include <iostream>

// local
#include <qt_logger.hpp>

using namespace tool::ex;

IdKey::IdKey(IdKey::Type type, int id) : m_type(type){

    if(id == -1){
        m_id = current_id()++;
    }else{
        m_id = id;
        if(id >= current_id()){
            current_id() = id+1;
        }
    }

    auto &set = current_set();
    if(set.count(m_id) != 0){
        if(m_type != Type::UiItemArgument && m_type != Type::Element && m_type != Type::Interval){// && m_type) != Type::Connector){
            QtLogger::error(QSL("Id of type ") % from_view(type_name()) % QSL(" already exists: ") % QString::number(m_id));
        }
    }else{
        set.insert(m_id);
    }
}

void IdKey::reset(){

    currentId.clear();
    for (auto& p : types.data) {
        currentId[std::get<0>(p)] = 0;
    }

    keys.clear();
    for (auto& p : types.data) {
        keys[std::get<0>(p)] = {};
    }
}

constexpr IdKey::TypeStr IdKey::type_name() const {
    return to_string(m_type);
}
