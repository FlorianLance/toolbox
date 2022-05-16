
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
        m_id = currentId[m_type];
        currentId[m_type]++;
    }else{
        m_id = id;
        if(m_id >= currentId[m_type]){
            currentId[m_type] = m_id + 1;
        }
    }

    if(keys[m_type].count(m_id) == 0){
        keys[m_type].insert(m_id);
    }else{

        if(m_type != Type::UiItemArgument && m_type != Type::Element && m_type != Type::Interval){

            int idToTest = 0;
            while(keys[m_type].count(idToTest) == 0){
                ++idToTest;
            }
            keys[m_type].insert(m_id = idToTest);

//            QtLogger::error(QSL("Id [") % QString::number(m_id) % QSL("] of type [") % from_view(type_name()) % QSL("] already exists:."));
        }
    }

}

IdKey::~IdKey(){

    if(m_type == Type::Component || m_type == Type::Condition || m_type == Type::Config){
        if(keys[m_type].contains(m_id)){
            keys[m_type].erase(m_id);
        }else{
            QtLogger::error(QSL("Id [") % QString::number(m_id) % QSL("] of type [") % from_view(type_name()) % QSL("] cannot be removed from set."));
        }
    }

//    if(m_type != Type::UiItemArgument && m_type != Type::Element && m_type != Type::Interval){
//        if(keys[m_type].contains(m_id)){
//            keys[m_type].erase(m_id);
//        }else{
//            QtLogger::error(QSL("Id [") % QString::number(m_id) % QSL("] of type [") % from_view(type_name()) % QSL("] cannot be removed from set."));
//        }
//    }
}

void IdKey::reset(){

    for (auto& p : types.data) {
        currentId[std::get<0>(p)] = 0;
    }

    for (auto& p : types.data) {
        keys[std::get<0>(p)].clear();
    }
}

constexpr IdKey::TypeStr IdKey::type_name() const {
    return to_string(m_type);
}
