
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
#include <any>

// local
#include "argument.hpp"

namespace tool::ex {

class ExBaseW : public QObject{

Q_OBJECT
public:
//    ExBaseW(UiType t, QString uiName) : type(t), key(IdKey::Type::UiItemArgument, -1), itemName(uiName){}
    ExBaseW(UiType t, QString uiName) : type(t), itemName(uiName){}
    virtual ~ExBaseW(){}

    virtual void init_tooltip(QString tooltip) = 0;
    virtual void init_default_tooltip(QString key) = 0;

    virtual void set_generator(QString genName){
        generatorName = genName;
    }

    virtual void update_from_arg(const Arg &arg);
    virtual Arg convert_to_arg() const;

    virtual void update_from_resources(){}
    virtual void update_from_components(){}

    virtual ExBaseW *init_widget2(std_v1<std::any> parameters){
        Q_UNUSED(parameters)
        return this;
    }

//    inline void trigger_ui_change(){emit ui_change_signal(UiElementKey{key()});}
//    inline void trigger_action(){emit action_signal(UiElementKey{key()});}

    inline void trigger_ui_change(){emit ui_change_signal(itemName);}
    inline void trigger_action(){emit action_signal(itemName);}

    UiType type;
//    IdKey key;
    QString itemName;
    QString generatorName = "";
    int generatorOrder = -1;

private:

signals:

//    void ui_change_signal(UiElementKey key);
//    void action_signal(UiElementKey key);
    void ui_change_signal(QStringView name);
    void action_signal(QStringView name);

    void update_from_components_signal();
    void update_from_resources_signal();
};



}


