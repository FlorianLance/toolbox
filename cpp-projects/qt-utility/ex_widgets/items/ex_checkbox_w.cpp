
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

#include "ex_checkbox_w.hpp"

using namespace tool::ex;

ExCheckBoxW::ExCheckBoxW(QString name) : ExItemW<QCheckBox>(UiType::Check_box, name){
    connect(w.get(), &QCheckBox::stateChanged, this, [=]{trigger_ui_change();});
}

ExCheckBoxW *ExCheckBoxW::init_widget(QString txt, bool checked, bool enabled){
    ui::W::init(w.get(), txt, checked, enabled);
    return this;
}

void ExCheckBoxW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(arg.generator.has_value()){

//        if(arg.generator->info.has_value()){
        init_widget("", arg.to_bool_value(), true);
//        }else{
//            qWarning() << "ExCheckBoxW Invalid generator.";
//        }
    }else{
        w->setChecked(arg.to_bool_value());
    }


    w->blockSignals(false);
}

Arg ExCheckBoxW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from(w->isChecked());

    // generator
    if(hasGenerator){
        arg.generator->info = w->text();
    }
    return arg;
}
