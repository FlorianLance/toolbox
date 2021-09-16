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


#include "ex_list_labels_w.hpp"

using namespace tool::ex;


ExListLabelsW::ExListLabelsW(QString name) : ExItemW<QListWidget>(UiType::List_labels, name){
    connect(w.get(), &QListWidget::itemChanged, this, [=]{trigger_ui_change();});
}

ExListLabelsW *ExListLabelsW::init_widget(bool enabled){
    w->clear();
    w->setEnabled(enabled);
    return this;
}

void ExListLabelsW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(arg.generator.has_value()){
        // ...
    }

    w->clear();
    auto labels = arg.split_value();
    for(auto &label : labels){
        w->addItem(label);
    }

    w->blockSignals(false);
}


Arg ExListLabelsW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();

    // value
    QStringList list;
    for(int ii = 0; ii < w->count(); ++ii){
        list << w->item(ii)->text();
    }
    arg.init_from(list, "|");

    // generator
    if(hasGenerator){
        // ...
    }
    return arg;
}

