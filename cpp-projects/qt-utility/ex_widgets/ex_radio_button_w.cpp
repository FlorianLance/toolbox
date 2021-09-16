
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

#include "ex_radio_button_w.hpp"

using namespace tool::ex;

ExRadioButtonW::ExRadioButtonW(QString name) : ExItemW<QRadioButton>(UiType::Radio_button, name){
    connect(w.get(), &QRadioButton::toggled, this, [=]{trigger_ui_change();});
}

std::vector<ExBaseW *> ExRadioButtonW::init_group_widgets(QButtonGroup &group, std::vector<ExRadioButtonW *> widgets, std::vector<QString> textes, std::vector<bool> checkedState, std::vector<bool> enabledState){

    std::vector<ExBaseW *> bW;
    group.blockSignals(true);

    for(auto &w : widgets){
        w->blockSignals(true);
        bW.push_back(w);
        group.addButton(w->w.get());
    }

    if(widgets.size() == textes.size() && widgets.size() == checkedState.size()){
        for(size_t ii = 0; ii < widgets.size(); ++ii){
            widgets[ii]->w->setText(textes[ii]);
            widgets[ii]->w->setChecked(checkedState[ii]);
        }

    }else{
        qWarning() << "ExRadioButtonW::init_group_widgets error";
        for(auto &w : widgets){
            w->blockSignals(false);
        }

        return bW;
    }

    if(widgets.size() == enabledState.size()){
        for(size_t ii = 0; ii < widgets.size(); ++ii){
            widgets[ii]->w->setEnabled(enabledState[ii]);
        }
    }else{
        for(size_t ii = 0; ii < widgets.size(); ++ii){
            widgets[ii]->w->setEnabled(true);
        }
    }

    for(auto &w : widgets){
        w->blockSignals(false);
    }
    group.blockSignals(false);

    return bW;
}

void ExRadioButtonW::update_from_arg(const Arg &arg) {

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(arg.generator.has_value()){

    }else{

    }

    w->setChecked(arg.to_bool_value());

    w->blockSignals(false);
}



Arg ExRadioButtonW::convert_to_arg() const {
    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from(w->isChecked());

    // generator
    if(hasGenerator){
        // ...
    }

    return arg;
}

//#include "moc_ex_radio_button_w.cpp"
