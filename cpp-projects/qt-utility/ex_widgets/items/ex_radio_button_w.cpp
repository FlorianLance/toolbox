
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

void ExRadioButtonW::update_from_arg(const Arg &arg) {
    ExItemW::update_from_arg(arg);

    w->blockSignals(true);
    w->setChecked(arg.to_bool_value());
    w->blockSignals(false);
}

ExRadioButtonW *ExRadioButtonW::init_widget(QString txt, bool checked, bool enabled){
    ui::W::init(w.get(), txt, checked, enabled);
    return this;
}

void ExRadioButtonW::init_connection(const QString &nameParam) {
    connect(w.get(), &QRadioButton::toggled, this, [=]{emit ui_change_signal(nameParam);});
}

Arg ExRadioButtonW::convert_to_arg() const {
    Arg arg = ExItemW::convert_to_arg();
    arg.init_from(w->isChecked());
    return arg;
}

