
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

#include "ex_color_frame_w.hpp"

using namespace tool;
using namespace tool::ex;

ExColorFrameW::ExColorFrameW() : ExItemW<QFrame>(UiType::Color_frame){
    w->setMinimumSize(QSize(20,20));
}

ExColorFrameW *ExColorFrameW::init_widget(bool on, bool enabled){
    onState = on;
    ui::W::init(w.get(), onState, enabled);
    return this;
}

void ExColorFrameW::update_from_arg(const Arg &arg){
    ExItemW::update_from_arg(arg);

    w->blockSignals(true);
    update(arg.to_bool_value());
    w->blockSignals(false);
}

Arg ExColorFrameW::convert_to_arg() const{

    Arg arg = ExItemW::convert_to_arg();
    arg.init_from(onState);

    // generator
    if(generatorName.length() > 0){
        // ...
    }
    return arg;
}

void ExColorFrameW::update(bool on){
    if(onState == on){
        return;
    }
    w->setStyleSheet((onState = on) ? stateOn : stateOff);
}

void ExColorFrameW::init_connection(const QString &nameParam){
    // no events
    Q_UNUSED(nameParam)
}
