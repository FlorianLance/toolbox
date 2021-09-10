
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

#include "ex_double_spin_box_w.hpp"

using namespace tool;
using namespace tool::ex;


ExDoubleSpinBoxW::ExDoubleSpinBoxW(QString name) : ExItemW<QDoubleSpinBox>(UiType::Double_spin_box, name){

    connect(w.get(), QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [&]{
        if(!editFinishedSignal){
            trigger_ui_change();
        }
    });
    connect(w.get(), &QDoubleSpinBox::editingFinished, this, [&]{
        if(editFinishedSignal){
            trigger_ui_change();
        }
    });
}

ExDoubleSpinBoxW *ExDoubleSpinBoxW::init_widget(MinV<qreal> min, V<qreal> value, MaxV<qreal> max, StepV<qreal> singleStep, int decimals, bool enabled){
    ui::W::init(w.get(), min, value, max, singleStep, decimals, enabled);
    w->setMinimumWidth(30);
    return this;
}

ExDoubleSpinBoxW *ExDoubleSpinBoxW::init_widget(DsbSettings settings, bool enabled){
    ui::W::init(w.get(), settings.min, settings.value, settings.max, settings.singleStep, settings.decimals, enabled);
    w->setMinimumWidth(30);
    return this;
}


void ExDoubleSpinBoxW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(arg.generator.name.length() > 0){

        if(const auto &min = arg.generator.min, max = arg.generator.max, decimals = arg.generator.decimals, step = arg.generator.step;
            min.has_value() && max.has_value() && decimals.has_value() && step.has_value()){

            init_widget(
                MinV<qreal>{min.value().toDouble()},
                V<qreal>{arg.to_double_value()},
                MaxV<qreal>{max.value().toDouble()},
                StepV<qreal>{step.value().toDouble()},
                decimals.value().toInt()
            );
        }else{
            qDebug() << "ExDoubleSpinBoxW Invalid generator";
        }
    }else{
        w->setValue(arg.to_double_value());
    }

    w->blockSignals(false);
}

Arg ExDoubleSpinBoxW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from(w->value());

    // generator
    if(arg.generator.name.length() > 0){
        arg.generator.min        = QString::number(w->minimum());
        arg.generator.max        = QString::number(w->maximum());
        arg.generator.step       = QString::number(w->singleStep());
        arg.generator.decimals   = QString::number(w->decimals());
    }
    return arg;
}


