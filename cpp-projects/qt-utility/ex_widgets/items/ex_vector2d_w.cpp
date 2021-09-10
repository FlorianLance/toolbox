
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

#include "ex_vector2d_w.hpp"

using namespace tool;
using namespace tool::ex;

ExVector2dW::ExVector2dW(QString name) : ExItemW<QFrame>(UiType::Vector2D, name){

    w->setFrameShadow(QFrame::Raised);
    w->setFrameShape(QFrame::Shape::NoFrame);

    QHBoxLayout *layout = new QHBoxLayout();
    w->setLayout(layout);
    layout->addWidget(laTitle = ui::W::txt(""));
    ui::L::stretch(layout);
    layout->addWidget(xText = ui::W::txt("x:"));
    layout->addWidget(x());
    layout->addWidget(yText = ui::W::txt("y:"));
    layout->addWidget(y());
    layout->setContentsMargins(0,0,0,0);

    connect(&x, &ExBaseW::ui_change_signal ,this, &ExBaseW::ui_change_signal);
    connect(&y, &ExBaseW::ui_change_signal ,this, &ExBaseW::ui_change_signal);
}

ExVector2dW *ExVector2dW::init_widget(const QString &title, Vector2dSettings settings, bool enabled){
    laTitle->setText(title);
    w->setEnabled(enabled);
    x.init_widget(settings.sX);
    y.init_widget(settings.sY);
    return this;
}

void ExVector2dW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    auto args = arg.split_value_to_atoms_args();
    if(args.size() != 2){
        qWarning() << "Invalid argument value " << arg.value() << " for vector2D widget : " << arg.name ;
        return;
    }

    w->blockSignals(true);

    if(generatorName.length() > 0){

        if(const auto &min = arg.generator.min, max = arg.generator.max, decimals = arg.generator.decimals, step = arg.generator.step;
            min.has_value() && max.has_value() && decimals.has_value() && step.has_value()){

            init_widget("", Vector2dSettings{
                DsbSettings{
                    MinV<qreal>{min.value().toDouble()},
                    V<qreal>{args[0].value().toDouble()}, MaxV<qreal>{max.value().toDouble()},
                    StepV<qreal>{step.value().toDouble()},
                    decimals.value().toInt()
                },
                DsbSettings{
                    MinV<qreal>{min.value().toDouble()},
                    V<qreal>{args[1].value().toDouble()}, MaxV<qreal>{max.value().toDouble()},
                    StepV<qreal>{step.value().toDouble()},
                    decimals.value().toInt()
                }
            });

        }else{
            qDebug() << "ExVector2dW Invalid generator";
        }
    }else{
        x.update_from_arg(args[0]);
        y.update_from_arg(args[1]);
    }

    w->blockSignals(false);
}

Arg ExVector2dW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from_args({x.convert_to_arg(),
                        y.convert_to_arg()}, " ", UnityType::System_single);

    // generator
    if(generatorName.length() > 0){
        arg.generator.min        = QString::number(x.w->minimum());
        arg.generator.max        = QString::number(x.w->maximum());
        arg.generator.step = QString::number(x.w->singleStep());
        arg.generator.decimals   = QString::number(x.w->decimals());
    }
    return arg;
}

void ExVector2dW::set_generator(QString genName){

    ExItemW::set_generator(genName);
    x.set_generator(genName);
    y.set_generator(genName);
}

void ExVector2dW::set_decimals(int d){
    x.blockSignals(true);
    y.blockSignals(true);
    x.w->setDecimals(d);
    y.w->setDecimals(d);
    x.blockSignals(false);
    y.blockSignals(false);
}

void ExVector2dW::set_values(geo::Pt2d values){
    x.blockSignals(true);
    y.blockSignals(true);
    x.w->setValue(values.x());
    y.w->setValue(values.y());
    x.blockSignals(false);
    y.blockSignals(false);
}

void ExVector2dW::set_steps(geo::Pt2d steps){
    x.blockSignals(true);
    y.blockSignals(true);
    x.w->setSingleStep(steps.x());
    y.w->setSingleStep(steps.y());
    x.blockSignals(false);
    y.blockSignals(false);
}

void ExVector2dW::set_min(geo::Pt2d min){
    x.blockSignals(true);
    y.blockSignals(true);
    x.w->setMinimum(min.x());
    y.w->setMinimum(min.y());
    x.blockSignals(false);
    y.blockSignals(false);
}

void ExVector2dW::set_max(geo::Pt2d max){
    x.blockSignals(true);
    y.blockSignals(true);
    x.w->setMaximum(max.x());
    y.w->setMaximum(max.y());
    x.blockSignals(false);
    y.blockSignals(false);
}

void ExVector2dW::init_default_tooltip(QString name){
    const QString tooltip = QSL("name: ") % name % QSL("  C# type: ") % from_view(get_tooltip_string(type));
    x.init_tooltip(tooltip);
    y.init_tooltip(tooltip);
}
