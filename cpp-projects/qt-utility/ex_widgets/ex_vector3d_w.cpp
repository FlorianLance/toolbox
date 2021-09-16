
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

#include "ex_vector3d_w.hpp"

using namespace tool;
using namespace tool::ex;

ExVector3dW::ExVector3dW(QString name) : ExItemW<QFrame>(UiType::Vector3D, name){

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
    layout->addWidget(zText = ui::W::txt("z:"));
    layout->addWidget(z());
    layout->setContentsMargins(0,0,0,0);

    connect(&x, &ExBaseW::ui_change_signal, this, &ExBaseW::ui_change_signal);
    connect(&y, &ExBaseW::ui_change_signal, this, &ExBaseW::ui_change_signal);
    connect(&z, &ExBaseW::ui_change_signal, this, &ExBaseW::ui_change_signal);
}

ExVector3dW *ExVector3dW::init_widget(const QString &title, Vector3dSettings settings, bool enabled){
    laTitle->setText(title);
    w->setEnabled(enabled);
    x.init_widget(settings.sX);
    y.init_widget(settings.sY);
    z.init_widget(settings.sZ);
    return this;
}

void ExVector3dW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    auto args = arg.split_value_to_atoms_args();
    if(args.size() != 3){
        qWarning() << "Invalid argument value " << arg.value() << " for vector3D widget : " << arg.name ;
        return;
    }

    w->blockSignals(true);

    if(arg.generator.has_value()){

        if(auto dec = arg.generator->decimals; dec.has_value()){
            const auto value = dec.value().toInt();
            x.w->setDecimals(value);
            y.w->setDecimals(value);
            z.w->setDecimals(value);
        }

        if(auto min = arg.generator->min; min.has_value()){
            const auto value = min.value().toDouble();
            x.w->setMinimum(value);
            y.w->setMinimum(value);
            z.w->setMinimum(value);
        }

        if(auto max = arg.generator->max; max.has_value()){
            const auto value = max.value().toDouble();
            x.w->setMaximum(value);
            y.w->setMaximum(value);
            z.w->setMaximum(value);
        }

        if(auto step = arg.generator->step; step.has_value()){
            const auto value = step.value().toDouble();
            x.w->setSingleStep(value);
            y.w->setSingleStep(value);
            z.w->setSingleStep(value);
        }
    }

    x.update_from_arg(args[0]);
    y.update_from_arg(args[1]);
    z.update_from_arg(args[2]);

    w->blockSignals(false);
}

Arg ExVector3dW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from_args({x.convert_to_arg(),
                        y.convert_to_arg(),
                        z.convert_to_arg()}, " ", UnityType::System_single);

    // generator
    if(hasGenerator){
            arg.generator->min      = QString::number(x.w->minimum());
            arg.generator->max      = QString::number(x.w->maximum());
            arg.generator->step     = QString::number(x.w->singleStep());
            arg.generator->decimals = QString::number(x.w->decimals());
    }
    return arg;
}

void ExVector3dW::set_as_generator(){
    ExItemW::set_as_generator();
    x.set_as_generator();
    y.set_as_generator();
    z.set_as_generator();
}

void ExVector3dW::set_decimals(int d){
    x.blockSignals(true);
    y.blockSignals(true);
    z.blockSignals(true);
    x.w->setDecimals(d);
    y.w->setDecimals(d);
    z.w->setDecimals(d);
    x.blockSignals(false);
    y.blockSignals(false);
    z.blockSignals(false);
}

void ExVector3dW::set_values(geo::Pt3d values){
    x.blockSignals(true);
    y.blockSignals(true);
    z.blockSignals(true);
    x.w->setValue(values.x());
    y.w->setValue(values.y());
    z.w->setValue(values.z());
    x.blockSignals(false);
    y.blockSignals(false);
    z.blockSignals(false);
}

void ExVector3dW::set_steps(geo::Pt3d steps){
    x.blockSignals(true);
    y.blockSignals(true);
    z.blockSignals(true);
    x.w->setSingleStep(steps.x());
    y.w->setSingleStep(steps.y());
    z.w->setSingleStep(steps.z());
    x.blockSignals(false);
    y.blockSignals(false);
    z.blockSignals(false);
}

void ExVector3dW::set_min(geo::Pt3d min){
    x.blockSignals(true);
    y.blockSignals(true);
    z.blockSignals(true);
    x.w->setMinimum(min.x());
    y.w->setMinimum(min.y());
    z.w->setMinimum(min.z());
    x.blockSignals(false);
    y.blockSignals(false);
    z.blockSignals(false);
}

void ExVector3dW::set_max(geo::Pt3d max){
    x.blockSignals(true);
    y.blockSignals(true);
    z.blockSignals(true);
    x.w->setMaximum(max.x());
    y.w->setMaximum(max.y());
    z.w->setMaximum(max.z());
    x.blockSignals(false);
    y.blockSignals(false);
    z.blockSignals(false);
}

void ExVector3dW::set_enable_state(geo::Pt3<bool> states){
    x.w->setEnabled(states.x());
    y.w->setEnabled(states.y());
    z.w->setEnabled(states.z());
}

geo::Pt3d ExVector3dW::values() const{
    return {x.w->value(),y.w->value(),z.w->value()};
}

void ExVector3dW::init_default_tooltip(QString name){
    const QString tooltip = QSL("name: ") % name % QSL("  C# type: ") % from_view(get_tooltip_string(type));
    x.init_tooltip(tooltip);
    y.init_tooltip(tooltip);
    z.init_tooltip(tooltip);
}
