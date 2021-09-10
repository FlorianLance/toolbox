
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

#include "ex_slider_w.hpp"

using namespace tool;
using namespace tool::ex;



ExSliderIntegerW::ExSliderIntegerW(QString name) : ExItemW<QFrame>(UiType::Slider_integer, name){

    w->setFrameShadow(QFrame::Raised);
    w->setFrameShape(QFrame::Shape::NoFrame);

    QHBoxLayout *layout = new QHBoxLayout();
    w->setLayout(layout);
    layout->addWidget(title = new QLabel());
    layout->addWidget(minMax = new QLabel());
    layout->addWidget(value = new QSlider());
    value->setOrientation(Qt::Horizontal);
    layout->addWidget(valueTxt = new QLabel());

    layout->setContentsMargins(2,2,2,2);
    layout->setSpacing(2);

    connect(value, &QSlider::valueChanged, this, [&](int value){
        valueTxt->setText(QString::number(value));
        trigger_ui_change();
    });
}

ExSliderIntegerW *ExSliderIntegerW::init_widget(QString titleV, MinV<int> minV, V<int> valueV, MaxV<int> maxV, StepV<int> stepV){
    title->setText(titleV);
    ui::W::init(value, minV, valueV, maxV, stepV);
    valueTxt->setText(QString::number(valueV.v));
    minMax->setText(QSL("[") % QString::number(minV.v) % QSL(" : ") % QString::number(maxV.v) % QSL("]"));
    return this;
}


void ExSliderIntegerW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(generatorName.length() > 0){

        if(arg.generator.min.has_value()){
            value->setMinimum(arg.generator.min.value().toInt());
        }

        if(arg.generator.max.has_value()){
            value->setMaximum(arg.generator.max.value().toInt());
        }

        if(arg.generator.step.has_value()){
            value->setSingleStep(arg.generator.step.value().toInt());
        }
    }

    int valueV = arg.to_int_value();
    value->setValue(valueV);
    minMax->setText(QSL("[") % QString::number(value->minimum()) % QSL(" : ") % QString::number(value->maximum()) % QSL("]"));
    valueTxt->setText(QString::number(valueV));

    w->blockSignals(false);
}

Arg ExSliderIntegerW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from(value->value());

    // generator
    if(generatorName.length() > 0){
        arg.generator.min   = QString::number(value->minimum());
        arg.generator.max   = QString::number(value->maximum());
        arg.generator.step  = QString::number(value->singleStep());
    }

    return arg;
}

void ExSliderIntegerW::init_default_tooltip(QString name){

    const QString tooltip = QSL("name: ") % name % QSL("  C# type: ") % from_view(get_tooltip_string(type));
    title->setToolTipDuration(-1);
    title->setToolTip(tooltip);
    minMax->setToolTipDuration(-1);
    minMax->setToolTip(tooltip);
    valueTxt->setToolTipDuration(-1);
    valueTxt->setToolTip(tooltip);
    value->setToolTipDuration(-1);
    value->setToolTip(tooltip);
}

ExSliderFloatW::ExSliderFloatW(QString name) : ExItemW<QFrame>(UiType::Slider_double, name){

    w->setFrameShadow(QFrame::Raised);
    w->setFrameShape(QFrame::Shape::NoFrame);

    QHBoxLayout *layout = new QHBoxLayout();
    w->setLayout(layout);
    layout->addWidget(title = new QLabel());
    layout->addWidget(minMax = new QLabel());
    layout->addWidget(value = new QSlider());
    value->setOrientation(Qt::Horizontal);
    layout->addWidget(valueTxt = new QLabel());

    connect(value, &QSlider::valueChanged, this, [&](int value){
        qreal dVal = m_min + (1.0*value/m_nbSteps) * (m_max-m_min);
        valueTxt->setText(QString::number(dVal));
        trigger_ui_change();
    });
}

ExSliderFloatW *ExSliderFloatW::init_widget(QString titleV, MinV<qreal> minV, V<qreal> valueV, MaxV<qreal> maxV, StepV<qreal> stepV){

    title->setText(titleV);

    m_min = minV.v;
    m_max = maxV.v;
    m_step = stepV.v;

    const qreal diff = (m_max-m_min);
    m_nbSteps = static_cast<int>(diff/m_step);
    ui::W::init(value, MinV<int>{0}, V<int>{static_cast<int>((valueV.v-m_min)/diff*m_nbSteps)}, MaxV<int>{m_nbSteps},  StepV<int>{1});
    minMax->setText(QSL("[") % QString::number(minV.v) % QSL(" : ") % QString::number(maxV.v) % QSL("]"));
    valueTxt->setText(QString::number(valueV.v));

    return this;
}


QString ExSliderFloatW::value_to_string() const {
    qreal dVal = m_min + (1.0*value->value()/m_nbSteps) * (m_max-m_min);
    return QString::number(dVal);
}


void ExSliderFloatW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    qreal valueV = arg.to_double_value();

    if(generatorName.length() > 0){

        if(arg.generator.min.has_value()){
            m_min = arg.generator.min.value().toDouble();
        }

        if(arg.generator.max.has_value()){
            m_max = arg.generator.max.value().toDouble();
        }

        if(arg.generator.step.has_value()){
            m_step = arg.generator.step.value().toDouble();
        }

        m_nbSteps = static_cast<int>((m_max-m_min)/m_step);
    }

    ui::W::init(value, MinV<int>{0}, V<int>{static_cast<int>((valueV/m_max)*m_nbSteps)}, MaxV<int>{m_nbSteps},  StepV<int>{1});
    minMax->setText(QSL("[") % QString::number(m_min) % QSL(" : ") % QString::number(m_max) % QSL("]"));
    valueTxt->setText(QString::number(valueV));

    w->blockSignals(false);

}

Arg ExSliderFloatW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from(static_cast<float>(m_min + (1.0*value->value()/m_nbSteps) * (m_max-m_min)));

    // generator
    if(generatorName.length() > 0){
        arg.generator.min   = QString::number(m_min);
        arg.generator.max   = QString::number(m_max);
        arg.generator.step  = QString::number(m_step);
    }

    return arg;
}

void ExSliderFloatW::init_default_tooltip(QString name){

    const QString tooltip = QSL("name: ") % name % QSL("  C# type: ") % from_view(get_tooltip_string(type));
    title->setToolTipDuration(-1);
    title->setToolTip(tooltip);
    minMax->setToolTipDuration(-1);
    minMax->setToolTip(tooltip);
    valueTxt->setToolTipDuration(-1);
    valueTxt->setToolTip(tooltip);
    value->setToolTipDuration(-1);
    value->setToolTip(tooltip);
}
