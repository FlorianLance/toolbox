
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

// local
#include "qt_str.hpp"
#include "ex_item_w.hpp"

namespace tool::ex {

class ExSliderIntegerW : public ExItemW<QFrame>{

public:

    ExSliderIntegerW(QString name ="");
    ExSliderIntegerW* init_widget(QString title, MinV<int> min, V<int> value, MaxV<int> max, StepV<int> step);

    void update_from_arg(const Arg &arg) override;
    Arg convert_to_arg() const override;

    void init_default_tooltip(QString name) override;

    QLabel *title = nullptr;
    QLabel *minMax = nullptr;
    QLabel *valueTxt = nullptr;
    QSlider *value = nullptr;
    QPushButton *less = nullptr;
    QPushButton *more = nullptr;
};

class ExSliderFloatW : public ExItemW<QFrame>{

public:

    ExSliderFloatW(QString name ="");
    ExSliderFloatW *init_widget(QString title, MinV<qreal> min, V<qreal> valueTxt, MaxV<qreal> max, StepV<qreal> step);

    void update_from_arg(const Arg &arg) override;
    Arg convert_to_arg() const override;

    void init_default_tooltip(QString name) override;

    QString value_to_string() const;

    QLabel *title = nullptr;
    QLabel *minMax = nullptr;
    QLabel *valueTxt = nullptr;
    QSlider *value = nullptr;
    QPushButton *less = nullptr;
    QPushButton *more = nullptr;

private:
    int m_nbSteps;
    qreal m_min;
    qreal m_max;
    qreal m_step;
    qreal m_value;
};


}
