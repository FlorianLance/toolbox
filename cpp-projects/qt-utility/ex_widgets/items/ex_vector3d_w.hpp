
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

// base
#include "geometry/point3.hpp"

// local
#include "ex_float_spin_box_w.hpp"

namespace tool::ex {

class ExVector3dW : public ExItemW<QFrame>{

public:

    constexpr static DsbSettings DefautlTraV = {MinV<qreal>{-10000.}, V<qreal>{0}, MaxV<qreal>{10000.}, StepV<qreal>{0.01}, 3};
    constexpr static DsbSettings DefaultRotV = {MinV<qreal>{-10000.}, V<qreal>{0}, MaxV<qreal>{10000.}, StepV<qreal>{0.1},  3};
    constexpr static DsbSettings DefaultScaV = {MinV<qreal>{-10000.}, V<qreal>{1}, MaxV<qreal>{10000.}, StepV<qreal>{0.01}, 3};

    ExVector3dW();
    ExVector3dW *init_widget(const QString &title, Vector3dSettings settings={}, bool enabled=true);
    void init_connection(const QString &nameParam) override;
    void update_from_arg(const Arg &arg) override;
    Arg convert_to_arg() const override;
    void set_generator(QString genName) override;

    void set_decimals(int d);
    void set_values(geo::Pt3d values);
    void set_steps(geo::Pt3d steps);
    void set_min(geo::Pt3d min);
    void set_max(geo::Pt3d max);
    void set_enable_state(geo::Pt3<bool> states);

    geo::Pt3d values()const;

    void init_default_tooltip(QString name) override;

public:

    QLabel *laTitle = nullptr;

    QLabel *xText = nullptr;
    QLabel *yText = nullptr;
    QLabel *zText = nullptr;

    ExFloatSpinBoxW x;
    ExFloatSpinBoxW y;
    ExFloatSpinBoxW z;
};

}
