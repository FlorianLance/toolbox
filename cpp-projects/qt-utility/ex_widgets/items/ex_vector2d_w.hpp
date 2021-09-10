
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

class ExVector2dW : public ExItemW<QFrame>{

public:

    ExVector2dW(QString name ="");
    ExVector2dW *init_widget(const QString &title, Vector2dSettings settings={}, bool enabled=true);

    void update_from_arg(const Arg &arg) override;
    Arg convert_to_arg() const override;
    void set_generator(QString genName) override;

    void set_decimals(int d);
    void set_values(geo::Pt2d values);
    void set_steps(geo::Pt2d steps);
    void set_min(geo::Pt2d min);
    void set_max(geo::Pt2d max);

    void init_default_tooltip(QString name) override;


public:

    QLabel *laTitle = nullptr;

    QLabel *xText = nullptr;
    QLabel *yText = nullptr;

    ExFloatSpinBoxW x;
    ExFloatSpinBoxW y;
};

}
