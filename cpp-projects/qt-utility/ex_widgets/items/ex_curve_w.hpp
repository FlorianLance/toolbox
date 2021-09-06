
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

// Qt
#include <QMouseEvent>

// local
#include "widgets/curve_widget.hpp"
#include "ex_spin_box_w.hpp"
#include "ex_double_spin_box_w.hpp"
#include "ex_checkbox_w.hpp"
#include "ex_combo_box_text_w.hpp"

namespace tool::ex{


class ExCurveW : public ExItemW<QFrame>{

public :

    ExCurveW();
    ExCurveW  *init_widget(QString title, QString xTitle = "x", QString yTitle = "y", geo::Pt2d xRange = {0,1}, geo::Pt2d yRange = {0,1}, bool enabled = true);

    void init_connection(const QString &nameParam) override;
    void update_from_arg(const Arg &arg) override;
    Arg convert_to_arg() const override;

    // curve
    tool::ui::CurveW *curveW = nullptr;
    // id curve
    ExSpinBoxW currentCurveId;
    // actions
    QPushButton *resetB = nullptr;
    QPushButton *addPointB = nullptr;
    // range x    
    ExDoubleSpinBoxW minX;
    ExDoubleSpinBoxW maxX;
    // range y
    ExDoubleSpinBoxW minY;
    ExDoubleSpinBoxW maxY;
    // first/last y
    ExDoubleSpinBoxW firstY;
    ExDoubleSpinBoxW lastY;
    // point to add
    ExDoubleSpinBoxW addX;
    ExDoubleSpinBoxW addY;
    // settings
    ExCheckBoxW fitted;
//    ExCheckBoxW relative;
    ExComboBoxTextW type;
};


}


