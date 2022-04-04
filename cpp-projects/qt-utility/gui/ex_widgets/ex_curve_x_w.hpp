
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

// QWt
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

// local
#include "ex_line_edit_w.hpp"
#include "ex_spin_box_w.hpp"
#include "ex_double_spin_box_w.hpp"
#include "ex_checkbox_w.hpp"
#include "ex_select_color_w.hpp"

namespace tool::ex{


class CurveXW : public QwtPlot{

    Q_OBJECT

public:

    std_v1<double> xCoords;
    std_v1<double> yCoords;

    CurveXW();


public slots:

    void set_y_range(qreal min, qreal max, bool autoMinMax);
    void set_line_color(QColor color);
    void set_max_size(int maxSize);
    void add_value(qreal y);
    void add_values(const std_v1<double> &yValues);
    void clear();

    void update_y_scale_from_min_max();

private:
    QwtPlotCurve m_curve;
    int m_maxSize = 100;
    bool m_autoMinMax = true;

};

class CurveXManagerW : public QWidget{
public:

    CurveXManagerW();

    CurveXW curve;

    ExLineEditW name = {"name"};    
    ExDoubleSpinBoxW minY = {"min_y"};
    ExDoubleSpinBoxW maxY = {"max_y"};
    ExCheckBoxW automaticMinMax = {"auto"};
    ExSpinBoxW sizeMax = {"size_max"};
    ExSelectColorW color = {"color"};
};


class ExCurveXW : public ExItemW<CurveXManagerW>{

Q_OBJECT
public :

    ExCurveXW(QString name ="");
    ExCurveXW  *init_widget(QString title, bool enabled = true);

    void update_from_arg(const Arg &arg) override;
    Arg convert_to_arg() const override;

signals:

    void name_changed_signal(QString name);

};


}
