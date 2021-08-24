
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

// std
#include <vector>

// Qt
#include <QMouseEvent>

// QWt
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_curve_fitter.h>

namespace tool::ui{



struct Curve{

    enum class Type : int {
        Custom = 0, YEqualsX,
        EaseInSine, EaseInCubic, EaseInQuint, EaseInCirc, EaseInElastic,
        EaseInQuad, EaseInQuart, EaseInExpo, EaseInBack, //EaseInBounce,
        EaseOutSine, EaseOutCubic, EaseOutQuint, EaseOutCirc,  EaseOutElastic,
        EaseOutQuad, EaseOutQuart, EaseOutExpo, EaseOutBack, //EaseOutBounce,
        EaseInOutSine, EaseInOutCubic, EaseOutInQuint, EaseInOutCirc, EaseInOutElastic,
        EaseInOutQuad, EaseInOutQuart, EaseInOutExpo, EaseInOutBack, //EaseInOutBounce,
        SizeEnum
    };

    Curve();

    void insert_point(double rPosX, double rPosY);
    void remove_point(QPointF pt, double minScaleX, double minScaleY, double diffScaleX, double diffScaleY);

    void set_fitted_state(bool state);
    void set_pen(const QPen &pen);
    void set_type(Type type, double minX, double maxX, double minY, double maxY);

    void update_plot_samples();

    std::vector<double> xCoords = {0,1};
    std::vector<double> yCoords = {0,1};
    QwtPlotCurve plot;
    bool fitted = true;

    Curve::Type type = Curve::Type::Custom;

private :

    const static inline auto n1 = 7.5625;
    const static inline auto d1 = 2.75;

//    double ease_out_bounce(double x);
};


class CurveW : public QwtPlot{

    Q_OBJECT
public:

    CurveW();
    void mousePressEvent(QMouseEvent *event) override;

signals:

    void data_updated_signal();
    void settings_updated_signal();

private:

    double minScaleX = 0.;
    double maxScaleX = 1.;

    double minScaleY = 0.;
    double maxScaleY = 1.;


public slots:

    void reset(size_t nbCurves = 1);

    // data
    void add_point_to_end(double xOffset, double y, size_t idCurve);
    void add_point(double x, double y, size_t idCurve);
    void set_points(std::vector<double> x, std::vector<double> y, size_t idCurve);
    void set_first_y(double value, size_t idCurve);
    void set_last_y(double value, size_t idCurve);
    void set_type(Curve::Type type, size_t idCurve);

    // interpolation
    void set_fitted_state(bool state);//, size_t idCurve = 0);

    // display
    void remove_symbol(size_t idCurve);

    void set_title(QString title);
    void set_x_title(QString title);
    void set_y_title(QString title);
    void set_x_range(qreal min, qreal max);
    void set_y_range(qreal min, qreal max);


    void set_pen(const QPen &pen, size_t idCurve);

public:

    std::vector<std::unique_ptr<Curve>> curves;
};
}
