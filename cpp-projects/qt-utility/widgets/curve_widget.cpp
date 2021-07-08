
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

#include "curve_widget.hpp"

// Qwt
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>
#include <qwt_curve_fitter.h>

// base
#include "geometry/point2.hpp"

using namespace tool;
using namespace tool::ui;

Curve::Curve(){

    plot.setTitle("");

    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidth(4.);
    set_pen(pen);

    plot.setRenderHint( QwtPlotItem::RenderAntialiased, true );
    plot.setCurveAttribute(QwtPlotCurve::Fitted, fitted);
    plot.setSymbol( new QwtSymbol(
        QwtSymbol::Ellipse,
        QBrush(Qt::yellow),
        QPen(Qt::red, 2),
        QSize(4, 4))
    );

    QwtSplineCurveFitter *fitter = new QwtSplineCurveFitter();
    fitter->setFitMode(QwtSplineCurveFitter::FitMode::Spline);
    plot.setCurveFitter(fitter);

    update_plot_samples();
}

void Curve::set_pen(const QPen &pen){
    plot.setPen(pen);
}

void Curve::set_fitted_state(bool state){
    plot.setCurveAttribute(QwtPlotCurve::Fitted, fitted = state);
}

void Curve::update_plot_samples(){
    plot.setRawSamples(xCoords.data(), yCoords.data(), static_cast<int>(xCoords.size()));
}

void Curve::insert_point(double rPosX, double rPosY){

    for(size_t ii = 0; ii < xCoords.size(); ++ii){
        if(xCoords[ii] > rPosX){
            xCoords.insert(std::begin(xCoords) + static_cast<int>(ii), rPosX);
            yCoords.insert(std::begin(yCoords) + static_cast<int>(ii), rPosY);
            break;
        }
    }
}

void Curve::remove_point(QPointF pt, double minScaleX, double minScaleY, double diffScaleX, double diffScaleY){

    double min = 0.1;
    int index = -1;

    for(size_t ii = 0; ii < xCoords.size(); ++ii){

        const auto &px = (xCoords[ii]-minScaleX)/diffScaleX;
        const auto &py = (yCoords[ii]-minScaleY)/diffScaleY;
        double dist = geo::norm(geo::Pt2<double>{pt.x()-px, pt.y()-py});
        if(dist < min){
            min = dist;
            index = static_cast<int>(ii);
        }
    }

    if(index > -1 && index != 0 && (index < static_cast<int>(xCoords.size())-1)){
        xCoords.erase(std::begin(xCoords) + index);
        yCoords.erase(std::begin(yCoords) + index);
    }
}



CurveW::CurveW(){

    // background
    setCanvasBackground( Qt::gray );

    // axis
    QFont axisFont("Calibri", 8);
    setAxisFont(QwtPlot::xBottom,   axisFont);
    setAxisFont(QwtPlot::yLeft,     axisFont);
    setAxisScale( QwtPlot::xBottom, minScaleX, maxScaleX);
    setAxisScale( QwtPlot::yLeft,   minScaleY, maxScaleY);

    curves.push_back(std::make_unique<Curve>());
    curves[0]->plot.attach(this);

    QwtPlot::replot();
    QwtPlot::repaint();
}

void CurveW::set_title(QString title){

    QwtText t(title);
    QFont font("Calibri", 10);
    font.setBold(true);
    t.setFont(font);
    setTitle(t);
}

void CurveW::set_x_title(QString title){

    QwtText t(title);
    QFont font("Calibri", 9);
    font.setBold(false);
    t.setFont(font);

    setAxisTitle(QwtPlot::xBottom, t);
}

void CurveW::set_y_title(QString title){

    QwtText t(title);
    QFont font("Calibri", 9);
    font.setBold(false);
    t.setFont(font);

    setAxisTitle(QwtPlot::yLeft, t);
}

void CurveW::set_x_range(qreal min, qreal max){

    minScaleX = min;
    maxScaleX = max;

    // erase values < minScaleX
    for(auto &curve : curves){

        int count = 0;
        for(const auto &x : curve->xCoords){
            if(x < minScaleX){
                ++count;
            }
        }

        if(count > 0){
            curve->xCoords.erase(std::begin(curve->xCoords), std::begin(curve->xCoords) + count);
            curve->yCoords.erase(std::begin(curve->yCoords), std::begin(curve->yCoords) + count);
        }

        // erase values > maxScaleX
        int start = -1;
        for(size_t ii = 0; ii < curve->xCoords.size(); ++ii){
            if(curve->xCoords[ii] > maxScaleX){
                start = static_cast<int>(ii);
                break;
            }
        }

        if(start > -1){
            curve->xCoords.erase(std::begin(curve->xCoords) + start, std::end(curve->xCoords));
            curve->yCoords.erase(std::begin(curve->yCoords) + start, std::end(curve->yCoords));
        }

        // set first and last x values
        if(curve->xCoords.size() >= 2){
            curve->xCoords[0] = minScaleX;
            curve->xCoords[curve->xCoords.size()-1] = maxScaleX;
        }else if(curve->xCoords.size() == 1){

            if(std::abs(minScaleX-curve->xCoords[0]) < std::abs(maxScaleX-curve->xCoords[0])){
                curve->yCoords = {curve->yCoords[0], maxScaleY};
            }else{
                curve->yCoords = {minScaleY, curve->yCoords[0]};
            }
            curve->xCoords = {minScaleX, maxScaleX};

        }else{
            curve->xCoords = {minScaleX, maxScaleX};
            curve->yCoords = {minScaleY, maxScaleY};
        }
    }

    setAxisScale( QwtPlot::xBottom, minScaleX, maxScaleX);

    for(auto &curve : curves){
        if(curve->xCoords.size() > 1){
            curve->update_plot_samples();
        }
    }

    QwtPlot::replot();
    QwtPlot::repaint();
    emit settings_updated_signal();
}

void CurveW::set_y_range(qreal min, qreal max){

    if(min > max){
        min = max - 0.01;
    }

    minScaleY = min;
    maxScaleY = max;

    for(auto &curve : curves){
        for(auto &y: curve->yCoords){
            if(y < minScaleY){
                y = minScaleY;
            }
            if(y > maxScaleY){
                y = maxScaleY;
            }
        }
    }

    setAxisScale( QwtPlot::yLeft, minScaleY, maxScaleY);

    for(auto &curve : curves){
        if(curve->xCoords.size() > 1){
            curve->update_plot_samples();
        }
    }

    QwtPlot::replot();
    QwtPlot::repaint();
    emit settings_updated_signal();
}

void CurveW::set_pen(const QPen &pen, size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }

    curves[idCurve]->set_pen(pen);

    QwtPlot::replot();
    QwtPlot::repaint();
    emit settings_updated_signal();
}


void CurveW::mousePressEvent(QMouseEvent *event){

    if(!canvas()->geometry().contains(event->pos())){
        return;
    }

    auto pos = event->pos()-canvas()->pos();
    QPointF pt(
        1.*pos.x()/canvas()->width(),
        1. - 1.*pos.y()/canvas()->height()
    );

    const auto diffScaleX = maxScaleX - minScaleX;
    const auto diffScaleY = maxScaleY - minScaleY;
    const auto rPosX      = minScaleX + pt.x()*diffScaleX;
    const auto rPosY      = minScaleY + pt.y()*diffScaleY;

    if(event->button() == Qt::MouseButton::LeftButton){

        if(curves.size() > 0){
            curves[0]->insert_point(rPosX, rPosY);
            curves[0]->update_plot_samples();
        }

    }else if(event->button() == Qt::MouseButton::RightButton){
        if(curves.size() > 0){
            curves[0]->remove_point(pt, minScaleX, minScaleY, diffScaleX, diffScaleY);
            curves[0]->update_plot_samples();
        }
    }

    QwtPlot::replot();
    QwtPlot::repaint();
    emit data_updated_signal();
}

void CurveW::reset(size_t nbCurves){

    for(auto &curve : curves){
        curve->plot.detach();
    }
    curves.clear();

    for(size_t ii = 0; ii < nbCurves; ++ii){
        auto curve  = std::make_unique<Curve>();
        curve->xCoords = {minScaleX, maxScaleX};
        curve->yCoords = {minScaleY, maxScaleY};
        curve->update_plot_samples();
        curves.push_back(std::move(curve));
        curves[ii]->plot.attach(this);
    }

    QwtPlot::replot();
    QwtPlot::repaint();
    emit data_updated_signal();
}

void CurveW::add_point_to_end(double xOffset, double y, size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }

    if(curves[idCurve]->xCoords.size() > 0){
        curves[idCurve]->xCoords.push_back(curves[idCurve]->xCoords[curves[idCurve]->xCoords.size()-1]+xOffset);
    }else{
        curves[idCurve]->xCoords.push_back(xOffset);
    }
    curves[idCurve]->yCoords.push_back(y);
    curves[idCurve]->update_plot_samples();

    QwtPlot::replot();
    QwtPlot::repaint();
    emit data_updated_signal();

}

void CurveW::add_point(double x, double y, size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }

    size_t id = 0;
    for(size_t ii = 0; ii < curves[idCurve]->xCoords.size(); ++ii){
        if(x < curves[idCurve]->xCoords[ii]){
            id = ii;
            break;
        }
    }
    curves[idCurve]->xCoords.insert(std::begin(curves[idCurve]->xCoords) + id, x);
    curves[idCurve]->yCoords.insert(std::begin(curves[idCurve]->yCoords) + id, y);
    curves[idCurve]->update_plot_samples();

    QwtPlot::replot();
    QwtPlot::repaint();
    emit data_updated_signal();
}

void CurveW::set_points(std::vector<double> x, std::vector<double> y, size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }

    curves[idCurve]->xCoords = x;
    curves[idCurve]->yCoords = y;
    curves[idCurve]->update_plot_samples();

    QwtPlot::replot();
    QwtPlot::repaint();
    emit data_updated_signal();
}

void CurveW::set_first_y(double value, size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }

    curves[idCurve]->yCoords[0] = value;
    curves[idCurve]->update_plot_samples();

    QwtPlot::replot();
    QwtPlot::repaint();
    emit data_updated_signal();
}

void CurveW::set_last_y(double value, size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }

    curves[idCurve]->yCoords[curves[idCurve]->yCoords.size()-1] = value;
    curves[idCurve]->update_plot_samples();

    QwtPlot::replot();
    QwtPlot::repaint();
    emit data_updated_signal();
}

void CurveW::set_fitted_state(bool state, size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }
    curves[idCurve]->set_fitted_state(state);

    QwtPlot::replot();
    QwtPlot::repaint();
    emit settings_updated_signal();
}

void CurveW::remove_symbol(size_t idCurve){

    if(idCurve >= curves.size()){
        return;
    }

    curves[idCurve]->plot.setSymbol(nullptr);

    QwtPlot::replot();
    QwtPlot::repaint();
    emit settings_updated_signal();
}

