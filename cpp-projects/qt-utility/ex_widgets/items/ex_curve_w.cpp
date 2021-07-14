
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

#include "ex_curve_w.hpp"


using namespace tool;
using namespace tool::ui;
using namespace tool::ex;

ExCurveW *ExCurveW::init_widget(QString title, QString xTitle, QString yTitle, geo::Pt2d xRange, geo::Pt2d yRange, bool enabled){

    // generate widgets
    curveW = new CurveW();
    auto f1  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Range X axis:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto f2  = ui::F::gen_frame(ui::L::HB(), { {ui::W::txt("min"),1}, {minX(), 10}, {ui::W::txt("max"),1}, {maxX(),10}},   0, LMarginsD{0,0,0,0,2});
    auto f3  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Range Y axis:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto f4  = ui::F::gen_frame(ui::L::HB(), { {ui::W::txt("min"),1}, {minY(), 10}, {ui::W::txt("max"),1}, {maxY(),10}},   0, LMarginsD{0,0,0,0,2});
    auto f5  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Y values:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto f6  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("first"),1},{firstY(),10}, {ui::W::txt("last"),1},{lastY(), 10}},  0, LMarginsD{0,0,0,0,2});
    auto f7  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Add specific point:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto f8  = ui::F::gen_frame(ui::L::HB(), { {ui::W::txt("X"),1}, {addX(), 10}, {ui::W::txt("Y"),1}, {addY(),10}},   0, LMarginsD{0,0,0,0,2});
    auto f9  = ui::F::gen_frame(ui::L::HB(), {{addPointB  = new QPushButton("Add point"),1}, {resetB  = new QPushButton("Reset points"),2}},  0, LMarginsD{0,0,0,0,2});
    auto f10 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Others:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto f11 = ui::F::gen_frame(ui::L::HB(), {{fitted(),1}},  0, LMarginsD{0,0,0,0,2});
    auto f12 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Infos:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto f13 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("Add point with left click"),1}},  0, LMarginsD{0,0,0,0,2});
    auto f14 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("Remove point with right click"),1}},  0, LMarginsD{0,0,0,0,2});
    auto fControl = ui::F::gen_frame(ui::L::VB(), {f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,f13,f14},
            LStretchD{}, LMarginsD{4,4,4,4}, QFrame::Shape::Box);

    // init widets
    auto diffRangeX = xRange.y() - xRange.x();
    auto diffRangeY = yRange.y() - yRange.x();
    // # range
    minX.init_widget(MinV<qreal>{-100000.}, V<qreal>{xRange.x()}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeX*0.01}, 3);
    maxX.init_widget(MinV<qreal>{-100000.}, V<qreal>{xRange.y()}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeX*0.01}, 3);
    minY.init_widget(MinV<qreal>{-100000.}, V<qreal>{yRange.x()}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeY*0.01}, 3);
    maxY.init_widget(MinV<qreal>{-100000.}, V<qreal>{yRange.y()}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeY*0.01}, 3);
    minX.editFinishedSignal = false;
    maxX.editFinishedSignal = false;
    minY.editFinishedSignal = false;
    maxY.editFinishedSignal = false;
    // # first/last
    firstY.init_widget(MinV<qreal>{-100000.}, V<qreal>{yRange.x()}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeY*0.01}, 3);
    lastY.init_widget(MinV<qreal>{-100000.},  V<qreal>{yRange.y()}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeY*0.01}, 3);
    // # add
    addX.init_widget(MinV<qreal>{-100000.}, V<qreal>{diffRangeX*0.5}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeY*0.01}, 3);
    addY.init_widget(MinV<qreal>{-100000.},  V<qreal>{diffRangeX*0.5}, MaxV<qreal>{100000.}, StepV<qreal>{diffRangeY*0.01}, 3);
    // # actions
    resetB->setMinimumWidth(150);
    fitted.init_widget("fitted ", true);

    // add widgets to layout
    auto mainL = new QHBoxLayout();
    w->setLayout(mainL);
    mainL->addWidget(ui::F::gen(ui::L::HB(), {curveW}, LStretch{false}, LMargins{false}, QFrame::Box));
    mainL->addWidget(fControl);
    mainL->setStretch(0, 3);
    mainL->setStretch(1, 1);

    resetB->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    // # curve
    curveW->set_title(title);
    curveW->set_x_title(xTitle);
    curveW->set_y_title(yTitle);
    curveW->set_x_range(xRange.x(), xRange.y());
    curveW->set_y_range(yRange.x(), yRange.y());
    curveW->set_points({xRange.x(), xRange.y()}, {yRange.x(), yRange.y()});


    w->setEnabled(enabled);

    return this;
}

void ExCurveW::init_connection(const QString &nameParam){

    minX.init_connection(QSL("min_x"));
    maxX.init_connection(QSL("max_x"));
    minY.init_connection(QSL("min_y"));
    maxY.init_connection(QSL("max_y"));
    firstY.init_connection(QSL("first_y"));
    lastY.init_connection(QSL("last_y"));
    fitted.init_connection(QSL("fitted"));

    connect(curveW, &CurveW::data_updated_signal, this, [=]{
        emit ui_change_signal(nameParam);
    });
    connect(curveW, &CurveW::settings_updated_signal, this, [=]{
        emit ui_change_signal(nameParam);
    });

    connect(&fitted, &ExCheckBoxW::ui_change_signal, this, [&,nameParam]{
        curveW->set_fitted_state(fitted.w->isChecked());
    });

    connect(&minX,  &ExDoubleSpinBoxW::ui_change_signal, this, [&,nameParam]{

        minX.blockSignals(true);
        if(minX.w->value() > maxX.w->value()){
           minX.w->setValue(maxX.w->value()-0.01);
        }
        minX.blockSignals(false);

        curveW->set_x_range(minX.w->value(), maxX.w->value());
    });
    connect(&maxX,  &ExDoubleSpinBoxW::ui_change_signal, this, [&,nameParam]{

        maxX.blockSignals(true);
        if(minX.w->value() > maxX.w->value()){
           maxX.w->setValue(minX.w->value()+0.01);
        }
        maxX.blockSignals(false);

        curveW->set_x_range(minX.w->value(), maxX.w->value());
    });
    connect(&minY,  &ExDoubleSpinBoxW::ui_change_signal, this, [&,nameParam]{

        minY.blockSignals(true);
        if(minY.w->value() > maxY.w->value()){
           minY.w->setValue(maxY.w->value()-0.01);
        }
        minY.blockSignals(false);

        firstY.blockSignals(true);
        firstY.blockSignals(true);
        const auto diff = maxY.w->value() - minY.w->value();
        firstY.w->setRange(minY.w->value(), maxY.w->value());
        lastY.w->setRange(minY.w->value(), maxY.w->value());
        firstY.w->setSingleStep(diff*0.01);
        lastY.w->setSingleStep(diff*0.01);
        firstY.blockSignals(false);
        firstY.blockSignals(false);

        curveW->set_y_range(minY.w->value(), maxY.w->value());
    });
    connect(&maxY,  &ExDoubleSpinBoxW::ui_change_signal, this, [&,nameParam]{

        maxY.blockSignals(true);
        if(minY.w->value() > maxY.w->value()){
           maxY.w->setValue(minY.w->value()+0.01);
        }
        maxY.blockSignals(false);

        firstY.blockSignals(true);
        lastY.blockSignals(true);

        const auto diff = maxY.w->value() - minY.w->value();
        firstY.w->setRange(minY.w->value(), maxY.w->value());
        lastY.w->setRange(minY.w->value(), maxY.w->value());        
        firstY.w->setSingleStep(diff*0.01);
        lastY.w->setSingleStep(diff*0.01);

        addX.w->setRange(minX.w->value(), maxX.w->value());
        addY.w->setRange(minY.w->value(), maxY.w->value());
        addX.w->setSingleStep(diff*0.01);
        addY.w->setSingleStep(diff*0.01);

        firstY.blockSignals(false);
        lastY.blockSignals(false);

        curveW->set_y_range(minY.w->value(), maxY.w->value());
    });
    connect(&firstY,  &ExDoubleSpinBoxW::ui_change_signal, this, [&,nameParam]{
        double v = firstY.w->value();
        if(v < minY.w->value()){
            v = minY.w->value();
        }
        if(v > maxY.w->value()){
            v = maxY.w->value();
        }
        curveW->set_first_y(v);
    });
    connect(&lastY,  &ExDoubleSpinBoxW::ui_change_signal, this, [&,nameParam]{
        double v = lastY.w->value();
        if(v < minY.w->value()){
            v = minY.w->value();
        }
        if(v > maxY.w->value()){
            v = maxY.w->value();
        }
        curveW->set_last_y(v);
    });

    connect(addPointB,    &QPushButton::clicked,   this, [&]{
        curveW->add_point(addX.w->value(), addY.w->value());
    });

    connect(resetB,    &QPushButton::clicked,   this, [&]{

        firstY.blockSignals(true);
        lastY.blockSignals(true);
        firstY.w->setValue(minY.w->value());
        lastY.w->setValue(maxY.w->value());
        firstY.blockSignals(false);
        lastY.blockSignals(false);

        curveW->reset();
    });
}

void ExCurveW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    auto pts = arg.to_curve_value();

    if(pts.first.size() < 2){
        pts.first = {0,1};
    }
    if(pts.second.size() < 2){
        pts.second = {0,1};
    }

    auto minXV = minX.w->value();
    auto maxXV = maxX.w->value();

    auto minYV = minY.w->value();
    auto maxYV = maxY.w->value();

    for(const auto & v : pts.first){
        if(v < minXV){
            minXV = v;
        }
        if(v > maxXV){
            maxXV = v;
        }
    }

    for(const auto & v : pts.second){
        if(v < minYV){
            minYV = v;
        }
        if(v > maxYV){
            maxYV = v;
        }
    }

    auto diffX = maxXV - minXV;
    auto diffY = maxYV - minYV;

    minX.blockSignals(true);
    maxX.blockSignals(true);
    minY.blockSignals(true);
    maxY.blockSignals(true);
    firstY.blockSignals(true);
    lastY.blockSignals(true);

    minX.w->setValue(minXV);
    maxX.w->setValue(maxXV);
    minY.w->setValue(minYV);
    maxY.w->setValue(maxYV);
    firstY.w->setValue(pts.second[0]);
    lastY.w->setValue(pts.second[pts.second.size()-1]);

    minX.w->setSingleStep(diffX*0.01);
    maxX.w->setSingleStep(diffX*0.01);
    minY.w->setSingleStep(diffY*0.01);
    maxY.w->setSingleStep(diffY*0.01);
    firstY.w->setSingleStep(diffY*0.01);
    lastY.w->setSingleStep(diffY*0.01);

    firstY.blockSignals(false);
    lastY.blockSignals(false);
    minY.blockSignals(false);
    maxY.blockSignals(false);
    minX.blockSignals(false);
    maxX.blockSignals(false);

    curveW->set_x_range(minXV, maxXV);
    curveW->set_y_range(minYV, maxYV);
    curveW->set_points(std::move(pts.first), std::move(pts.second));

    w->blockSignals(false);
}

Arg ExCurveW::convert_to_arg() const{

    Arg arg = ExItemW::convert_to_arg();
    arg.init_from_curve(&curveW->curves[0]->xCoords, &curveW->curves[0]->yCoords, " ");
    return arg;
}


