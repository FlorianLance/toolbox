
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

    // right layout
    // # all curves
    auto fg0  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Range X axis:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto fg1  = ui::F::gen_frame(ui::L::HB(), { {ui::W::txt("min"),1}, {minX(), 10}, {ui::W::txt("max"),1}, {maxX(),10}},   0, LMarginsD{0,0,0,0,2});
    auto fg2  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Range Y axis:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto fg3  = ui::F::gen_frame(ui::L::HB(), { {ui::W::txt("min"),1}, {minY(), 10}, {ui::W::txt("max"),1}, {maxY(),10}},   0, LMarginsD{0,0,0,0,2});
    auto allCurves = ui::F::gen_frame(ui::L::VB(), {fg0,fg1,fg2,fg3}, LStretchD{}, LMarginsD{4,4,4,4}, QFrame::Shape::Box);
    // # current curve
    auto fc0  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Current curve:</b>"),1},{currentCurveId(),3}},   0, LMarginsD{0,0,0,0,2});
    auto fc1  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Preset:</b>"),1},{type(),3}},   0, LMarginsD{0,0,0,0,2});
    auto fc2  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Y values:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto fc3  = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("first"),1},{firstY(),10}, {ui::W::txt("last"),1},{lastY(), 10}},  0, LMarginsD{0,0,0,0,2});
    auto fc4 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Add specific point:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto fc5 = ui::F::gen_frame(ui::L::HB(), { {ui::W::txt("X"),1}, {addX(), 10}, {ui::W::txt("Y"),1}, {addY(),10}},   0, LMarginsD{0,0,0,0,2});
    auto fc6 = ui::F::gen_frame(ui::L::HB(), {{addPointB  = new QPushButton("Add point"),1}, {resetB  = new QPushButton("Reset points"),2}},  0, LMarginsD{0,0,0,0,2});
    auto fc7 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Others:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto fc8 = ui::F::gen_frame(ui::L::HB(), {{fitted(),1}},  0, LMarginsD{0,0,0,0,2});
    auto fc9 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("<b>Infos:</b>"),1}},   0, LMarginsD{0,0,0,0,2});
    auto fc10 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("Add point with left click"),1}},  0, LMarginsD{0,0,0,0,2});
    auto fc11 = ui::F::gen_frame(ui::L::HB(), {{ui::W::txt("Remove point with right click"),1}},  0, LMarginsD{0,0,0,0,2});
    auto currentCurve = ui::F::gen_frame(ui::L::VB(), {fc0,fc1,fc2,fc3,fc4,fc5,fc6,fc7,fc8,fc9,fc10,fc11}, LStretchD{}, LMarginsD{4,4,4,4}, QFrame::Shape::Box);
    // # container
    auto rightL = new QVBoxLayout();
    rightL->addWidget(allCurves);
    rightL->addWidget(currentCurve);
    rightL->setContentsMargins(0,0,0,0);
    auto rightW = new QWidget();
    rightW->setLayout(rightL);

    // main layout
    auto mainL = new QHBoxLayout();
    w->setLayout(mainL);
    mainL->addWidget(ui::F::gen(ui::L::HB(), {curveW}, LStretch{false}, LMargins{false}, QFrame::Box));
    mainL->addWidget(rightW);
    mainL->setStretch(0, 3);
    mainL->setStretch(1, 1);

    // init widets
    QStringList items;
    items << "custom"        << "y=x"             <<
             "easeInSine"    << "easeInCubic"     << "easeInQuint"    << "easeInCirc"    << "easeInElastic"    <<
             "easeInQuad"    << "easeInQuart"     << "easeInExpo"     << "easeInBack"    << // "easeInBounce"     <<
             "easeOutSine"   << "easeOutCubic"    << "easeOutQuint"   << "easeOutCirc"   << "easeOutElastic"   <<
             "easeOutQuad"   << "easeOutQuart"    << "easeOutExpo"    << "easeOutBack"   << // "easeOutBounce"    <<
             "easeInOutSine" << "easeInOutCubic"  << "easeOutInQuint" << "easeInOutCirc" << "easeInOutElastic" <<
             "easeInOutQuad" << "easeInOutQuart"  << "easeInOutExpo"  << "easeInOutBack";// <<  "easeInOutBounce";
    type.init_widget(items, Index{0});

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

    // # curve id
    currentCurveId.w->setMinimum(0);
    currentCurveId.w->setMaximum(10);
    // # actions
    resetB->setMinimumWidth(150);
    fitted.init_widget("fitted ", true);
    resetB->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    // # curve
    curveW->set_title(title);
    curveW->set_x_title(xTitle);
    curveW->set_y_title(yTitle);
    curveW->set_x_range(xRange.x(), xRange.y());
    curveW->set_y_range(yRange.x(), yRange.y());
    curveW->set_points({xRange.x(), xRange.y()}, {yRange.x(), yRange.y()}, 0);

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
    type.init_connection(QSL("type"));

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
        curveW->set_first_y(v, currentCurveId.w->value());
    });
    connect(&lastY,  &ExDoubleSpinBoxW::ui_change_signal, this, [&,nameParam]{


        double v = lastY.w->value();
        if(v < minY.w->value()){
            v = minY.w->value();
        }
        if(v > maxY.w->value()){
            v = maxY.w->value();
        }
        curveW->set_last_y(v, currentCurveId.w->value());
    });

    connect(addPointB,    &QPushButton::clicked,   this, [&]{

        curveW->add_point(addX.w->value(), addY.w->value(), currentCurveId.w->value());
    });

    connect(resetB,    &QPushButton::clicked,   this, [&]{

        type.blockSignals(true);
        type.w->setCurrentIndex(0);
        type.blockSignals(false);

        firstY.blockSignals(true);
        lastY.blockSignals(true);
        firstY.w->setValue(minY.w->value());
        lastY.w->setValue(maxY.w->value());
        firstY.blockSignals(false);
        lastY.blockSignals(false);

        curveW->reset();
    });

    connect(&type, &ExComboBoxTextW::ui_change_signal, this, [&,nameParam]{
        auto id = type.w->currentIndex();
        curveW->set_type(static_cast<Curve::Type>(id), currentCurveId.w->value());
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

    const auto stepX =  diffX*0.01;
    const auto stepY =  diffY*0.01;
    minX.w->setSingleStep(stepX);
    maxX.w->setSingleStep(stepX);
    minY.w->setSingleStep(stepY);
    maxY.w->setSingleStep(stepY);
    firstY.w->setSingleStep(stepY);
    lastY.w->setSingleStep(stepY);

    firstY.blockSignals(false);
    lastY.blockSignals(false);
    minY.blockSignals(false);
    maxY.blockSignals(false);
    minX.blockSignals(false);
    maxX.blockSignals(false);

//    qDebug() << "range";
    curveW->set_x_range(minXV, maxXV);
    curveW->set_y_range(minYV, maxYV);

//    qDebug() << "diffX " << diffX;
//    qDebug() << "diffY " << diffY;
//    qDebug() << "pts.first " << pts.first.size();
//    qDebug() << "pts.second " << pts.second.size();

    curveW->set_points(std::move(pts.first), std::move(pts.second), 0);
//    qDebug() << "--";

    w->blockSignals(false);

//    qDebug() << "ExCurveW::update_from_arg end";
}

Arg ExCurveW::convert_to_arg() const{

    Arg arg = ExItemW::convert_to_arg();
    arg.init_from_curve(&curveW->curves[0]->xCoords, &curveW->curves[0]->yCoords, " ");
    return arg;
}


