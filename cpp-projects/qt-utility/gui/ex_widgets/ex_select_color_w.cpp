
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

#include "ex_select_color_w.hpp"

using namespace tool::str;
using namespace tool::ex;

ExSelectColorW::ExSelectColorW(QString name) : ExItemW<QPushButton>(UiType::Color_pick, name){

    p = std::make_unique<QPixmap>(50,50);
    connect(w.get(), &QPushButton::clicked, &m_colDialog, &QColorDialog::show);
    w->setMinimumSize(QSize(50,50));
    w->setMaximumSize(QSize(50,50));
    m_colDialog.setOptions(QColorDialog::ShowAlphaChannel | QColorDialog::NoButtons);
    m_colDialog.setModal(true);
//    hintHeightSize = 70;

    connect(&m_colDialog, &QColorDialog::currentColorChanged, this, [&](const QColor &c){
        set_color(c);
        trigger_ui_change();
    });
}

ExSelectColorW::~ExSelectColorW(){
    m_colDialog.close();
}

QColor ExSelectColorW::current_color() const{
    return m_colDialog.currentColor();
}

void ExSelectColorW::set_color(const QColor &color){
    m_colDialog.setCurrentColor(color);

    p->fill(color);
    w->setIcon(QIcon(*p));
    w->setIconSize(QSize(40,40));
}

ExSelectColorW *ExSelectColorW::init_widget(QString dialogName, QColor color, bool enabled){

    m_colDialog.setWindowTitle(dialogName);
    set_color(color);
    w->setEnabled(enabled);
    return this;
}

void ExSelectColorW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(arg.generator.has_value()){
        init_widget("Select color", arg.to_color_value());
    }else{
        set_color(arg.to_color_value());
    }

    w->blockSignals(false);
}

Arg ExSelectColorW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from(m_colDialog.currentColor());

    // generator
    if(hasGenerator){
        // ...
    }
    return arg;
}

void ExSelectColorW::close(){
    m_colDialog.close();
}

//#include "moc_ex_select_color_w.cpp"
