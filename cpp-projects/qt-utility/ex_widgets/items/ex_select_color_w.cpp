
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

ExSelectColorW::ExSelectColorW() : ExItemW<QPushButton>(UiType::Color_pick){
//    m_colDialog.setOption(QColorDialog::ShowAlphaChannel);
    connect(w.get(), &QPushButton::clicked, &m_colDialog, &QColorDialog::show);
    w->setMinimumSize(QSize(50,50));
    w->setMaximumSize(QSize(50,50));
//    hintHeightSize = 70;
}

ExSelectColorW::~ExSelectColorW(){
    m_colDialog.close();
}

QColor ExSelectColorW::current_color() const{
    return m_colDialog.currentColor();
}

ExSelectColorW *ExSelectColorW::init_widget(QString dialogName, QColor color, bool enabled){

    m_colDialog.setCurrentColor(color);
    m_colDialog.setWindowTitle(dialogName);
    m_colDialog.setOptions(QColorDialog::ShowAlphaChannel | QColorDialog::NoButtons);
    m_colDialog.setModal(true);
    w->setEnabled(enabled);

    QPixmap p(50,50);
    p.fill(color);
    w->setIcon(QIcon(p));
    w->setIconSize(QSize(40,40));

    return this;
}

void ExSelectColorW::init_connection(const QString &nameParam){
    connect(&m_colDialog, &QColorDialog::currentColorChanged, this, [&,nameParam](const QColor &c){
        QPixmap p(50,50);
        p.fill(c);
        w->setIcon(QIcon(p));
        w->setIconSize(QSize(40,40));
        emit ui_change_signal(nameParam);}
    );
}

void ExSelectColorW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    QColor c = arg.to_color_value();
    m_colDialog.setCurrentColor(c);

    QPixmap p(50,50);
    p.fill(c);
    w->setIcon(QIcon(p));
    w->setIconSize(QSize(40,40));

    if(generatorName.length() > 0){
        // ...
    }

    w->blockSignals(false);
}

Arg ExSelectColorW::convert_to_arg() const{

    Arg arg = ExItemW::convert_to_arg();
    arg.init_from(m_colDialog.currentColor());

    // generator
    if(generatorName.length() > 0){
        // ...
    }
    return arg;
}

void ExSelectColorW::close(){
    m_colDialog.close();
}
#include "moc_ex_select_color_w.cpp"
