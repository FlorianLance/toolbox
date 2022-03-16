
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

#include "qt_ui.hpp"

using namespace tool;
using namespace tool::ui;

void style::generate_tab_widget_style(QTabWidget *tab, QString id, int widthBorder, QTabWidget::TabPosition position){

    Q_UNUSED(position)
    QString tabWidgetObjectName = id;
    QString tabBarObjectName = id + "_bar";

    tab->setObjectName(tabWidgetObjectName);
    tab->tabBar()->setObjectName(tabBarObjectName);

    QString lB = QString::number(widthBorder);

    QString style =
    "QTabWidget::tab-bar#"+ tabWidgetObjectName +
    "{"
    "   alignment: left;"
    "}"
    "QTabWidget::pane#"+ tabWidgetObjectName +
    "{"
    "   border: "+lB+"px solid rgb(0,0,0);"
    "   border-radius: 0px;"
    "   border-top-left-radius: 0px;"
    "}"
    "QTabBar::tab#"+ tabBarObjectName +
    "{"
    "   background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,stop: 0 #f6f7fa, stop: 1 #dadbde);"
    "   border: "+lB+"px solid rgb(0,0,0);"
    "   border-right: 0px;"
    "   border-radius: 0px;"
    "   min-width:30px;"
    "   padding: 2px;"
    "}"
    "QTabBar::tab#"+ tabBarObjectName + ":selected"
    "{"
    "   color : rgb(0,106,255);"
    "   font: bold 11pt Calibri;"
    "   background-color:  rgb(220,220,220);"
    "}"
    "QTabBar::tab#"+ tabBarObjectName + ":!selected"
    "{"
    "   color : rgb(127,180,255);"
    "   font: bold 11pt Calibri;"
    "   background-color:  rgb(220,220,220);"
    "   background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,stop: 0 #f6f7fa, stop: 1 #dadbde);"
    "}"
    "QTabBar::tab#" + tabBarObjectName + ":hover"
    "{"
    "   color : rgb(0,106,255);"
    "   font: bold 11pt Calibri;"
    "   background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,stop: 0 #dadbde, stop: 1 #f6f7fa);"
    "}"
    "QTabBar::tab#" + tabBarObjectName + ":first"
    "{"
    "   border: "+lB+"px solid rgb(0,0,0);"
    "   border-right: 0px;"
    "   border-bottom: 0px;"
    "   min-height:200px;"
    "}"
    "QTabBar::tab#" + tabBarObjectName + ":last"
    "{"
    "   border: "+lB+"px solid rgb(0,0,0);"
    "   border-right: 0px;"
    "   border-top: 4px solid rgb(0,0,0);"
    "   min-height:150px;"
    "}";

    tab->setStyleSheet(style);
}


QFrame *W::init(QFrame *f, bool on, bool enabled){
    const QSignalBlocker blocker(f);
    f->setStyleSheet(on ? "background-color:green;" : "background-color:red;");
    f->setEnabled(enabled);
    return f;
}

QComboBox *W::init(QComboBox *cb, QStringList items, int index, bool enabled){
    const QSignalBlocker blocker(cb);
    cb->addItems(items);
    cb->setEnabled(enabled);
    if(index < cb->count()){
        cb->setCurrentIndex(index);
    }
    return cb;
}

QCheckBox *W::init(QCheckBox *cb, const QString &txt, bool checked, bool enabled){
    const QSignalBlocker blocker(cb);
    cb->setText(txt);
    cb->setChecked(checked);
    cb->setEnabled(enabled);
    return cb;
}

QRadioButton *W::init(QRadioButton *rb, const QString &txt, bool checked, bool enabled){
    const QSignalBlocker blocker(rb);
    rb->setText(txt);
    rb->setEnabled(enabled);
    rb->setChecked(checked);
    return rb;
}

QSlider *W::init(QSlider *sl, MinV<int> min, V<int> value, MaxV<int> max, StepV<int> singleStep, bool enabled){
    const QSignalBlocker blocker(sl);
    sl->setMinimum(min.v);
    sl->setMaximum(max.v);
    sl->setValue(value.v);
    sl->setSingleStep(singleStep.v);
    sl->setEnabled(enabled);
    return sl;
}

QPushButton *W::init(QPushButton *pb, const QString &txt, bool enabled){
    const QSignalBlocker blocker(pb);
    pb->setText(txt);
    pb->setEnabled(enabled);
    return pb;
}

QTextEdit *W::init(QTextEdit *te, const QString &txt, Qt::TextFormat tf, bool enabled){

    const QSignalBlocker blocker(te);
    if(tf == Qt::TextFormat::PlainText){
        te->setPlainText(txt);
    }else if(tf == Qt::TextFormat::RichText){
        te->setHtml(txt);
    }else{
        te->setMarkdown(txt);
    }
    te->setEnabled(enabled);
    return te;
}

QLineEdit *W::init(QLineEdit *le, const QString &txt, bool enabled){
    const QSignalBlocker blocker(le);
    le->setText(txt);
    le->setEnabled(enabled);
    return le;
}

QLabel *W::init_label(QLabel *la, const QString &txt, bool enabled){
    const QSignalBlocker blocker(la);
    la->setText(txt);
    la->setEnabled(enabled);
    return la;
}

QSpinBox *W::init(QSpinBox *sb, MinV<int> min, V<int> value, MaxV<int> max, StepV<int> singleStep, bool enabled){
    const QSignalBlocker blocker(sb);
    sb->setMinimum(min.v);
    sb->setMaximum(max.v);
    sb->setValue(value.v);
    sb->setSingleStep(singleStep.v);
    sb->setEnabled(enabled);
    return sb;
}

QDoubleSpinBox *W::init(QDoubleSpinBox *dsb, MinV<qreal> min, V<qreal> value, MaxV<qreal> max, StepV<qreal> singleStep, int decimals, bool enabled){
    const QSignalBlocker blocker(dsb);
    dsb->blockSignals(true);
    dsb->setDecimals(decimals);
    dsb->setMinimum(min.v);
    dsb->setMaximum(max.v);
    dsb->setValue(value.v);
    dsb->setSingleStep(singleStep.v);
    dsb->setEnabled(enabled);
    dsb->blockSignals(false);
    return dsb;
}

QLabel *W::txt(const QString &txt){return new QLabel(txt);}

QLabel *W::icon(QIcon icon, QSize size){
    auto l = new QLabel();
    l->setPixmap(icon.pixmap(size));
    return l;
}

QLabel *W::icon(QColor color, QSize size){
    auto l = new QLabel();
    auto p = QPixmap(size);
    p.fill(color);
    l->setPixmap(std::move(p));
    return l;
}

QFrame *W::horizontal_line(){
    auto line = new QFrame();
    line->setObjectName(QString::fromUtf8("line"));
    line->setGeometry(QRect(320, 150, 118, 3));
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    return line;
}

void L::stretch(QLayout *layout){
    auto hbl = qobject_cast<QHBoxLayout*>(layout);
    if(hbl){
        hbl->addStretch();
        return;
    }
    auto vbl = qobject_cast<QVBoxLayout*>(layout);
    if(vbl){
        vbl->addStretch();
        return;
    }
}

QFrame *F::gen_frame(QLayout *layout, std::vector<std::pair<QWidget *, int> > widgetsStretch, int stretchFactor, LMarginsD margins, QFrame::Shape shape, QFrame::Shadow shadow){


    QFrame *frame = new QFrame();
    frame->setLayout(layout);
    layout->setContentsMargins(margins.left, margins.top, margins.right, margins.bottom);
    layout->setSpacing(margins.inter);

    for(auto widget : widgetsStretch){
        frame->layout()->addWidget(widget.first);
    }

    if(auto hbL = qobject_cast<QHBoxLayout*>(frame->layout()); hbL != nullptr){
        for(size_t ii = 0; ii < widgetsStretch.size(); ++ii){
            hbL->setStretch(static_cast<int>(ii), widgetsStretch[ii].second);
        }
        if(stretchFactor > 0){
            hbL->addStretch(stretchFactor);
        }

    }else if(auto vbL = qobject_cast<QVBoxLayout*>(frame->layout()); hbL != nullptr){
        for(size_t ii = 0; ii < widgetsStretch.size(); ++ii){
            vbL->setStretch(static_cast<int>(ii), widgetsStretch[ii].second);
        }
        if(stretchFactor > 0){
            vbL->addStretch(stretchFactor);
        }
    }

    frame->setFrameShadow(shadow);
    frame->setFrameShape(shape);

    return frame;
}

QFrame *F::gen_frame(QLayout *layout, std::vector<QWidget *> widgets, LStretchD strech, LMarginsD margins, QFrame::Shape shape, QFrame::Shadow shadow){

    QFrame *frame = new QFrame();
    frame->setLayout(layout);
    layout->setContentsMargins(margins.left, margins.top, margins.right, margins.bottom);
    layout->setSpacing(margins.inter);

    for(auto widget : widgets){
        frame->layout()->addWidget(widget);
    }

    if(auto hbL = qobject_cast<QHBoxLayout*>(frame->layout()); hbL != nullptr){

        for(size_t ii = 0; ii < strech.elementsStretch.size(); ++ii){
            if(ii < static_cast<size_t>(layout->count())){
                hbL->setStretch(static_cast<int>(ii), strech.elementsStretch[ii]);
            }
        }

        if(strech.spacerStretch > 0){
            hbL->addStretch(strech.spacerStretch);
        }


    }else if(auto vbL = qobject_cast<QVBoxLayout*>(frame->layout()); vbL != nullptr){

        for(size_t ii = 0; ii < strech.elementsStretch.size(); ++ii){
            if(ii < static_cast<size_t>(layout->count())){
                vbL->setStretch(static_cast<int>(ii), strech.elementsStretch[ii]);
            }
        }

        if(strech.spacerStretch > 0){
            vbL->addStretch(strech.spacerStretch);
        }
    }

    frame->setFrameShadow(shadow);
    frame->setFrameShape(shape);

    return frame;
}

QFrame *F::gen(QLayout *layout, std::vector<QWidget *> widgets, LStretch stretch, LMargins margins, QFrame::Shape shape, QFrame::Shadow shadow){
    QFrame *frame = new QFrame();
    frame->setLayout(layout);
    if(!margins.v){
        layout->setContentsMargins(0,0,0,0);
    }

    for(auto widget : widgets){
        frame->layout()->addWidget(widget);
    }

    auto *hbL = qobject_cast<QHBoxLayout*>(frame->layout());
    if(hbL && stretch.v){
        hbL->addStretch(20);
    }
    auto *vbL = qobject_cast<QVBoxLayout*>(frame->layout());
    if(vbL && stretch.v){
        vbL->addStretch(20);
    }
    frame->setFrameShadow(shadow);
    frame->setFrameShape(shape);

    return frame;
}
