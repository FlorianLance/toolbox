

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
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QRadioButton>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QTextEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QDebug>

// local
#include "qt_str.hpp"

namespace tool {

    // names types
    template<typename T>
    struct MinV {T v;};
    template<typename T>
    struct V{T v;};
    struct VTxt{QString v;};
    struct Name{QString v;};
    struct Sep{QString v;};
    template<typename T>
    struct MaxV {T v;};
    template<typename T>
    struct StepV {T v;};
    struct LMargins{bool v;};
    struct LStretch {bool v;};


    struct LMarginsD{
        int left = 2;
        int top = 2;
        int right = 2;
        int bottom = 2;
        int inter = 2;
    };

    struct LStretchD{
        int spacerStretch = 10;
        std::vector<int> elementsStretch;
    };


    struct DsbSettings{
        MinV<qreal> min = {-1000.};
        V<qreal> value = {0.};
        MaxV<qreal> max = {1000.};
        StepV<qreal> singleStep = {0.1};
        int decimals = 2;
    };

    struct Vector2dSettings{
        DsbSettings sX = {MinV<qreal>{-1000.}, V<qreal>{0}, MaxV<qreal>{1000.}, StepV<qreal>{0.01}, 2};
        DsbSettings sY = {MinV<qreal>{-1000.}, V<qreal>{0}, MaxV<qreal>{1000.}, StepV<qreal>{0.01}, 2};
    };

    struct Vector3dSettings{
        DsbSettings sX = {MinV<qreal>{-1000.}, V<qreal>{0}, MaxV<qreal>{1000.}, StepV<qreal>{0.01}, 2};
        DsbSettings sY = {MinV<qreal>{-1000.}, V<qreal>{0}, MaxV<qreal>{1000.}, StepV<qreal>{0.01}, 2};
        DsbSettings sZ = {MinV<qreal>{-1000.}, V<qreal>{0}, MaxV<qreal>{1000.}, StepV<qreal>{0.01}, 2};
    };
}

namespace tool::ui {

    template<class T> class Blocker {
        T *blocked;
        bool previous;
    public:
        Blocker(T *blocked) : blocked(blocked), previous(blocked->blockSignals(true)) {}
        ~Blocker() { blocked->blockSignals(previous); }
        T *operator->() { return blocked; }
    };

    template<class T> inline Blocker<T> w_blocking(T *blocked) {
        return Blocker<T>(blocked);
    }


    struct style{
        static void generate_tab_widget_style(QTabWidget *tab, QString id, int widthBorder=3, QTabWidget::TabPosition position = QTabWidget::West);
    };

    struct W{
        static QFrame* init(QFrame *f, bool on, bool enabled = true){
            f->setStyleSheet(on ? "background-color:green;" : "background-color:red;");
            f->setEnabled(enabled);
            return f;
        }
        static QComboBox* init(QComboBox *cb, QStringList items, int index=0, bool enabled = true){
            cb->addItems(items);
            cb->setEnabled(enabled);
            if(index < cb->count()){
                cb->setCurrentIndex(index);
            }
            return cb;
        }

        static QCheckBox* init(QCheckBox *cb, QString txt, bool checked, bool enabled = true){
            cb->setText(txt);
            cb->setChecked(checked);
            cb->setEnabled(enabled);
            return cb;
        }
        static QRadioButton* init(QRadioButton *rb, QString txt, bool checked, bool enabled = true){
            rb->setText(txt);
            rb->setEnabled(enabled);
            rb->setChecked(checked);
            return rb;
        }
        static QSlider* init(QSlider *sl, MinV<int> min, V<int> value, MaxV<int> max, StepV<int> singleStep, bool enabled = true){
            sl->setMinimum(min.v);
            sl->setMaximum(max.v);
            sl->setValue(value.v);
            sl->setSingleStep(singleStep.v);
            sl->setEnabled(enabled);
            return sl;
        }
        static QPushButton* init(QPushButton *pb, QString txt, bool enabled = true){
            pb->setText(txt);
            pb->setEnabled(enabled);
            return pb;
        }
        static QTextEdit* init(QTextEdit *te, QString txt, bool enabled = true){
            te->setText(txt);
            te->setEnabled(enabled);
            return te;
        }
        static QLineEdit* init(QLineEdit *le, QString txt, bool enabled = true){
            le->setText(txt);
            le->setEnabled(enabled);
            return le;
        }
        static QLabel* init_label(QLabel *la, QString txt, bool enabled = true){
            la->setText(txt);
            la->setEnabled(enabled);
            return la;
        }
        static QSpinBox* init(QSpinBox *sb, MinV<int> min, V<int> value, MaxV<int> max, StepV<int> singleStep, bool enabled = true){
            sb->setMinimum(min.v);
            sb->setMaximum(max.v);
            sb->setValue(value.v);
            sb->setSingleStep(singleStep.v);
            sb->setEnabled(enabled);
            return sb;
        }
        static QDoubleSpinBox* init(QDoubleSpinBox *dsb, MinV<qreal> min, V<qreal> value, MaxV<qreal> max, StepV<qreal> singleStep, int decimals, bool enabled = true);
        static inline QLabel *txt(QString txt){return new QLabel(txt);}

        static inline QLabel *icon(QIcon icon, QSize size){
            auto l = new QLabel();
            l->setPixmap(icon.pixmap(size));
            return l;
        }

        static inline QLabel *icon(QColor color, QSize size){
            auto l = new QLabel();
            auto p = QPixmap(size);
            p.fill(color);
            l->setPixmap(std::move(p));
            return l;
        }

        static QFrame *horizontal_line(){
            auto line = new QFrame();
            line->setObjectName(QString::fromUtf8("line"));
            line->setGeometry(QRect(320, 150, 118, 3));
            line->setFrameShape(QFrame::HLine);
            line->setFrameShadow(QFrame::Sunken);
            return line;
        }
    };

    struct L{
        static void stretch(QLayout *layout){
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
        static inline QVBoxLayout *VB(){return new QVBoxLayout();}
        static inline QHBoxLayout *HB(){return new QHBoxLayout();}
        static inline QGridLayout *G(){return new QGridLayout();}
    };

    struct F{

        static QFrame *gen_frame(QLayout *layout, std::vector<std::pair<QWidget*,int>> widgetsStretch, int stretchFactor = 0, LMarginsD margins = LMarginsD{},
                           QFrame::Shape shape = QFrame::Shape::NoFrame, QFrame::Shadow shadow = QFrame::Raised){


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

        static QFrame *gen_frame(QLayout *layout, std::vector<QWidget*> widgets, LStretchD strech = LStretchD{}, LMarginsD margins = LMarginsD{},
                           QFrame::Shape shape = QFrame::Shape::NoFrame, QFrame::Shadow shadow = QFrame::Raised){

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


        static QFrame* gen(QLayout *layout, std::vector<QWidget*> widgets, LStretch stretch = LStretch{true}, LMargins margins = LMargins{true},
                           QFrame::Shape shape = QFrame::Shape::NoFrame, QFrame::Shadow shadow = QFrame::Raised){
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
    };


}

