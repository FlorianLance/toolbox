

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

    using MinI  = MinV<int>;
    using MaxI  = MaxV<int>;
    using ValI  = V<int>;
    using StepI = StepV<int>;

    using MinR  = MinV<qreal>;
    using MaxR  = MaxV<qreal>;
    using ValR  = V<qreal>;
    using StepR = StepV<qreal>;

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
        MinR  min         = {-1000.};
        ValR  value       = {0.};
        MaxR  max         = {1000.};
        StepR singleStep  = {0.1};
        int   decimals    = 2;
    };

    struct Vector2dSettings{
        DsbSettings sX = {MinR{-1000.}, ValR{0}, MaxR{1000.}, StepR{0.01}, 2};
        DsbSettings sY = {MinR{-1000.}, ValR{0}, MaxR{1000.}, StepR{0.01}, 2};
    };

    struct Vector3dSettings{
        DsbSettings sX = {MinR{-1000.}, ValR{0}, MaxR{1000.}, StepR{0.01}, 2};
        DsbSettings sY = {MinR{-1000.}, ValR{0}, MaxR{1000.}, StepR{0.01}, 2};
        DsbSettings sZ = {MinR{-1000.}, ValR{0}, MaxR{1000.}, StepR{0.01}, 2};
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

        static QFrame* init(QFrame *f, bool on, bool enabled = true);
        static QComboBox* init(QComboBox *cb, QStringList items, int index=0, bool enabled = true);
        static QCheckBox* init(QCheckBox *cb, const QString &txt, bool checked, bool enabled = true);
        static QRadioButton* init(QRadioButton *rb, const QString &txt, bool checked, bool enabled = true);
        static QSlider* init(QSlider *sl, MinV<int> min, V<int> value, MaxV<int> max, StepV<int> singleStep, bool enabled = true);
        static QPushButton* init(QPushButton *pb, const QString &txt, bool enabled = true);
        static QTextEdit* init(QTextEdit *te, const QString &txt, Qt::TextFormat tf, bool enabled = true);
        static QLineEdit* init(QLineEdit *le, const QString &txt, bool enabled = true);
        static QLabel* init_label(QLabel *la, const QString &txt, bool enabled = true);
        static QSpinBox* init(QSpinBox *sb, MinV<int> min, V<int> value, MaxV<int> max, StepV<int> singleStep, bool enabled = true);
        static QDoubleSpinBox* init(QDoubleSpinBox *dsb, MinV<qreal> min, V<qreal> value, MaxV<qreal> max, StepV<qreal> singleStep, int decimals, bool enabled = true);

        static QLabel *txt(const QString &txt);
        static QLabel *txt(const QString &txt, Qt::Alignment alignment);
        static QLabel *icon(QIcon icon, QSize size);
        static QLabel *icon(QColor color, QSize size);
        static QFrame *horizontal_line();
    };

    struct L{
        static void stretch(QLayout *layout);
        static inline QVBoxLayout *VB(){return new QVBoxLayout();}
        static inline QHBoxLayout *HB(){return new QHBoxLayout();}
        static inline QGridLayout *G(){return new QGridLayout();}
    };

    struct F{

        static QFrame *gen_frame(QLayout *layout, std::vector<std::pair<QWidget*,int>> widgetsStretch, int stretchFactor = 0, LMarginsD margins = LMarginsD{},
                           QFrame::Shape shape = QFrame::Shape::NoFrame, QFrame::Shadow shadow = QFrame::Raised);

        static QFrame *gen_frame(QLayout *layout, std::vector<QWidget*> widgets, LStretchD strech = LStretchD{}, LMarginsD margins = LMarginsD{},
                           QFrame::Shape shape = QFrame::Shape::NoFrame, QFrame::Shadow shadow = QFrame::Raised);

        static QFrame* gen(QLayout *layout, std::vector<QWidget*> widgets, LStretch stretch = LStretch{true}, LMargins margins = LMargins{true},
                           QFrame::Shape shape = QFrame::Shape::NoFrame, QFrame::Shadow shadow = QFrame::Raised);
    };


}

