

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
#include <QTreeWidget>
#include <QItemDelegate>
#include <QDragMoveEvent>
#include <QUrl>
#include <QMimeData>
#include <QApplication>
#include <QDrag>

#include <QDebug>



//class TestItemModel : public QAbstractItemModel{
//public:

//};

namespace tool::ui{


class TestWidgetItem : public QTreeWidgetItem{
public:
    TestWidgetItem(int id, QStringList textes, TestWidgetItem *parent);
    int id;
    bool clickedState = false;
    bool hoveredState = false;
};

class TestViewDelegate : public QItemDelegate{
    // Q_OBJECT
public:
    QIcon icon;
    inline TestViewDelegate(QTreeView *parent);
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index ) const override;
};

class TestTreeView : public QTreeWidget{
    Q_OBJECT
public:
    TestTreeView(QWidget *parent);


    TestWidgetItem *item_from_index(const QModelIndex &index){
        return static_cast<TestWidgetItem*>(itemFromIndex(index));
    }


    void populate();
    QList<QTreeWidgetItem *> items;

signals:

//    void fileDropped(const QString &fileName);

protected:


    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void dragMoveEvent(QDragMoveEvent *event);
    void dropEvent(QDropEvent* event);
};
}

