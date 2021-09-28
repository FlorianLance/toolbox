

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


#include "tree_view_w.hpp"


using namespace tool::ui;

TestViewDelegate::TestViewDelegate(QTreeView *parent) : QItemDelegate(parent) {
    QPixmap p(10,10);
    p.fill(Qt::red);
    icon = QIcon(p);
}

void TestViewDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const{

    if (index.column() == 0){//|| index.column() == 1) {
        QItemDelegate::paint(painter, option, index);
    }else if(index.column() == 1){

        //            tw->itemAt()

        // Set up a QStyleOptionProgressBar to precisely mimic the
        // environment of a progress bar.
        QStyleOptionProgressBar progressBarOption;
        progressBarOption.state = QStyle::State_Enabled;
        progressBarOption.direction   = QApplication::layoutDirection();
        progressBarOption.rect = option.rect;
        progressBarOption.fontMetrics = QApplication::fontMetrics();
        progressBarOption.minimum = 0;
        progressBarOption.maximum = 100;
        progressBarOption.textAlignment = Qt::AlignCenter;
        progressBarOption.textVisible = true;

        // Set the progress and text values of the style option.
        int progress = 50;// qobject_cast<MainWindow *>(parent())->clientForRow(index.row())->progress();
        progressBarOption.progress = progress < 0 ? 0 : progress;
        progressBarOption.text = QString::asprintf("%d%%", progressBarOption.progress);

        // Draw the progress bar onto the view.
        QApplication::style()->drawControl(QStyle::CE_ProgressBar, &progressBarOption, painter);
    }else if(index.column() == 2){

        auto tw = static_cast<TestTreeView*>(parent());
        auto item = tw->item_from_index(index);

        QStyleOptionButton buttonOption;//(QStyleOptionButton::None);
        buttonOption.state |= QStyle::State_Enabled;

        if(item->clickedState){
            buttonOption.text += "c";
        }
        if(item->hoveredState){
            buttonOption.text += "o";
            buttonOption.state |= QStyle::State_Sunken;
        }

        buttonOption.icon = icon;
        buttonOption.iconSize = QSize(25,25);
        buttonOption.rect = option.rect;
        buttonOption.fontMetrics = QApplication::fontMetrics();
        buttonOption.direction = QApplication::layoutDirection();



//        tw->item
//        tw->itemFromIndex(id)

        QApplication::style()->drawControl(QStyle::CE_PushButton, &buttonOption, painter);
    }

//    qDebug() << "has parent " << index.row() << (index.parent() != QModelIndex());

    //        if (index.column() != 2) {

    //            // Set up a QStyleOptionProgressBar to precisely mimic the
    //            // environment of a progress bar.
    //            QStyleOptionProgressBar progressBarOption;
    //            progressBarOption.state = QStyle::State_Enabled;
    //            progressBarOption.direction   = QApplication::layoutDirection();
    //            progressBarOption.rect = option.rect;
    //            progressBarOption.fontMetrics = QApplication::fontMetrics();
    //            progressBarOption.minimum = 0;
    //            progressBarOption.maximum = 100;
    //            progressBarOption.textAlignment = Qt::AlignCenter;
    //            progressBarOption.textVisible = true;

    //            // Set the progress and text values of the style option.
    //            int progress = 50;// qobject_cast<MainWindow *>(parent())->clientForRow(index.row())->progress();
    //            progressBarOption.progress = progress < 0 ? 0 : progress;
    //            progressBarOption.text = QString::asprintf("%d%%", progressBarOption.progress);

    //            // Draw the progress bar onto the view.
    //            QApplication::style()->drawControl(QStyle::CE_ProgressBar, &progressBarOption, painter);

    //            return;
    //        }else{

    //        }
}

TestWidgetItem::TestWidgetItem(int id, QStringList textes, TestWidgetItem *parent) : QTreeWidgetItem(parent, textes, QTreeWidgetItem::UserType){
    this->id = id;
}

TestTreeView::TestTreeView(QWidget *parent) : QTreeWidget(parent){
    setAcceptDrops(true);
    setDragEnabled(true);
    setMouseTracking(true);

    setColumnCount(3);
    setItemDelegate(new TestViewDelegate(this));

    // void activated(const QModelIndex &index)
    // void clicked(const QModelIndex &index)
    // void doubleClicked(const QModelIndex &index)
    // void entered(const QModelIndex &index)
    // void iconSizeChanged(const QSize &size)
    // void pressed(const QModelIndex &index)
    // void viewportEntered()

    // void collapsed(const QModelIndex &index)
    // void expanded(const QModelIndex &index)

//    void currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous)
//    void itemActivated(QTreeWidgetItem *item, int column)
//    void itemChanged(QTreeWidgetItem *item, int column)
//    void itemClicked(QTreeWidgetItem *item, int column)
//    void itemCollapsed(QTreeWidgetItem *item)
//    void itemDoubleClicked(QTreeWidgetItem *item, int column)
//    void itemEntered(QTreeWidgetItem *item, int column)
//    void itemExpanded(QTreeWidgetItem *item)
//    void itemPressed(QTreeWidgetItem *item, int column)
//    void itemSelectionChanged()
    connect(this, &TestTreeView::itemEntered, this, [&](QTreeWidgetItem *item, int column){
        for(auto &item : items){
            static_cast<TestWidgetItem*>(item)->hoveredState = false;
        }
        static_cast<TestWidgetItem*>(item)->hoveredState = true;
    });

    connect(this, &TestTreeView::itemPressed, this, [&](QTreeWidgetItem *item, int column){
        static_cast<TestWidgetItem*>(item)->clickedState = true;
    });
}

void TestTreeView::populate(){    
    for (int i = 0; i < 10; ++i){
        items.append(new TestWidgetItem(i, QStringList(QString("item: %1").arg(i)), static_cast<TestWidgetItem *>(nullptr)));
    }

    for (int i = 11; i < 20; ++i){
        items.append(new TestWidgetItem(i, QStringList(QString("item: %1").arg(i)), static_cast<TestWidgetItem *>(items[0])));
    }

    insertTopLevelItems(0, items);
}

void TestTreeView::mousePressEvent(QMouseEvent *event){
    QTreeWidget::mousePressEvent(event);
}

void TestTreeView::mouseReleaseEvent(QMouseEvent *event){

    QTreeWidget::mouseReleaseEvent(event);
    for(auto &item : items){
        static_cast<TestWidgetItem*>(item)->clickedState = false;
    }
    viewport()->update();
}

void TestTreeView::mouseMoveEvent(QMouseEvent *event){
//    if (!(event->buttons() & Qt::LeftButton)){
//        return;
//    }
    QTreeWidget::mouseMoveEvent(event);

    viewport()->update();
}

void TestTreeView::dragMoveEvent(QDragMoveEvent *event){
    setDropIndicatorShown(true);
    QTreeView::dragMoveEvent(event);

    viewport()->update();
}

void TestTreeView::dropEvent(QDropEvent *event){

    bool dropOK = false;
    DropIndicatorPosition dropIndicator = dropIndicatorPosition();


    auto tw = dynamic_cast<QTreeWidget*>(event->source());
    qDebug() << indexAt(event->pos()).row() << tw->currentIndex().row() << tw->currentIndex().column();

    QTreeWidgetItem *itemSource = tw->currentItem();
    QTreeWidgetItem *itemDest = tw->itemAt(event->pos());

    if(itemDest->parent() != nullptr){
//        qDebug() << "dest parent " << itemDest->parent()->text(0);
    }

    qDebug() << itemSource->text(0) << itemDest->text(0);
    //        QTreeWidgetItem*

    switch (dropIndicator){
    case QAbstractItemView::AboveItem:
        dropOK = true;
        break;
    case QAbstractItemView::BelowItem:
        dropOK = true;
        break;
    case QAbstractItemView::OnItem:
        dropOK = false;
        break;
    case QAbstractItemView::OnViewport:
        dropOK = false;
        break;
    }


    if(dropOK){
//        qDebug() << "ok!";
        // Here, you need to manage yourself the case of dropping an item
    }
    setDropIndicatorShown(false); // hide the drop indicator once the drop is done

    viewport()->update();
}
