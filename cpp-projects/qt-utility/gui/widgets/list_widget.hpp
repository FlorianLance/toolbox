
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
#include <QScrollArea>
#include <QVBoxLayout>
#include <QEvent>

namespace tool::ui {

//class ListWidget2 : public QScrollArea{


//public:

//    ListWidget2();

//private:

//    QGridLayout *m_layout = nullptr;
//    QWidget *m_content = nullptr;
//};

class ListWidget : public QScrollArea{

public:

    ListWidget();

    void set_margins(int left, int top, int right, int bottom, int inter);

    int count() const;

    QWidget *widget_at(int id);

    void move_from_to(int from, int to);

    void add_widget(QWidget *w);
    void insert_widget(int id, QWidget *w);

    QWidget *remove_at(int id);
    void remove_all();
    void delete_at(int id);
    void delete_all();

    void set_widget_selection(bool state);

    int current_selected_widget_id() const;

    void set_selected_widget(int id);

protected:

    bool eventFilter(QObject *obj, QEvent *ev) override;

private:

    void update_selection();

    QWidget *m_content = nullptr;
    QVBoxLayout *m_layout = nullptr;
    bool m_widgetSelection = false;
    int m_currentId = -1;

    const static inline QString m_selectedStyle = "QWidget[objectName=\"sub\"] {background-color:lightgray;}";
    const static inline QString m_unselectedStyle = "QWidget[objectName=\"sub\"] {background-color:white;}";
};
}
