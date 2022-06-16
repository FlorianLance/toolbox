
#include "list_widget.hpp"

// local
#include "qt_str.hpp"

// Qt
#include <QMouseEvent>

using namespace tool::ui;


ListWidget::ListWidget(QColor contentColor){

    setWidgetResizable(true);

    m_layout = new QVBoxLayout();
    m_content = new QWidget(this);
    m_content->installEventFilter(this);
    m_content->setLayout(m_layout);
    m_content->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    m_content->setObjectName("content");
//    m_content->setStyleSheet("QWidget[objectName=\"content\"] {background-color:white;}");

    m_content->setStyleSheet(QSL("QWidget[objectName=\"content\"] {background-color:") % contentColor.name() % QSL(";}"));
    setWidget(m_content);

    m_layout->setContentsMargins(0,0,0,0);
    m_layout->addStretch(1);
}

void ListWidget::set_margins(int left, int top, int right, int bottom, int inter){
    m_layout->setContentsMargins(left,top,right,bottom);
    m_layout->setSpacing(inter);
}

int ListWidget::count() const{
    return m_layout->count()-1;
}

QWidget *ListWidget::widget_at(int id){
    return m_layout->itemAt(id)->widget();
}

void ListWidget::move_from_to(int from, int to){

    if(from >= count() || to >= count()){
        return;
    }

    m_layout->insertWidget(to, m_layout->takeAt(from)->widget());
}

void ListWidget::add_widget(QWidget *w){
    w->setObjectName("sub");
    m_layout->insertWidget(count(), w);
    m_currentId = count()-1;
    update_selection();
}

void ListWidget::insert_widget(int id, QWidget *w){
    w->setObjectName("sub");
    m_layout->insertWidget(id, w);
    m_currentId = id;
    update_selection();
}

QWidget *ListWidget::remove_at(int id){

    if(id < m_currentId){
        --m_currentId;
    }else if(id == m_currentId){
        if(count() == 1){
            m_currentId = -1;
        }else{
            if(m_currentId > 0){
                --m_currentId;
            }
        }
    }

    auto w = m_layout->takeAt(id)->widget();
    update_selection();
    return w;
}

void ListWidget::remove_all(){

    for(int ii = count()-1; ii >= 0; --ii){
        m_layout->takeAt(ii)->widget();
    }

    m_currentId = -1;
    update_selection();
}


void ListWidget::delete_at(int id){

    if(id < m_currentId){
        --m_currentId;
    }else if(id == m_currentId){
        if(count() == 1){
            m_currentId = -1;
        }else{
            if(m_currentId > 0){
                --m_currentId;
            }
        }
    }

    auto w = m_layout->takeAt(id)->widget();
    update_selection();
    delete w;
}

void ListWidget::delete_all(){

    for(int ii = count()-1; ii >= 0; --ii){
        delete m_layout->takeAt(ii)->widget();
    }

    m_currentId = -1;
    update_selection();
}

void ListWidget::set_widget_selection(bool state){
    m_widgetSelection = state;
    update_selection();
}

int ListWidget::current_selected_widget_id() const{
    return m_currentId;
}

void ListWidget::set_selected_widget(int id){
    m_currentId = id;
    update_selection();
}

bool ListWidget::eventFilter(QObject *obj, QEvent *event){

    if(event->type() == QEvent::MouseButtonPress && m_widgetSelection){

        m_currentId = -1;
        for(int ii = 0; ii < count(); ++ii){
            auto localMousePos = widget_at(ii)->mapFromGlobal(QCursor::pos());
            if (widget_at(ii)->rect().contains(localMousePos)) {
                m_currentId = ii;
                break;
            }
        }

        update_selection();
    }

    return QScrollArea::eventFilter(obj, event);
}

void ListWidget::update_selection(){

    for(int ii = 0; ii < count(); ++ii){
        if(m_widgetSelection){
            if(m_currentId == ii){
                widget_at(ii)->setStyleSheet(m_selectedStyle);
            }else{
                widget_at(ii)->setStyleSheet(m_unselectedStyle);
            }
        }else{
            widget_at(ii)->setStyleSheet(m_unselectedStyle);
        }
    }
}

