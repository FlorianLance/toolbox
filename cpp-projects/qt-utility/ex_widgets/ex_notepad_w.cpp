
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

#include "ex_notepad_w.hpp"
#include "qt_logger.hpp"

using namespace tool::ex;

ExNotepadW::ExNotepadW(QString name) : ExItemW<QFrame>(UiType::Notepad, name){

    // main layout
    auto mainL = new QVBoxLayout();
    w->setLayout(mainL);
    mainL->addWidget(ui::F::gen(ui::L::HB(), { ui::W::txt("Type:"), type()}, LStretch{false}, LMargins{false}, QFrame::Box));
    mainL->addWidget(text());
    mainL->setStretch(0, 1);
    mainL->setStretch(1, 20);

    connect(&text, &ExTextEditW::ui_change_signal, this, [=]{trigger_ui_change();});
    connect(&type, &ExComboBoxIndexW::ui_change_signal, this, [=]{
        if(type.w->currentIndex() == 0){
//            text.init_widget("PT", Qt::TextFormat::PlainText);
        }else if(type.w->currentIndex() == 1){
//            text.init_widget("RT", Qt::TextFormat::RichText);
        }else if(type.w->currentIndex() == 2){
//            text.init_widget("MT", Qt::TextFormat::MarkdownText);
        }
        trigger_ui_change();
    });
}

ExNotepadW *ExNotepadW::init_widget(QString txt, Qt::TextFormat tf, bool enabled){

    // init widets
    QStringList items;
    items << "Plain text" << "Rich text" << "Markdown text";
    if(tf == Qt::TextFormat::PlainText){
        type.init_widget(items, 0);
    }else if(tf == Qt::TextFormat::RichText){
        type.init_widget(items, 1);
    }else if(tf == Qt::TextFormat::MarkdownText){
        type.init_widget(items, 2);
    }
    text.init_widget(txt, Qt::TextFormat::PlainText, enabled);

    return this;
}

void ExNotepadW::update_from_arg(const Arg &arg){
    ExItemW::update_from_arg(arg);

    auto args = arg.split_value_to_atoms_args();
    if(args.size() != 2){
        QtLogger::error("ExNotepadW::Invalid arg.");
        return;
    }
    type.update_from_arg(args[0]);
    text.update_from_arg(args[1]);
}

Arg ExNotepadW::convert_to_arg() const{
    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from_args({type.convert_to_arg(), text.convert_to_arg()}, "%%%");
    return arg;
}

Qt::TextFormat ExNotepadW::selected_format() const{
    if(type.w->currentIndex() == 0){
        return Qt::TextFormat::PlainText;
    }else if(type.w->currentIndex() == 1){
        return Qt::TextFormat::RichText;
    }else {
        return Qt::TextFormat::MarkdownText;
    }
}

