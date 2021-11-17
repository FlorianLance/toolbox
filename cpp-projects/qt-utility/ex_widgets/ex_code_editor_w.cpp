
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

#include "ex_code_editor_w.hpp"

// qt-utility
#include "widgets/text_widget_highlighter.hpp"


using namespace tool::ui;
using namespace tool::ex;

ExCodeEditorW::ExCodeEditorW(QString name) : ExItemW<CodeEditor>(UiType::Code_editor, name) {}

ExCodeEditorW *ExCodeEditorW::init_widget(QString txt, bool enabled){
    w->setPlainText(txt);
    w->setEnabled(enabled);
    return this;
}

ExCodeEditorW *ExCodeEditorW::init_widget_as_csharp_editor(const QColor &bc, QString txt, bool enabled){

    QFont font;
    font.setFamily("Courier");
    font.setStyleHint(QFont::Monospace);
    font.setFixedPitch(true);
    font.setPointSize(10);
    w->setFont(font);

    // tab size
    QFontMetrics metrics(font);
    auto distance = metrics.horizontalAdvance("    ");
    w->setTabStopDistance(distance);

    ui::CSharpHighlighter *cshStartFunction = new ui::CSharpHighlighter(w->document(), &CSharpHighlighting::csharpHighlingRules);
    static_cast<void>(cshStartFunction);
    w->setStyleSheet(QString("background-color: rgb(%1,%2,%3); border: 0px ;").arg(bc.red()).arg(bc.green()).arg(bc.blue()));
    w->zoomIn(2);

    w->setPlainText(txt);
    w->setEnabled(enabled);


    connect(w.get(), &CodeEditor::textChanged,this, [=]{trigger_ui_change();});

    return this;
}


void ExCodeEditorW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(arg.generator.has_value()){
//        if(const auto &info = arg.generator->info; info.has_value()){

//        }else{
//            qDebug() << "ExCodeEditorW Invalid genrator.";
//        }
    }else{

        w->setPlainText(arg.to_string_value());
    }

    w->blockSignals(false);
}

Arg ExCodeEditorW::convert_to_arg() const{

    Arg arg = ExBaseW::convert_to_arg();
    arg.init_from(w->toPlainText());

    // generator
    if(hasGenerator){
        // ...
    }
    return arg;
}
