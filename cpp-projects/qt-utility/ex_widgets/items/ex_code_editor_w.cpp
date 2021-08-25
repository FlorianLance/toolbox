
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

using namespace tool::ui;
using namespace tool::ex;


ExCodeEditorW::ExCodeEditorW() : ExItemW<CodeEditor>(UiType::Code_editor) {}

ExCodeEditorW *ExCodeEditorW::init_widget(QString txt, bool enabled){
    w->setPlainText(txt);
    w->setEnabled(enabled);
    return this;
}

void ExCodeEditorW::init_connection(const QString &nameParam){connect(w.get(), &CodeEditor::textChanged,this, [=]{emit ui_change_signal(nameParam);});}

void ExCodeEditorW::update_from_arg(const Arg &arg){

    ExItemW::update_from_arg(arg);

    w->blockSignals(true);

    if(arg.generator.name.length() > 0){
        // ...
    }
    w->setPlainText(arg.to_string_value());

    w->blockSignals(false);
}

Arg ExCodeEditorW::convert_to_arg() const{

    Arg arg = ExItemW::convert_to_arg();
    arg.init_from(w->toPlainText());

    // generator
    if(generatorName.length() > 0){
        // ...
    }
    return arg;
}
