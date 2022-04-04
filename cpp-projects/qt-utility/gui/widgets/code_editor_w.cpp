

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


#include "code_editor_w.hpp"

// Qt
#include <QPainter>
#include <QTextBlock>
#include <QStringListModel>
#include <QAbstractItemView>
#include <QScrollBar>

using namespace tool::ui;


CodeEditor::CodeEditor(QWidget *parent) : QPlainTextEdit(parent){

    // line number
    lineNumberArea = new LineNumberArea(this);    
    connect(this, &CodeEditor::blockCountChanged, this, &CodeEditor::updateLineNumberAreaWidth);
    connect(this, &CodeEditor::updateRequest, this, &CodeEditor::updateLineNumberArea);
    connect(this, &CodeEditor::cursorPositionChanged, this, &CodeEditor::highlightCurrentLine);

    // completer
//    c = new QCompleter(this);

//    words << "abc";
//    c->setModel(new QStringListModel(words, c));
//    c->setModelSorting(QCompleter::CaseInsensitivelySortedModel);
//    c->setCaseSensitivity(Qt::CaseInsensitive);
//    c->setWrapAround(false);
//    setCompleter(c);

    // highlighting
    highlightCurrentLine();
    // updateLineNumberAreaWidth(0);
    setViewportMargins(35, 0, 0, 0);
}


int CodeEditor::lineNumberAreaWidth(){

    int digits = 1;
    int max = qMax(1, blockCount());
    while (max >= 10) {
        max /= 10;
        ++digits;
    }

    int space = 3 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;
    return space;
}

//void CodeEditor::setCompleter(QCompleter *completer){
//    if (c){
//        c->disconnect(this);
//    }

//    c = completer;
//    if (!c){
//        return;
//    }

//    c->setWidget(this);
//    c->setCompletionMode(QCompleter::PopupCompletion);
//    c->setCaseSensitivity(Qt::CaseInsensitive);
//    QObject::connect(c, QOverload<const QString &>::of(&QCompleter::activated),this, &CodeEditor::insertCompletion);
//}


void CodeEditor::updateLineNumberAreaWidth(int /* newBlockCount */){
    //    setViewportMargins(lineNumberAreaWidth(), 0, 0, 0);
}

void CodeEditor::updateLineNumberArea(const QRect &rect, int dy){

    if (dy){
        lineNumberArea->scroll(0, dy);
    }else{
        lineNumberArea->update(0, rect.y(), lineNumberArea->width(), rect.height());
    }

    if (rect.contains(viewport()->rect())){
        updateLineNumberAreaWidth(0);
    }
}

void CodeEditor::resizeEvent(QResizeEvent *e){

    QPlainTextEdit::resizeEvent(e);
    QRect cr = contentsRect();
    lineNumberArea->setGeometry(QRect(cr.left(), cr.top(), lineNumberAreaWidth(), cr.height()));
}

void CodeEditor::highlightCurrentLine(){

    QList<QTextEdit::ExtraSelection> extraSelections;
    if (!isReadOnly()) {
        QTextEdit::ExtraSelection selection;

        QColor lineColor = highlightedLineColor;//.lighter(160);

        selection.format.setBackground(lineColor);
        selection.format.setProperty(QTextFormat::FullWidthSelection, true);
        selection.cursor = textCursor();
        selection.cursor.clearSelection();
        extraSelections.append(selection);
    }

    setExtraSelections(extraSelections);
}

void CodeEditor::lineNumberAreaPaintEvent(QPaintEvent *event){

    QPainter painter(lineNumberArea);
    painter.fillRect(event->rect(), Qt::lightGray);

    QTextBlock block = firstVisibleBlock();
    int blockNumber = block.blockNumber();
    int top = qRound(blockBoundingGeometry(block).translated(contentOffset()).top());
    int bottom = top + qRound(blockBoundingRect(block).height());

    while (block.isValid() && top <= event->rect().bottom()) {
        if (block.isVisible() && bottom >= event->rect().top()) {
            QString number = QString::number(blockNumber + 1 + offsetLineCounter);
            painter.setPen(Qt::black);
            painter.drawText(0, top, lineNumberArea->width(), fontMetrics().height(),Qt::AlignRight, number);
        }

        block = block.next();
        top = bottom;
        bottom = top + qRound(blockBoundingRect(block).height());
        ++blockNumber;
    }
}

//void CodeEditor::insertCompletion(const QString &completion){
//    if (c->widget() != this){
//        return;
//    }
//    QTextCursor tc = textCursor();
//    int extra = completion.length() - c->completionPrefix().length();
//    tc.movePosition(QTextCursor::Left);
//    tc.movePosition(QTextCursor::EndOfWord);
//    tc.insertText(completion.right(extra));
//    setTextCursor(tc);
//}


//QString CodeEditor::textUnderCursor() const{
//    QTextCursor tc = textCursor();
//    tc.select(QTextCursor::WordUnderCursor);
//    return tc.selectedText();
//}

//void CodeEditor::focusInEvent(QFocusEvent *e){
//    if (c){
//        c->setWidget(this);
//    }
//    CodeEditor::focusInEvent(e);
//}

//void CodeEditor::keyPressEvent(QKeyEvent *e){

//    if (c && c->popup()->isVisible()) {
//        // The following keys are forwarded by the completer to the widget
//        switch (e->key()) {
//        case Qt::Key_Enter:
//        case Qt::Key_Return:
//        case Qt::Key_Escape:
//        case Qt::Key_Tab:
//        case Qt::Key_Backtab:
//            e->ignore();
//            return; // let the completer do default behavior
//        default:
//            break;
//        }
//    }

//    const bool isShortcut = (e->modifiers().testFlag(Qt::ControlModifier) && e->key() == Qt::Key_E); // CTRL+E
//    if (!c || !isShortcut) {// do not process the shortcut when we have a completer
//        CodeEditor::keyPressEvent(e);
//    }

//    const bool ctrlOrShift = e->modifiers().testFlag(Qt::ControlModifier) ||e->modifiers().testFlag(Qt::ShiftModifier);
//    if (!c || (ctrlOrShift && e->text().isEmpty())){
//        return;
//    }

//    static QString eow("~!@#$%^&*()_+{}|:\"<>?,./;'[]\\-="); // end of word
//    const bool hasModifier = (e->modifiers() != Qt::NoModifier) && !ctrlOrShift;
//    QString completionPrefix = textUnderCursor();
//    if (!isShortcut && (hasModifier || e->text().isEmpty()|| completionPrefix.length() < 3 || eow.contains(e->text().right(1)))) {
//        c->popup()->hide();
//        return;
//    }

//    if (completionPrefix != c->completionPrefix()) {
//        c->setCompletionPrefix(completionPrefix);
//        c->popup()->setCurrentIndex(c->completionModel()->index(0, 0));
//    }
//    QRect cr = cursorRect();
//    cr.setWidth(c->popup()->sizeHintForColumn(0)+ c->popup()->verticalScrollBar()->sizeHint().width());
//    c->complete(cr); // popup it up!
//}
//#include "moc_code_editor_w.cpp"
