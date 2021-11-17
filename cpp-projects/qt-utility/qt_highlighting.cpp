

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

#include "qt_highlighting.hpp"




void tool::CSharpHighlighting::initialize(){

    // init formats
    structInstancesFormat.setForeground(QColor(134,176,95));
    structInstancesFormat.setFontWeight(QFont::Bold);
    classesInstancesFormat.setForeground(QColor(63, 201, 176));
    classesInstancesFormat.setFontWeight(QFont::Bold);
    keywordFormat.setForeground(QColor(82,156,214));
    keywordFormat.setFontWeight(QFont::Bold);
    digitFormat.setForeground(QColor(167,206,168));
    typeFormat.setForeground(QColor(82,156,214));
    typeFormat.setFontWeight(QFont::Bold);
    classFormat.setFontWeight(QFont::Bold);
    classFormat.setForeground(QColor(63, 201, 176));
    quotationFormat.setForeground(QColor(214, 157, 133));
    functionFormat.setFontItalic(true);
    functionFormat.setForeground(QColor(220,220,200));
    singleLineCommentFormat.setForeground(QColor(87, 166, 70));
    multiLineCommentFormat.setForeground(QColor(87, 166, 70));
    defaultFormat.setForeground(QColor(220,220,200));

    // add rules
    // # default
    csharpHighlingRules.append(HighlightingRule{QRegularExpression(QSL("\\b[A-Za-z]+\\b")), defaultFormat});
    // # digits
    csharpHighlingRules.append(HighlightingRule{QRegularExpression(QSL("\\b[0-9]*[,.]?[0-9]*[f]?\\b")), digitFormat});
    // # function
    csharpHighlingRules.append(HighlightingRule{QRegularExpression(QSL("\\b[A-Za-z0-9_]+(?=\\()")), functionFormat});
    // # quotation
    csharpHighlingRules.append(HighlightingRule{QRegularExpression(QSL("\".*\"")), quotationFormat});
    // # single line comment
    csharpHighlingRules.append(HighlightingRule{QRegularExpression(QSL("//[^\n]*")), singleLineCommentFormat});

    // # keywords
    for (const QString &pattern : keywordPatterns) {
        csharpHighlingRules.append(HighlightingRule{QRegularExpression(pattern),keywordFormat});
    }
    // # types
    for (const QString &pattern : typesPatterns) {
        csharpHighlingRules.append(HighlightingRule{QRegularExpression(pattern),typeFormat});
    }
    // # structs instances
    for (const QString &pattern : structsInstancesPatterns) {
        csharpHighlingRules.append(HighlightingRule{QRegularExpression(pattern),structInstancesFormat});
    }
    // # classes
    // ## unity
    for (const QString &pattern : unityClassesInstancesPatterns) {
        csharpHighlingRules.append(HighlightingRule{QRegularExpression(pattern),classesInstancesFormat});
    }
}

void tool::CSharpHighlighting::add_structures(const QStringList &structures){
    for(const auto &structureName : structures){
        csharpHighlingRules.append(HighlightingRule{QRegularExpression(structureName), structInstancesFormat});
    }
}

void tool::CSharpHighlighting::add_classes(const QStringList &classes){
    for(const auto &className : classes){
        csharpHighlingRules.append(HighlightingRule{QRegularExpression(className), classesInstancesFormat});
    }
}
