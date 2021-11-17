
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

// std
#include <string>

// Qt
#include <QRegularExpression>
#include <QTextCharFormat>

// local
#include "qt_str.hpp"

namespace tool {

struct HighlightingRule{
    QRegularExpression pattern;
    QTextCharFormat format;
};

struct CSharpHighlighting{

    // rules
    static inline QVector<HighlightingRule> csharpHighlingRules;

    static void initialize();
    static void add_structures(const QStringList &structures);
    static void add_classes(const QStringList &classes);


    // keywords
    static inline const QString keywordPatterns[] = {
        QSL("\\bdo\\b"), QSL("\\bsizeof\\b"), QSL("\\bthis\\b"),
        QSL("\\bref\\b"), QSL("\\binternal\\b"), QSL("\\bout\\b"),
        QSL("\\bvolatile\\b"), QSL("\\breturn\\b"),QSL("\\babstract\\b"),
        QSL("\\bas\\b"), QSL("\\bbase\\b"),QSL("\\btry\\b"),
        QSL("\\btry\\b"), QSL("\\bcatch\\b"), QSL("\\bdelegate\\b"),
        QSL("\\bchecked\\b"),QSL("\\bexplicit\\b"),
        QSL("\\bforeach\\b"), QSL("\\bgoto\\b"),
        QSL("\\bis\\b"),QSL("\\block\\b"), QSL("\\bevent\\b"),
        QSL("\\bdefault\\b"), QSL("\\bextern\\b"),
        QSL("\\bstackalloc\\b"), QSL("\\bin\\b"), QSL("\\bimplicit\\b"),
        QSL("\\bfinally\\b"),QSL("\\bfixed\\b"),
        QSL("\\bnull\\b"), QSL("\\boperator\\b"),
        QSL("\\bparams\\b"),QSL("\\breadonly\\b"),
        QSL("\\bnew\\b"), QSL("\\bcontinue\\b"),
        QSL("\\bsealed\\b"),QSL("\\bthrow\\b"), QSL("\\bunsafe\\b"),
        QSL("\\bunchecked\\b"),QSL("\\btypeof\\b"),
        QSL("\\bswitch\\b"), QSL("\\bcase\\b"),QSL("\\bbreak\\b"),
        QSL("\\bwhile\\b"), QSL("\\bfor\\b"),
        QSL("\\bif\\b"), QSL("\\belse\\b"),
        QSL("\\btrue\\b"), QSL("\\bfalse\\b"),
        QSL("\\bclass\\b"), QSL("\\bconst\\b"),
        QSL("\\bEnum\\b"), QSL("\\busing\\b"),
        QSL("\\bprivate\\b"), QSL("\\bprotected\\b"), QSL("\\bpublic\\b"),
        QSL("\\bstatic\\b"), QSL("\\bstruct\\b"),
        QSL("\\bvirtual\\b"), QSL("\\boverride\\b"), QSL("\\bnamespace\\b")
    };

    // types
    static inline const QString typesPatterns[] = {
        QSL("\\bvoid\\b"), QSL("\\bvar\\b"),
        // built-in values types
        QSL("\\bbool\\b"),
        QSL("\\bbyte\\b"), QSL("\\bsbyte\\b"),
        QSL("\\bchar\\b"),
        QSL("\\bdecimal\\b"),
        QSL("\\bdouble\\b"), QSL("\\bfloat\\b"),
        QSL("\\buint\\b"), QSL("\\bint\\b"),
        QSL("\\bulong\\b"), QSL("\\blong\\b"),
        QSL("\\bushort\\b"), QSL("\\bshort\\b"),
        // built-in references types
        QSL("\\bstring\\b"), QSL("\\bobject\\b"), QSL("\\bdynamic\\b"),
    };

    // structs instances
    static inline const QString structsInstancesPatterns[] = {
        QSL("\\Color\\b"),
        QSL("\\bVector2\\b"), QSL("\\bVector3\\b"), QSL("\\bQuaternion\\b"),
    };

    // unity classes instances
    static inline const QString unityClassesInstancesPatterns[] = {
        QSL("\\bType\\b"),
        QSL("\\bArrayList\\b"), QSL("\\bHashtable\\b"),
        QSL("\\bList\\b"), QSL("\\bSortedList\\b"),
        QSL("\\bQueue\\b"), QSL("\\bStack\\b"),
        QSL("\\bDictionary\\b"), QSL("\\bTuple\\b"),
        QSL("\\bGameObject\\b"),QSL("\\bTransform\\b"),
        QSL("\\bMesh\\b"),QSL("\\bAudioClip\\b"),
        QSL("\\bAudioSource\\b"),QSL("\\bTexture2D\\b"),
        QSL("\\bMonoBehaviour\\b"),QSL("\\bObject\\b"),
        QSL("\\bMathf\\b"),QSL("\\bRandom\\b"),
        QSL("\\bTime\\b"),QSL("\\bDebug\\b"),
    };

    // formats
    static inline QTextCharFormat structInstancesFormat;
    static inline QTextCharFormat classesInstancesFormat;
    static inline QTextCharFormat digitFormat;
    static inline QTextCharFormat keywordFormat;
    static inline QTextCharFormat typeFormat;
    static inline QTextCharFormat classFormat;
    static inline QTextCharFormat singleLineCommentFormat;
    static inline QTextCharFormat multiLineCommentFormat;
    static inline QTextCharFormat quotationFormat;
    static inline QTextCharFormat functionFormat;
    static inline QTextCharFormat defaultFormat;

    // regex
    static inline QRegularExpression commentStartExpression = QRegularExpression(QSL("/\\*"));
    static inline QRegularExpression commentEndExpression = QRegularExpression(QSL("\\*/"));

};

}
