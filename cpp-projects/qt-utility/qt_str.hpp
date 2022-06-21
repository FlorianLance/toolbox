
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
#include <QStringBuilder>
#include <QStringView>

#define QSL QStringLiteral

namespace tool {    

    [[maybe_unused]] static QString from_view(std::string_view str){
        return QString::fromStdString(std::string(str));
    }

    [[maybe_unused]] static QStringView sub_view_started_by(QStringView view, QChar start, QChar end){
        auto idStart = view.indexOf(start) +1;
        if(idStart == -1){
            return view;
        }
        auto idEnd = view.indexOf(end, idStart);
        if(idEnd == -1){
            return view;
        }
        return view.mid(idStart, idEnd-idStart);
    }

    [[maybe_unused]] static QString clamp(QString n, int maxLength){
        if(n.length() > maxLength){
            return n.left(maxLength) % QSL("...");
        }
        return n;
    }

}


