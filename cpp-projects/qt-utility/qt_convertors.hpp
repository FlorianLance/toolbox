

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
#include <QStringList>
#include <QColor>
#include <QPolygonF>

// base
#include "utility/keyboard_utility.hpp"
#include "utility/mouse_utility.hpp"
#include "utility/types_utility.hpp"
#include "geometry/point2.hpp"
#include "geometry/point3.hpp"

// local
#include "qt_str.hpp"
#include "qt_types.hpp"


namespace tool::str {

struct Convertor{

    static bool to_bool(const QString &value){
        return value == QSL("True") || value == QSL("true") || value == QSL("1");
    }
    static int to_int(const QString &value){
        return value.toInt();
    }
    static float to_float(const QString &value){
        return value.toFloat();
    }
    static double to_double(const QString &value){
        return value.toDouble();
    }            
    static constexpr double to_double(int value){
        return static_cast<double>(value);
    }
    static constexpr double to_double(float value){
        return static_cast<double>(value);
    }

    static std_v1<double> to_double_list(double value){
        return {value};
    }

    static std_v1<double> to_double_list(const QString &value){

        if(value.count(',') == 0){
            if(value.size() > 0){
                return {to_double(value)};
            }else{
                return {};
            }
        }

        auto split = value.split(',');
        std_v1<double> values;
        values.reserve(static_cast<size_t>(split.size()));
        for(const auto &v : split){
            values.emplace_back(to_double(v));
        }        
        return values;
    }

    static QColor to_color(const QString &value, QString separator = " "){
        QStringList split = value.split(separator);
        return QColor(static_cast<int>(split[1].toFloat()*255),
                      static_cast<int>(split[2].toFloat()*255),
                      static_cast<int>(split[3].toFloat()*255),
                      static_cast<int>(split[0].toFloat()*255));
    }

    static std::pair<std_v1<double>,std_v1<double>> to_curve(const QString &value, QString separator = " "){

        std_v1<double> px,py;
        QStringList split = value.split(separator);
        px.reserve(static_cast<size_t>(split.size()));
        py.reserve(static_cast<size_t>(split.size()));
        for(int ii = 0; ii < split.size()/2; ++ii){
            px.emplace_back(to_double(split[ii*2]));
            py.emplace_back(to_double(split[ii*2+1]));
        }
        return {px,py};
    }

    static std::vector<std::pair<std_v1<double>,std_v1<double>>> to_curves(const QString &value, QString sep1 = "*", QString sep2 = " "){

        std::vector<std::pair<std_v1<double>,std_v1<double>>> curves;
        QStringList split = value.split(sep1);

        curves.reserve(split.size());
        for(int ii = 0; ii < split.size(); ++ii){
            curves.emplace_back(to_curve(split[ii], sep2));
        }

        return curves;
    }

    static QString to_str(const geo::Pt2<float> &value){
        return  QString::number(static_cast<double>(value.x())) % QSL(", ") %
                QString::number(static_cast<double>(value.y()));
    }


    static QString to_str(const geo::Pt2<double> &value){
        return  QString::number(value.x()) % QSL(", ") %
                QString::number(value.y());
    }

    static QString to_str(const geo::Pt3<float> &value){
        return  QString::number(static_cast<double>(value.x())) % QSL(", ") %
                QString::number(static_cast<double>(value.y())) % QSL(", ") %
                QString::number(static_cast<double>(value.z()));
    }

    static QString to_str(const geo::Pt3<double> &value){
        return  QString::number(value.x()) % QSL(", ") %
                QString::number(value.y()) % QSL(", ") %
                QString::number(value.z());
    }

    static QString to_str(const Transform &value){
        return to_str(value.position) % QSL("\n") % to_str(value.rotation) %  QSL("\n") % to_str(value.scale);
    }

    static QString to_str(input::Keyboard::Button value){
        return from_view(input::Keyboard::get_name(value));
    }

    static QString to_str(input::Mouse::Button value){
        return from_view(input::Mouse::get_name(value));
    }

    static QString to_str(bool value){
        return value ? QSL("true") : QSL("false");
    }

    static QString to_str(int value){
        return QString::number(value);
    }

    static QString to_str(float value){
        return QString::number(static_cast<double>(value));
    }

    static QString to_str(double value){
        return QString::number(value);
    }

    static QString to_str(QColor value){
        return QString::number(value.alphaF()) % QSL(" ") %
               QString::number(value.redF())   % QSL(" ") %
               QString::number(value.greenF()) % QSL(" ") %
               QString::number(value.blueF());
    }

    static QString to_str(Decimal value){
        if(value.is_int()){
            return to_str(value.to_int());
        }else if(value.is_float()){
            return to_str(value.to_float());
        }
        return to_str(value.to_double());
    }

    static QString to_str(const DecimalList &list){
        return QSL("vector size(") % QString::number(list.size()) % QSL(")");
    }

    static QString to_str(const RealList &list){
        return QSL("vector size(") % QString::number(list.size()) % QSL(")");
    }

    static QString to_str(const QString &value){
        return value;
    }

    static QString to_str(const QStringList &list, const QString &separator){
        return list.join(separator);
    }


    static QString to_str(VoidV value){
        Q_UNUSED(value)
        return QSL("void");
    }

    static QString to_str(UndefinedV value){
        Q_UNUSED(value)
        return QSL("undefined");
    }

//    static QString to_str(const std::vector<double> &values){
//        QStringList valuesStr;
//        valuesStr.reserve(static_cast<int>(values.size()));
//        for(const auto & v : values){
//            valuesStr << to_str(v);
//        }
//        return to_str(valuesStr, ",");
//    }

    static QString curve_to_str(std::vector<double> *xCoords, std::vector<double> *yCoords, const QString &separator = " "){

        QString value{""};

        for(size_t ii = 0; ii < xCoords->size(); ++ii){
            value += to_str((*xCoords)[ii]) % separator % to_str((*yCoords)[ii]);
            if(ii < xCoords->size()-1){
                value += separator;
            }
        }

        return value;
    }
};
}
