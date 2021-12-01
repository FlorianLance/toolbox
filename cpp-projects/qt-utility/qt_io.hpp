

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

// std
#include <fstream>
#include <iostream>

// Qt
#include <QFileDialog>

// base
#include "geometry/point3.hpp"
#include "utility/vector.hpp"


namespace tool::files{

template <typename acc>
static bool save_cloud(QString path, const std_v1<geo::Point3<acc>> &cloud){
    return save_cloud(path, cloud.data(), cloud.size());
}

template <typename acc>
static bool save_cloud(QString path, const std_v1<geo::Point3<acc>> &points, std_v1<geo::Point3<acc>> &colors){
    if(points.size() != colors.size()){
        std::cerr << "Invalid sizes for input cloud: " << points.size() << " " << colors.size() << "\n";
        return false;
    }
    return save_cloud(path, points.data(), colors.data(), points.size());
}

template <typename acc>
static bool save_cloud(QString path, const geo::Point3<acc> *cloud, const geo::Point3<float> *colors, size_t size){

    if(path.length() == 0){
        path = QFileDialog::getSaveFileName(nullptr, "Select cloud file", "", "OBJ file (*.obj)");
    }

    if(path.length() == 0){
        return false;
    }

    std::ofstream file;
    std::string name(path.toStdString());
    file.open(name);
    if(file.is_open() && size > 0){
        for(size_t ii = 0; ii < size; ++ii){
            const auto &v = cloud[ii];
            const auto &c = colors[ii];
            file << "v " << v.x() << " " << v.y() << " " << v.z() << " " << c.x() << " " << c.y() << " " << c.z() << "\n";
        }
    }
    file.close();
    return true;
}

template <typename acc>
static bool save_cloud(QString path, const geo::Point3<acc> *cloud, size_t size, geo::Pt3<float> color = {-1.f,-1.f,-1.f}){

    if(path.length() == 0){
        path = QFileDialog::getSaveFileName(nullptr, "Select cloud file", "", "OBJ file (*.obj)");
    }

    if(path.length() == 0){
        return false;
    }

    std::ofstream file;
    std::string name(path.toStdString());
    file.open(name);
    if(file.is_open() && size > 0){
        for(size_t ii = 0; ii < size; ++ii){
            const auto &v = cloud[ii];
            if(color.x() > -0.5f){
                file << "v " << v.x() << " " << v.y() << " " << v.z() << " " << color.x() << " " << color.y() << " " << color.z() << "\n";
            }else{
                file << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
            }
        }
    }
    file.close();
    return true;
}



}
