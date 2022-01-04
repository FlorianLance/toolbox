
/*******************************************************************************
** Toolbox-base                                                               **
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
#include <fstream>
#include <ostream>
#include <string_view>
#include <vector>

// local
#include "geometry/point2.hpp"
#include "geometry/point3.hpp"
#include "geometry/point4.hpp"

namespace tool::files {

using namespace std::string_view_literals;
class CloudIO{

public:

    template<typename TV>
    static bool save_cloud(const std::string &path, geo::Pt2<TV> *vertices, size_t size){
        return save_cloud<TV,2>(path, reinterpret_cast<TV*>(vertices), size);
    }
    template<typename TV>
    static bool save_cloud(const std::string &path, geo::Pt3<TV> *vertices, size_t size){
        return save_cloud<TV,3>(path, reinterpret_cast<TV*>(vertices), size);
    }
    template<typename TV,typename TC>
    static bool save_cloud(const std::string &path, geo::Pt2<TV> *vertices, geo::Pt3<TC> *colors, size_t size){
        return save_cloud<TC,TV,2,3>(path, reinterpret_cast<TV*>(vertices), reinterpret_cast<TC*>(colors), size);
    }
    template<typename TV,typename TC>
    static bool save_cloud(const std::string &path, geo::Pt2<TV> *vertices, geo::Pt4<TC> *colors, size_t size){
        return save_cloud<TC,TV,2,4>(path, reinterpret_cast<TV*>(vertices), reinterpret_cast<TC*>(colors), size);
    }
    template<typename TV,typename TC>
    static bool save_cloud(const std::string &path, geo::Pt3<TV> *vertices, geo::Pt3<TC> *colors, size_t size){
        return save_cloud<TC,TV,3,3>(path, reinterpret_cast<TV*>(vertices), reinterpret_cast<TC*>(colors), size);
    }
    template<typename TV,typename TC>
    static bool save_cloud(const std::string &path, geo::Pt3<TV> *vertices, geo::Pt4<TC> *colors, size_t size){
        return save_cloud<TC,TV,3,4>(path, reinterpret_cast<TV*>(vertices), reinterpret_cast<TC*>(colors), size);
    }

    template <typename TV,typename TC, int dimension, int nbChannels>
    static bool save_cloud(const std::string &path, const std::vector<TV> &vertices, const std::vector<TC> &colors){

        if(check_bufers_sizes(vertices.size(), colors.size())){
            if(colors.size() == 0){
                return save_cloud<TV, dimension>(path, vertices.data(), vertices.size(), dimension);
            }else{
                return save_cloud<TV, TC, dimension, nbChannels>(path, vertices.data(), colors.data(), vertices.size(), dimension, nbChannels);
            }
        }
        return false;
    }

    template <typename TV, int dimension>
    static bool save_cloud(const std::string &path, TV *vertices, size_t size){

        if(!check_input_values(size, vertices != nullptr, false, dimension, 0)){
            return false;
        }

        std::ofstream file;
        if(!open_file(path, file)){
            return false;
        }

        for(size_t ii = 0; ii < size; ++ii){
            write_line<TV,dimension>(file, &vertices[ii*dimension]);
        }

        file.close();
        return true;
    }


    template <typename TV,typename TC, int dimension, int nbChannels>
    static bool save_cloud(const std::string &path, TV *vertices, TC *colors, size_t size){


        if(!check_input_values(size, vertices != nullptr, colors != nullptr, dimension, nbChannels)){
            return false;
        }

        std::ofstream file;
        if(!open_file(path, file)){
            return false;
        }

        if(colors != nullptr && nbChannels != 0){
            for(size_t ii = 0; ii < size; ++ii){
                write_line<TV,TC,dimension,nbChannels>(file, &vertices[ii*dimension], &colors[ii*nbChannels]);
            }
        }else{
            for(size_t ii = 0; ii < size; ++ii){
                write_line<TV,dimension>(file, &vertices[ii*dimension]);
            }
        }

        file.close();
        return true;
    }

private:


    template <typename TV, int dimension>
    static void write_line(std::ostream &file, TV *v){
        if constexpr (dimension == 2){
            write(file, v[0], v[1]);
        } else if constexpr (dimension == 3){
            write(file, v[0], v[1], v[2]);
        }
    }

    template <typename TV,typename TC, int dimension, int nbChannels>
    static void write_line(std::ostream &file, TV *v, TC *c){
        if constexpr (dimension == 2 && nbChannels == 3){
            write(file, v[0], v[1], c[0], c[1], c[2]);
        }else if constexpr (dimension == 2 && nbChannels == 4){
            write(file, v[0], v[1], c[0], c[1], c[2], c[3]);
        }else if constexpr (dimension == 3 && nbChannels == 3){
            write(file, v[0], v[1], v[2], c[0], c[1], c[2]);
        }else if constexpr (dimension == 3 && nbChannels == 4){
            write(file, v[0], v[1], v[2], c[0], c[1], c[2], c[3]);
        }
    }

    static void write(std::ostream &file, float v1, float v2);
    static void write(std::ostream &file, float v1, float v2, float v3);
    static void write(std::ostream &file, float v1, float v2, float v3, float v4, float v5);
    static void write(std::ostream &file, float v1, float v2, float v3, float v4, float v5, float v6);
    static void write(std::ostream &file, float v1, float v2, float v3, float v4, float v5, float v6, float v7);

    static bool open_file(const std::string &path, std::ofstream &file);
    static bool check_bufers_sizes(size_t sizeVertices, size_t sizeColors);
    static bool check_input_values(size_t size, bool hasVertices, bool hasColors, int dimension, int nbChannels);

    static constexpr std::string_view line2 = "v {} {}\n"sv;
    static constexpr std::string_view line3 = "v {} {} {}\n"sv;
    static constexpr std::string_view line5 = "v {} {} {} {} {}\n"sv;
    static constexpr std::string_view line6 = "v {} {} {} {} {} {}\n"sv;
    static constexpr std::string_view line7 = "v {} {} {} {} {} {} {}•\n"sv;
};

}
