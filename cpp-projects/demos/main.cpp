/*******************************************************************************
** Toolbox-3d-engine                                                          **
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
#include <iostream>
#include <filesystem>

// local
#include "tests/samples/draw_samples_window.hpp"
#include "tests/biopac/biopac_control_window.hpp"

#include "geometry/point.hpp"

struct Point2t : public tool::geo::Point<float,2>{

    using Point<float,2>::v;

    Point2t() = default;
    Point2t(const Point2t& other) = default;
    Point2t& operator=(const Point2t& other) = default;
    Point2t(Point2t&& other) = default;
    Point2t& operator=(Point2t&& other) = default;

    explicit Point2t(const tool::geo::Point<float,2>& other){
        std::cout << "constexpr explicit Point2t(const tool::geo::Point<float,2>& other){ \n";
        v = other.v;
    }
      explicit Point2t(tool::geo::Point<float,2>&& other){
        std::cout << "constexpr explicit Point2t(tool::geo::Point<float,2>&& other){ \n";
        v = std::move(other.v);
    }

      inline Point2t(std::initializer_list<float> l) noexcept{
          std::cout << "inline Point2t(std::initializer_list<float> l) noexcept{ \n";
          std::move(l.begin(), l.end(), std::begin(v));
      }

      explicit Point2t(float x, float y) noexcept {
        std::cout << "constexpr explicit Point2t(float x, float y) noexcept { \n";
        v = {x,y};
    }

//    inline explicit Point2t(const tool::vecN<float,2> &array) noexcept {
//        std::cout << "inline Point2t(const tool::vecN<float,2> &array) noexcept{ \n";
//        v = array;
//    }

//    inline explicit Point2t(tool::vecN<float,2> &&array) noexcept{
//        std::cout << "inline Point2t(tool::vecN<float,2> &&array) noexcept{ \n";
//        v = std::move(array);
//    }







    //    constexpr Point(const vecN<acc,dim> &array) noexcept{
    //        v = array;
    //    }

    //    constexpr Point(vecN<acc,dim> &&array) noexcept{
    //        v = std::move(array);
    //    }


//    inline Point2t(const std::array<float,2> &l) noexcept{
//        std::cout << "inline Point2t(std::array<float,2> l) noexcept{ \n";
//        std::copy(l.begin(), l.end(), std::begin(v));
//    }

//    Point2t& operator=(const Point2t& other) = default;
};

int main(int argc, char *argv[]){

//    tool::geo::Pt3f p;
//    tool::geo::Pt3<std::uint8_t> p1(155,200,150);
//    tool::geo::Pt4<std::uint8_t> p2(155,100,150,200);
//    std::cout << p1.conv<int>() << " | " << p2.conv<int>() << "\n";
//    auto p3 = p1.conv<int>()-p2.xyz().conv<int>();
//    std::cout << p3 << "\n";
//    auto delta = norm(p3);
//    std::cout << delta << "\n";

//    return 0;

//    Point2t p0;
//    Point2t p1 = {1.f,2.f};
//    Point2t p2{1.f,2.f};
//    Point2t p3(1.f,2.f);
//    Point2t p4({1.f, 2.f});
//    Point2t p5(p4.v);
//    Point2t p6(p5.v);
//    Point2t p7(p5);
//    Point2t p8(Point2t{1.f,2.f});
//    Point2t p9(std::move(p8));
//    Point2t p10 = {p4.v};

    std::cout << "path ; " << std::filesystem::current_path().string() << "\n";
//     tool::graphics::BiopacControlWindow scene("Biopac controller", tool::geo::Pt2<unsigned int>{1600,1200});
    tool::graphics::DrawSampleWindow scene("Samples", tool::geo::Pt2<unsigned int>{1600,1200});
    scene.init();
    scene.start();

    return 0;
}
