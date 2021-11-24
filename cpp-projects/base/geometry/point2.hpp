
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

// local
#include "geometry/point.hpp"

namespace tool::geo {

template<typename acc>
struct Point2;

template<typename acc>
using Pt2 = Point2<acc>;
using Pt2f = Pt2<float>;
using Pt2d = Pt2<double>;

template<typename acc>
using Vec2  = Pt2<acc>;
using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;

template<typename acc>
struct Point2 : public Point<acc,2>{

    using Point<acc,2>::v;

    Point2() = default;
    Point2(const Point2& other) = default;
    Point2& operator=(const Point2& other) = default;
    Point2(Point2&& other) = default;
    Point2& operator=(Point2&& other) = default;

    constexpr explicit Point2(const Point<acc,2>& other){
        v = other.v;
    }

    constexpr explicit Point2(Point<acc,2>&& other){
        v = std::move(other.v);
    }

    constexpr explicit Point2(acc x, acc y) noexcept {
        v = {x,y};
    }

    constexpr Point2(std::initializer_list<acc> l) noexcept{
        std::move(l.begin(), l.end(), std::begin(v));
    }

    inline acc& x() noexcept {return v[0];}
    constexpr acc x() const noexcept {return v[0];}

    inline acc& y() noexcept {return v[1];}
    constexpr acc y() const noexcept {return v[1];}

    template <typename acc2>
    constexpr Point2<acc2> conv() const noexcept{
        return {static_cast<acc2>(x()),static_cast<acc2>(y())};
    }

    constexpr Point2 operator-() const noexcept{
        return invert(*this);
    }

    inline Point2 &operator+=(const Point2 &pt) noexcept{
        (*this) = *this + pt;
        return *this;
    }

    inline Point2 &operator+=(acc v) noexcept{
        (*this) = *this + v;
        return *this;
    }

    inline Point2 &operator-=(const Point2 &pt) noexcept{
        (*this) = *this - pt;
        return *this;
    }

    inline Point2 &operator-=(acc v) noexcept{
        (*this) = *this - v;
        return *this;
    }

    inline Point2 &operator*=(const Point2 &ptl) noexcept{
        (*this) = *this * ptl;
        return *this;
    }

    inline Point2 &operator*=(acc v) noexcept{
        (*this) = *this * v;
        return *this;
    }

    inline Point2 &operator/=(const Point2 &ptl){
        (*this) = *this / ptl;
        return *this;
    }

    inline Point2 &operator/=(acc v) {
        (*this) = *this / v;
        return *this;
    }

};

// functions
template <typename acc>
constexpr Pt2<acc> add(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    return {l.x()+r.x(), l.y()+r.y()};
}

template <typename acc>
constexpr Pt2<acc> add(const Pt2<acc> &p, acc value) noexcept{
    return {p.x() + value, p.y() + value};
}

template <typename acc>
constexpr Pt2<acc> add(acc value, const Pt2<acc> &p) noexcept{
    return add(p,value);
}

template <typename acc>
constexpr Pt2<acc> substract(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    return {l.x()-r.x(), l.y()-r.y()};
}

template <typename acc>
constexpr Pt2<acc> substract(const Pt2<acc> &p, acc value) noexcept{
    return {p.x() - value, p.y() - value};
}

template <typename acc>
constexpr Pt2<acc> multiply(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    return Pt2<acc>{l.x()*r.x(), l.y()*r.y()};
}

template <typename acc>
constexpr Pt2<acc> multiply(const Pt2<acc> &p, acc value) noexcept{
    return Pt2<acc>{p.x() * value, p.y() * value};
}

template <typename acc>
constexpr Pt2<acc> divide(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    const acc zero{0};
    if(r.x() > zero && r.y() > zero){
        return {l.x()/r.x(), l.y()/r.y()};
    }
    return l; // do nothing
}

template <typename acc>
constexpr Pt2<acc> divide(const Pt2<acc> &p, acc value) noexcept{
    if(value > acc{0}){
        return {p.x() / value, p.y() / value};
    }
    return p; // do nothing
}

template <typename acc>
constexpr acc sum(const Vec2<acc> &vec)  noexcept{
    return vec.x() + vec.y();
}

template <typename acc>
constexpr acc dot(const Vec2<acc> &l, const Vec2<acc> &r) noexcept {
    return sum(multiply(l,r));
}

template<typename acc>
constexpr Pt2<acc> invert(const Vec2<acc> &vec){
    return multiply(vec,acc{-1});
}

template <typename acc>
inline Vec2<acc> normalize(const Vec2<acc> &vec){
    return divide(vec, norm(vec));
}

template <typename acc>
constexpr acc square_norm(const Vec2<acc> &vec) noexcept {
    return dot(vec,vec);
}

template <typename acc>
inline acc norm(const Pt2<acc> &p) noexcept {
    return sqrt(square_norm(p));
}

// operators
template <typename acc>
constexpr Pt2<acc> operator+(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    return add(l,r);
}

template <typename acc>
constexpr Pt2<acc> operator-(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    return substract(l,r);
}

template <typename acc>
constexpr Pt2<acc> operator*(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    return multiply(l,r);
}

template <typename acc>
constexpr Pt2<acc> operator/(const Pt2<acc> &l, const Pt2<acc> &r) noexcept{
    return divide(l,r);
}

template <typename acc>
constexpr Pt2<acc> operator+(const Pt2<acc> &p, acc value) noexcept{
    return add(p, value);
}
template <typename acc>
constexpr Pt2<acc> operator+(acc value, const Pt2<acc> &p) noexcept{
    return add(p, value);
}

template <typename acc>
constexpr Pt2<acc> operator-(const Pt2<acc> &p, acc value) noexcept{
    return substract(p, value);
}

template <typename acc>
constexpr Pt2<acc> operator*(const Pt2<acc> &p, acc value) noexcept{
    return multiply(p, value);
}

template <typename acc>
constexpr Pt2<acc> operator*(acc value, const Pt2<acc> &p) noexcept{
    return multiply(p, value);
}

template <typename acc>
constexpr Pt2<acc> operator/(const Pt2<acc> &p, acc value) noexcept{
    return divide(p, value);
}

}
