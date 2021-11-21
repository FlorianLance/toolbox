
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
struct Point3;

template<typename acc>
using Pt3 = Point3<acc>;
using Pt3f = Pt3<float>;
using Pt3d = Pt3<double>;

template<typename acc>
using Vec3  = Pt3<acc>;
using Vec3f = Pt3<float>;
using Vec3d = Pt3<double>;

using Col3 = Pt3<std::uint8_t>;
using Col3f= Pt3f;

template<typename acc>
using Normal3 = Vec3<acc>;

template<typename acc>
struct Point3 : public Point<acc,3>{

    using Point<acc,3>::v;

    Point3() = default;        
    Point3(const Point3& other) = default;
    Point3& operator=(const Point3& other) = default;
    Point3(Point3&& other) = default;
    Point3& operator=(Point3&& other) = default;

    constexpr explicit Point3(const Point<acc,3>& other){
        v = other.v;
    }

    constexpr explicit Point3(Point<acc,3>&& other){
        v = std::move(other.v);
    }

    constexpr explicit Point3(acc v) noexcept {
        this->v = {v,v,v};
    }

    constexpr Point3(acc x, acc y, acc z) noexcept {
        v = {x,y,z};
    }

    inline acc& x() noexcept {return v[0];}
    constexpr acc x() const noexcept {return v[0];}

    inline acc& y() noexcept {return v[1];}
    constexpr acc y() const noexcept {return v[1];}

    inline acc& z() noexcept {return v[2];}
    constexpr acc z() const noexcept {return v[2];}

    template <typename acc2>
    constexpr Point3<acc2> conv() const noexcept{
        return {static_cast<acc2>(x()),static_cast<acc2>(y()),static_cast<acc2>(z())};
    }

    constexpr Point3 operator-() const noexcept{
        return invert(*this);
    }

    inline Point3 &operator+=(const Point3 &pt) noexcept{
        (*this) = *this + pt;
        return *this;
    }

    inline Point3 &operator+=(acc v) noexcept{
        (*this) = *this + v;
        return *this;
    }

    inline Point3 &operator-=(const Point3 &pt) noexcept{
        (*this) = *this - pt;
        return *this;
    }

    inline Point3 &operator-=(acc v) noexcept{
        (*this) = *this - v;
        return *this;
    }

    inline Point3 &operator*=(const Point3 &ptl) noexcept{
        (*this) = *this * ptl;
        return *this;
    }

    inline Point3 &operator*=(acc v) noexcept{
        (*this) = *this * v;
        return *this;
    }

    inline Point3 &operator/=(const Point3 &ptl){
        (*this) = *this / ptl;
        return *this;
    }

    inline Point3 &operator/=(acc v) {
        (*this) = *this / v;
        return *this;
    }
};

// functions
template <typename acc>
constexpr Pt3<acc> add(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return {l.x()+r.x(), l.y()+r.y(),l.z()+r.z()};
}

template <typename acc>
constexpr Pt3<acc> add(const Pt3<acc> &p, acc value) noexcept{
    return {p.x() + value, p.y() + value, p.z() + value};
}

template <typename acc>
constexpr Pt3<acc> add(acc value, const Pt3<acc> &p) noexcept{
    return add(p,value);
}

template <typename acc>
constexpr Pt3<acc> substract(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return {l.x()-r.x(), l.y()-r.y(), l.z()-r.z()};
}

template <typename acc>
constexpr Pt3<acc> substract(const Pt3<acc> &p, acc value) noexcept{
    return {p.x() - value, p.y() - value, p.z() - value};
}

template <typename acc>
constexpr Pt3<acc> multiply(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return {l.x()*r.x(), l.y()*r.y(), l.z()*r.z()};
}

template <typename acc>
constexpr Pt3<acc> multiply(const Pt3<acc> &p, acc value) noexcept{
    return {p.x() * value, p.y() * value, p.z() * value};
}

template <typename acc>
constexpr Pt3<acc> divide(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    const acc zero{0};
    if(r.x() > zero && r.y() > zero && r.z() > zero){
        return {l.x()/r.x(), l.y()/r.y(), l.z()/r.z()};
    }
    return l; // do nothing
}

template <typename acc>
constexpr Pt3<acc> divide(const Pt3<acc> &p, acc value) noexcept{
    if(value > acc{0}){
        return {p.x() / value, p.y() / value, p.z() / value};
    }
    return p; // do nothing
}

template <typename acc>
constexpr acc sum(const Vec3<acc> &vec)  noexcept{
    return vec.x() + vec.y() + vec.z();
}

template <typename acc>
constexpr acc dot(const Vec3<acc> &l, const Vec3<acc> &r) noexcept {
    return sum(multiply(l,r));
}

template<typename acc>
constexpr Pt3<acc> cross(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return {
        l.y() * r.z() - l.z() * r.y(),
        l.z() * r.x() - l.x() * r.z(),
        l.x() * r.y() - l.y() * r.x()
    };

//    __m128 const set0 = _mm_set_ps(0.0f, a.z, a.y, a.x);
//    __m128 const set1 = _mm_set_ps(0.0f, b.z, b.y, b.x);
//    __m128 const xpd0 = glm_vec4_cross(set0, set1);

//    vec<4, float, Q> Result;
//    Result.data = xpd0;
//    return vec<3, float, Q>(Result);

}

template<typename acc>
constexpr Pt3<acc> invert(const Vec3<acc> &vec){
    return multiply(vec,acc{-1});
}

template <typename acc>
inline Vec3<acc> normalize(const Vec3<acc> &vec){
    return divide(vec, norm(vec));
}

template <typename acc>
constexpr acc square_norm(const Vec3<acc> &vec) noexcept {
    return dot(vec,vec);
}

template <typename acc>
inline acc norm(const Pt3<acc> &p) noexcept {
    return sqrt(square_norm(p));
}

// operators
template <typename acc>
constexpr Pt3<acc> operator+(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return add(l,r);
}

template <typename acc>
constexpr Pt3<acc> operator-(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return substract(l,r);
}

template <typename acc>
constexpr Pt3<acc> operator*(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return multiply(l,r);
}

template <typename acc>
constexpr Pt3<acc> operator/(const Pt3<acc> &l, const Pt3<acc> &r) noexcept{
    return divide(l,r);
}

template <typename acc>
constexpr Pt3<acc> operator+(const Pt3<acc> &p, acc value) noexcept{
    return add(p, value);
}
template <typename acc>
constexpr Pt3<acc> operator+(acc value, const Pt3<acc> &p) noexcept{
    return add(p, value);
}

template <typename acc>
constexpr Pt3<acc> operator-(const Pt3<acc> &p, acc value) noexcept{
    return substract(p, value);
}

template <typename acc>
constexpr Pt3<acc> operator*(const Pt3<acc> &p, acc value) noexcept{
    return multiply(p, value);
}

template <typename acc>
constexpr Pt3<acc> operator*(acc value, const Pt3<acc> &p) noexcept{
    return multiply(p, value);
}

template <typename acc>
constexpr Pt3<acc> operator/(const Pt3<acc> &p, acc value) noexcept{
    return divide(p, value);
}
};


