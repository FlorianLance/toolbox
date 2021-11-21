
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
#include "geometry/point3.hpp"

namespace tool::geo {


template<typename acc>
struct Point4;

template<typename acc>
using Pt4 = Point4<acc>;
using Pt4f = Pt4<float>;
using Pt4d = Pt4<double>;

template<typename acc>
using Vec4  = Pt4<acc>;
using Vec4f = Pt4<float>;
using Vec4d = Pt4<double>;

using Col4 = Pt4<std::uint8_t>;
using Col4f= Pt4f;

using RGBA  = Col4;
using RGBAf = Col4f;

template<typename acc>
struct Point4 : public Point<acc,4>{

    using Point<acc,4>::v;

    Point4() = default;
    Point4(const Point4& other) = default;
    Point4& operator=(const Point4& other) = default;
    Point4(Point4&& other) = default;
    Point4& operator=(Point4&& other) = default;

    constexpr explicit Point4(const Point<acc,4>& other){
        v = other.v;
    }

    constexpr explicit Point4(Point<acc,4>&& other){
        v = std::move(other.v);
    }

    constexpr explicit Point4(Pt3<acc> p, acc w) noexcept{
        v = {p.x(), p.y(), p.z(), w};
    }

    constexpr Point4(acc x, acc y, acc z, acc w) noexcept {
        v = {x,y,z,w};
    }

    inline acc& x() noexcept {return v[0];}
    constexpr acc x() const noexcept {return v[0];}

    inline acc& y() noexcept {return v[1];}
    constexpr acc y() const noexcept {return v[1];}

    inline acc& z() noexcept {return v[2];}
    constexpr acc z() const noexcept {return v[2];}

    inline acc& w() noexcept {return v[3];}
    constexpr acc w()const noexcept {return v[3];}

    constexpr Pt3<acc> xyz() const noexcept {return {v[0],v[1],v[2]};}

    template <typename acc2>
    constexpr Point4<acc2> conv() const{
        return {static_cast<acc2>(x()),static_cast<acc2>(y()),static_cast<acc2>(z()),static_cast<acc2>(w())};
    }

    constexpr Point4 operator-() const noexcept{
        return invert(*this);
    }

    inline Point4 &operator+=(const Point4 &pt) noexcept{
        (*this) = *this + pt;
        return *this;
    }

    inline Point4 &operator+=(acc v) noexcept{
        (*this) = *this + v;
        return *this;
    }

    inline Point4 &operator-=(const Point4 &pt) noexcept{
        (*this) = *this - pt;
        return *this;
    }

    inline Point4 &operator-=(acc v) noexcept{
        (*this) = *this - v;
        return *this;
    }

    inline Point4 &operator*=(const Point4 &ptl) noexcept{
        (*this) = *this * ptl;
        return *this;
    }

    inline Point4 &operator*=(acc v) noexcept{
        (*this) = *this * v;
        return *this;
    }

    inline Point4 &operator/=(const Point4 &ptl){
        (*this) = *this / ptl;
        return *this;
    }

    inline Point4 &operator/=(acc v) {
        (*this) = *this / v;
        return *this;
    }

};

// functions
template <typename acc>
constexpr Pt4<acc> add(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    return {l.x()+r.x(), l.y()+r.y(),l.z()+r.z(), l.w() +r.w()};
}

template <typename acc>
constexpr Pt4<acc> add(const Pt4<acc> &p, acc value) noexcept{
    return {p.x() + value, p.y() + value, p.z() + value, p.w() + value};
}

template <typename acc>
constexpr Pt4<acc> add(acc value, const Pt4<acc> &p) noexcept{
    return add(p,value);
}

template <typename acc>
constexpr Pt4<acc> substract(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    return {l.x()-r.x(), l.y()-r.y(), l.z()-r.z(), l.w()-r.w()};
}

template <typename acc>
constexpr Pt4<acc> substract(const Pt4<acc> &p, acc value) noexcept{
    return {p.x() - value, p.y() - value, p.z() - value, p.w() - value};
}

template <typename acc>
constexpr Pt4<acc> multiply(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    return {l.x()*r.x(), l.y()*r.y(), l.z()*r.z(), l.w()*r.w()};
}

template <typename acc>
constexpr Pt4<acc> multiply(const Pt4<acc> &p, acc value) noexcept{
    return {p.x() * value, p.y() * value, p.z() * value, p.w() * value};
}

template <typename acc>
constexpr Pt4<acc> divide(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    const acc zero{0};
    if(r.x() > zero && r.y() > zero && r.z() > zero && r.w() > zero){
        return {l.x()/r.x(), l.y()/r.y(), l.z()/r.z(), l.w()/r.w()};
    }
    return l; // do nothing
}

template <typename acc>
constexpr Pt4<acc> divide(const Pt4<acc> &p, acc value) noexcept{
    if(value > acc{0}){
        return {p.x() / value, p.y() / value, p.z() / value, p.w() / value};
    }
    return p; // do nothing
}

template <typename acc>
constexpr acc sum(const Vec4<acc> &vec)  noexcept{
    return vec.x() + vec.y() + vec.z() + vec.w();
}

template <typename acc>
constexpr acc dot(const Vec4<acc> &l, const Vec4<acc> &r) noexcept {
    return sum(multiply(l,r));
}

template<typename acc>
constexpr Pt4<acc> invert(const Vec4<acc> &vec){
    return multiply(vec,acc{-1});
}

template <typename acc>
inline Vec4<acc> normalize(const Vec4<acc> &vec){
    return divide(vec, norm(vec));
}

template <typename acc>
constexpr acc square_norm(const Vec4<acc> &vec) noexcept {
    return dot(vec,vec);
}

template <typename acc>
inline acc norm(const Pt4<acc> &p) noexcept {
    return sqrt(square_norm(p));
}

// operators
template <typename acc>
constexpr Pt4<acc> operator+(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    return add(l,r);
}

template <typename acc>
constexpr Pt4<acc> operator-(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    return substract(l,r);
}

template <typename acc>
constexpr Pt4<acc> operator*(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    return multiply(l,r);
}

template <typename acc>
constexpr Pt4<acc> operator/(const Pt4<acc> &l, const Pt4<acc> &r) noexcept{
    return divide(l,r);
}

template <typename acc>
constexpr Pt4<acc> operator+(const Pt4<acc> &p, acc value) noexcept{
    return add(p, value);
}
template <typename acc>
constexpr Pt4<acc> operator+(acc value, const Pt4<acc> &p) noexcept{
    return add(p, value);
}

template <typename acc>
constexpr Pt4<acc> operator-(const Pt4<acc> &p, acc value) noexcept{
    return substract(p, value);
}

template <typename acc>
constexpr Pt4<acc> operator*(const Pt4<acc> &p, acc value) noexcept{
    return multiply(p, value);
}

template <typename acc>
constexpr Pt4<acc> operator*(acc value, const Pt4<acc> &p) noexcept{
    return multiply(p, value);
}

template <typename acc>
constexpr Pt4<acc> operator/(const Pt4<acc> &p, acc value) noexcept{
    return divide(p, value);
}

//template<typename acc>
//constexpr Vec4<acc> cross(const Vec4<acc> &l, const Vec4<acc> &r) noexcept{

//    return {
//        l.y() * r.z() - l.z() * r.y(),
//        l.z() * r.x() - l.x() * r.z(),
//        l.x() * r.y() - l.y() * r.x()
//    };
    //    __m128 const set0 = _mm_set_ps(0.0f, a.z, a.y, a.x);
    //    __m128 const set1 = _mm_set_ps(0.0f, b.z, b.y, b.x);
    //    __m128 const xpd0 = glm_vec4_cross(set0, set1);
    //    vec<4, float, Q> Result;
    //    Result.data = xpd0;
    //    return vec<3, float, Q>(Result);
//    glm_vec4 const swp0 = _mm_shuffle_ps(v1, v1, _MM_SHUFFLE(3, 0, 2, 1));
//    glm_vec4 const swp1 = _mm_shuffle_ps(v1, v1, _MM_SHUFFLE(3, 1, 0, 2));
//    glm_vec4 const swp2 = _mm_shuffle_ps(v2, v2, _MM_SHUFFLE(3, 0, 2, 1));
//    glm_vec4 const swp3 = _mm_shuffle_ps(v2, v2, _MM_SHUFFLE(3, 1, 0, 2));
//    glm_vec4 const mul0 = _mm_mul_ps(swp0, swp3);
//    glm_vec4 const mul1 = _mm_mul_ps(swp1, swp2);
//    glm_vec4 const sub0 = _mm_sub_ps(mul0, mul1);
//    return sub0;
//    return {};
//}


}
