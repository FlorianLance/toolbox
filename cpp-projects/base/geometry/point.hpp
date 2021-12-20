
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
#include <functional>
#include <numeric>
#include <ostream>
#include <algorithm>

// local
#include "utility/array.hpp"
#include "utility/math.hpp"

namespace tool::geo {

// aliases
template<typename acc, std::size_t dim>
struct Point;

template<typename acc, std::size_t dim>
using Pt = Point<acc, dim>;

template<typename acc, std::size_t dim>
using Vector = Pt<acc, dim>;

template<typename acc, std::size_t dim>
using Vec = Vector<acc, dim>;

template<typename acc, std::size_t dim>
using Normal = vecN<acc, dim>;

// point
template<typename acc, std::size_t dim>
struct Point{

    static_assert(dim != 0, "Point must have a dimension > 0" );

    Point() = default;
    Point(const Point& other) = default;
    Point& operator=(const Point& other) = default;
    Point(Point&& other) = default;
    Point& operator=(Point&& other) = default;

    constexpr Point(std::initializer_list<acc> l) noexcept{
        std::move(l.begin(), l.end(), v.begin());
    }

    constexpr Point(const vecN<acc,dim> &array) noexcept{
        v = array;
    }

    constexpr Point(vecN<acc,dim> &&array) noexcept{
        v = std::move(array);
    }

    constexpr std::size_t size()const noexcept {return dim;}      

    constexpr Pt<acc,dim>& operator=(vecN<acc,dim> array) noexcept{
        v = std::move(array);
        return *this;
    }

    constexpr acc& operator()(int id) {return v[id];}
    constexpr acc operator()(int id)  const  { return v[id];}
    constexpr acc& operator[](int id) {return v[id];}
    constexpr acc operator[](int id) const { return v[id];}

    inline Pt<acc, dim> &operator+=(const Pt<acc, dim> &pt) noexcept{
        (*this) = *this + pt;
        return *this;
    }

    inline Pt<acc, dim> &operator+=(acc v) noexcept{
        (*this) = *this + v;
        return *this;
    }

    inline Pt<acc, dim> &operator-=(const Pt<acc, dim> &pt) noexcept{
        (*this) = *this - pt;
        return *this;
    }

    inline Pt<acc, dim> &operator-=(acc v) noexcept{
        (*this) = *this - v;
        return *this;
    }

    inline Pt<acc, dim> &operator*=(const Pt<acc, dim> &ptl) noexcept{
        (*this) = *this * ptl;
        return *this;
    }

    inline Pt<acc, dim> &operator*=(acc v) noexcept{
        (*this) = *this * v;
        return *this;
    }

    inline Pt<acc, dim> &operator/=(const Pt<acc, dim> &ptl){
        (*this) = *this / ptl;
        return *this;
    }

    inline Pt<acc, dim> &operator/=(acc v){
        (*this) = *this / v;
        return *this;
    }

    vecN<acc, dim> v = {};
};


// functions
//template <typename acc, std::size_t dim>
//inline acc square_norm(const Pt<acc,dim> &p) noexcept {
//    return dot(p,p);
//}
//template <typename acc, std::size_t dim>
//inline acc norm(const Pt<acc,dim> &p) noexcept {
//    return sqrt(square_norm(p));
//}

template <typename acc, std::size_t dim>
Pt<acc,dim> add(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    Pt<acc,dim> res;
    std::transform(l.v.cbegin(), l.v.cend(), r.v.cbegin(), res.v.begin(), std::plus<acc>());
    return res;
}

//template <typename acc, std::size_t dim>
//Pt<acc,dim> add(const Pt<acc,dim> &p, acc value){
//    Pt<acc,dim> res;
//    std::transform(p.v.cbegin(), p.v.cend(), res.v.begin(), std::bind(std::plus<acc>(), std::placeholders::_1, value));
//    return res;
//}

template <typename acc, std::size_t dim>
Pt<acc,dim> substract(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    Pt<acc,dim> res;
    std::transform(l.v.cbegin(), l.v.cend(), r.v.cbegin(), res.v.begin(), std::minus<acc>());
    return res;
}

//template <typename acc, std::size_t dim>
//Pt<acc,dim> substract(const Pt<acc,dim> &p, acc value){
//    Pt<acc,dim> res;
//    std::transform(p.v.cbegin(), p.v.cend(), res.v.begin(), std::bind(std::minus<acc>(), std::placeholders::_1, value));
//    return res;
//}

template <typename acc, std::size_t dim>
Pt<acc,dim> multiply(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    Pt<acc,dim> res;
    std::transform(l.v.cbegin(), l.v.cend(), r.v.cbegin(), res.v.begin(), std::multiplies<acc>());
    return res;
}

template <typename acc, std::size_t dim>
Pt<acc,dim> multiply(const Pt<acc,dim> &p, acc value){
    Pt<acc,dim> res;
    std::transform(p.v.cbegin(), p.v.cend(), res.v.begin(), std::bind(std::multiplies<acc>(), std::placeholders::_1, value));
    return res;
}

template <typename acc, std::size_t dim>
Pt<acc,dim> divide(const Pt<acc,dim> &l, const Pt<acc,dim> &r){

    if(r > acc{0}){
        Pt<acc,dim> res;
        std::transform(l.v.cbegin(), l.v.cend(), r.v.cbegin(), res.v.begin(), std::divides<acc>());
        return res;
    }
    return Pt<acc,dim>{};
}

template <typename acc, std::size_t dim>
Pt<acc,dim> divide(const Pt<acc,dim> &p, acc value){
    if(value > acc{0}){
        Pt<acc,dim> res;
        std::transform(p.v.cbegin(), p.v.cend(), res.v.begin(), std::bind(std::divides<acc>(), std::placeholders::_1, value));
        return res;
    }
    return p;
}

template <typename acc, std::size_t dim>
Pt<acc,dim> invert(const Pt<acc,dim> &p){
    return multiply(p,acc{-1});
}

template <typename acc, std::size_t dim>
static bool equals(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    return is_almost_equal(l,r);
}

template <typename acc, std::size_t dim>
constexpr Vec<acc, dim> vec(const Pt<acc, dim> &l, const Pt<acc, dim> &r) noexcept {
    return r-l;
}

//template <typename acc, std::size_t dim>
//acc dot(const Vec<acc, dim> &l, const Vec<acc, dim> &r) noexcept {
//    return std::inner_product(l.v.cbegin(), l.v.cend(), r.v.cbegin(), acc{0});
//}

template <typename acc, std::size_t dim>
constexpr acc sum(const Vector<acc,dim> &vec)  noexcept{
    // TODO : need constexpr std::accumulate (C++20?)
    acc s{0};
    for(size_t ii = 0; ii < dim; ++ii){
        s += vec.v[ii];
    }
    return s;
}

//template <typename acc, std::size_t dim>
//Vector<acc,dim> normalize(const Vector<acc,dim> &vec){
//    return divide(vec, vec.norm());
//}

template <typename acc, std::size_t dim>
bool is_almost_equal(const Pt<acc,dim> &l, const Pt<acc,dim> &r, int ulp = 3) noexcept{
    for(auto it1 = l.v.cbegin(), it2 = r.v.cbegin(); it1 != l.v.cend(); ++it1, ++it2){
        if(!almost_equal(*it1, *it2, ulp)){
            return false;
        }
    }
    return true;
}

// operators
template <typename acc, std::size_t dim>
std::ostream &operator<<(std::ostream &flux, const Pt<acc,dim> &p){
    flux <<"[";
    for(const auto& val : p.v){
        flux << val << " ";
    }
    flux << "]";
    return flux;
}

template <typename acc, std::size_t dim>
Pt<acc,dim> operator+(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    return add(l,r);
}

//template <typename acc, std::size_t dim>
//Pt<acc,dim> operator+(const Pt<acc,dim> &p, acc value){
//    return add(p,value);
//}

template <typename acc, std::size_t dim>
Pt<acc,dim> operator-(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    return substract(l,r);
}

//template <typename acc, std::size_t dim>
//Pt<acc,dim> operator-(const Pt<acc,dim> &p, acc value){
//    return substract(p,value);
//}

template <typename acc, std::size_t dim>
Pt<acc,dim> operator*(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    return multiply(l,r);
}

template <typename acc, std::size_t dim>
Pt<acc,dim> operator*(const Pt<acc,dim> &p, acc value){
    return multiply(p,value);
}

template <typename acc, std::size_t dim>
Pt<acc,dim> operator/(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    return divide(l,r);
}


template <typename acc, std::size_t dim>
Pt<acc,dim> operator/(const Pt<acc,dim> &p, acc value){
    return divide(p,value);
}

template <typename acc, std::size_t dim>
Pt<acc,dim> operator-(const Pt<acc,dim> &p){
    return invert(p);
}

template <typename acc, std::size_t dim>
static bool operator==(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    return is_almost_equal(l,r);
}

template <typename acc, std::size_t dim>
static bool operator!=(const Pt<acc,dim> &l, const Pt<acc,dim> &r){
    return !is_almost_equal(l,r);
}

template <typename acc, std::size_t dim>
bool operator<(const Pt<acc,dim> &p, acc value){

    for(const auto &pval : p.v){
        if(pval < value){
            continue;
        }
        return false;
    }

    return true;
}


template <typename acc, std::size_t dim>
bool operator>(const Pt<acc,dim> &p, acc value){

    for(const auto &pval : p.v){
        if(pval > value){
            continue;
        }
        return false;
    }

    return true;
}

}
