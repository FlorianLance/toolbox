
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
#include "utility/math.hpp"

namespace tool::geo {

template<typename acc>
struct Quaternion;

template<typename acc>
using Quat  = Quaternion<acc>;
using Quatf = Quat<float>;
using Quatd = Quat<double>;

template<class acc>
struct Quaternion{

    acc x{0}; /**< i factor */
    acc y{0}; /**< j factor */
    acc z{0}; /**< k factor */
    acc w{1}; /**< scalar   */

    Quaternion() = default;

    constexpr Quaternion(acc x, acc y, acc z, acc w) : x(x), y(y), z(z), w(w){
    }

    static constexpr Quaternion from_axis(const Vec3<acc> &axe, const acc angle) noexcept{
        const acc radAngle = deg_2_rad(angle/2);
        const acc s = std::sin(radAngle);
        return {axe.x() * s, axe.y() * s, axe.z() * s, cos(radAngle)};
    }

    // TODO: test?
    static constexpr Quaternion from_euler(const Vec3<acc> &eulerAngles) noexcept{
        return
            from_axis({1,0,0}, eulerAngles.x()) *
            from_axis({0,1,0}, eulerAngles.y()) *
            from_axis({0,0,1}, eulerAngles.z());
    }

    // operators
    constexpr Quaternion operator+() const noexcept{
        return *this;
    }

    constexpr Quaternion operator-() const noexcept{
        return {-x, -y, -z, -w};
    }

    constexpr Quaternion &operator+=(acc value) noexcept{
        (*this) = *this + value;
        return *this;
    }

    constexpr Quaternion &operator-=(acc value) noexcept{
        (*this) = *this - value;
        return *this;
    }

    constexpr Quaternion &operator*=(acc value) noexcept{
        (*this) = *this * value;
        return *this;
    }

    constexpr Quaternion &operator/=(acc value){
        (*this) = *this / value;
        return *this;
    }

    constexpr Quaternion &operator+=(const Quaternion &q) noexcept{
        (*this) = *this + q;
        return *this;
    }

    constexpr Quaternion &operator-=(const Quaternion &q) noexcept{
        (*this) = *this + q;
        return *this;
    }

    constexpr Quaternion &operator*=(const Quaternion &q) noexcept{
        (*this) = *this * q;
        return *this;
    }
};

// functions
template <typename acc>
constexpr Quaternion<acc> add(const Quaternion<acc> &q1, const Quaternion<acc> &q2) noexcept{
    return {q1.x+q2.x,q1.y+q2.y,q1.z+q2.z,q1.w+q2.w};
}

template <typename acc>
constexpr Quaternion<acc> add(const Quaternion<acc> &q, acc value) noexcept{
    return {q.x+value, q.y, q.z, q.w};
}

template <typename acc>
constexpr Quaternion<acc> substract(const Quaternion<acc> &q1, const Quaternion<acc> &q2) noexcept{
    return {q1.x-q2.x,q1.y-q2.y,q1.z-q2.z,q1.w-q2.w};
}

template <typename acc>
constexpr Quaternion<acc> substract(const Quaternion<acc> &q, acc value) noexcept{
    return {q.x-value, q.y, q.z, q.w};
}

template <typename acc>
constexpr Quaternion<acc> multiply(const Quaternion<acc> &q, acc value) noexcept{
    return {q.x*value, q.y*value, q.z*value, q.w*value};
}

template <typename acc>
constexpr Quaternion<acc> multiply(const Quaternion<acc> &q1, const Quaternion<acc> &q2) noexcept{
    return {        
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w,
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
    };
}

template <typename acc>
constexpr Quaternion<acc> divide(const Quaternion<acc> &q, acc value) noexcept{
    if(value > acc{0}){
        return {q.x/value, q.y/value, q.z/value, q.w/value};
    }
    return q;
}

template <typename acc>
constexpr bool is_almost_equal(const Quaternion<acc> &l, const Quaternion<acc> &r, int ulp = 3) noexcept{
    return
        almost_equal(l.x, r.x, ulp) &&
        almost_equal(l.y, r.y, ulp) &&
        almost_equal(l.z, r.z, ulp) &&
        almost_equal(l.w, r.w, ulp);
}

template <typename acc>
constexpr acc square_norm(const Quaternion<acc> &q)  noexcept{
    return (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
}

template <typename acc>
acc norm(const Quaternion<acc> &q)  noexcept{
    return sqrt(square_norm(q));
}

template <typename acc>
constexpr Quaternion<acc> conjugate(const Quaternion<acc> &q)noexcept{
    return {-q.x,-q.y,-q.z,q.w};
}

template <typename acc>
constexpr acc dot(const Quaternion<acc> &q1, const Quaternion<acc> &q2) noexcept{
    return {q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w};
}

template <typename acc>
Quaternion<acc> normalize(const Quaternion<acc> &q){
    return divide(q, norm(q));
}

template <typename acc>
Quaternion<acc> inverse(const Quaternion<acc> &q){
    return conjugate(normalize(q));
}

template <typename acc>
acc angle(const Quaternion<acc> &q) noexcept{
    return acos(q.w/norm(q))*2;
}

template <typename acc>
constexpr Vec3<acc> axis(const Quaternion<acc> &q) noexcept{
    auto q1 = normalize(q);
    return normalize(Vec3<acc>{q1.x,q1.y,q1.z});
}

template <typename acc>
constexpr acc pitch(const Quaternion<acc> &q){

    const acc y = acc{2} * (q.y * q.z + q.w * q.x);
    const acc x = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

    if(almost_equal(x,acc{0}) && almost_equal(y,acc{0})){
        return acc{2} * std::atan2(q.x, q.w);
    }
    return std::atan2(y, x);
}

template <typename acc>
constexpr acc yaw(const Quaternion<acc> &q){
    return std::asin(std::clamp(acc{-2} * (q.x * q.z - q.w * q.y),acc{-1}, acc{1}));
}

template <typename acc>
constexpr acc roll(const Quaternion<acc> &q){
    return std::atan2(acc{2} * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}

template <typename acc>
constexpr Vec3<acc> euler_angles(const Quaternion<acc> &q){
    return {pitch(q),yaw(q),roll(q)};
}

template <typename acc>
Quaternion<acc> slerp(const Quaternion<acc> &q1, const Quaternion<acc> &q2, acc t){

    // only unit quaternions are valid rotations.
    Quaternion<acc> v1 = normalize(q1);
    Quaternion<acc> v2 = normalize(q2);

    // compute the cosine of the angle between the two vectors.
    acc dotV = dot(v1, v2);

    // if the dot product is negative, slerp won't take
    // the shorter path. Note that v0 and -v0 are equivalent when
    // the negation is applied to all four components. Fix by
    // reversing one quaternion.
    if (dotV < 0) {
        v2 = -v2;
        dotV = -dotV;
    }

    const acc dotThreshold = static_cast<acc>(0.9995);
    if (dotV > dotThreshold){
        // if the inputs are too close for comfort, linearly interpolateand normalize the result.
        return normalize(v1 + (v2 - v1)*t);
    }

    // Since dot is in range [0, DOT_THRESHOLD], acos is safe
    acc theta_0 = acos(dotV);        // theta_0 = angle between input vectors
    acc theta = theta_0*t;          // theta = angle between v0 and result
    acc sin_theta = sin(theta);     // compute this value only once
    acc sin_theta_0 = sin(theta_0); // compute this value only once
    acc s0 = cos(theta) - dotV * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
    acc s1 = sin_theta / sin_theta_0;

    return (v1*s0) + (v2*s1);
}


// operators
template <typename acc>
constexpr Quaternion<acc> operator+(const Quaternion<acc> &q, acc value) noexcept{
    return add(q,value);
}

template <typename acc>
constexpr Quaternion<acc> operator+(const Quaternion<acc> &q1, const Quaternion<acc> &q2) noexcept{
    return add(q1,q2);
}

template <typename acc>
constexpr Quaternion<acc> operator-(const Quaternion<acc> &q, acc value) noexcept{
    return substract(q,value);
}

template <typename acc>
constexpr Quaternion<acc> operator-(const Quaternion<acc> &q1, const Quaternion<acc> &q2) noexcept{
    return substract(q1,q2);
}

template <typename acc>
constexpr Quaternion<acc> operator*(const Quaternion<acc> &q, acc value) noexcept{
    return multiply(q, value);
}

template <typename acc>
constexpr Quaternion<acc> operator*(const Quaternion<acc> &q1, const Quaternion<acc> &q2) noexcept{
    return multiply(q1,q2);
}

template <typename acc>
constexpr Quaternion<acc> operator/(const Quaternion<acc> &q, acc value){
    return divide(q, value);
}

template <typename acc>
constexpr static bool operator==(const Quaternion<acc> &l, const Quaternion<acc> &r){
    return is_almost_equal(l,r);
}

// operators overload
template <typename acc>
std::ostream &operator<<(std::ostream &flux, const Quaternion<acc> &q){
    flux <<"[" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "]";
    return flux;
}

}


///**
// * Unary division with other Quaternion.
// *
// * Warning: if the norm of y is zero, the result is
// * 4 NaNs, but maybe it should be inf.
// */
//template<typename T1>
//Quaternion operator/=(const Quaternion<T1>& y) {

//  T n2 = y.norm_squared();

//  T at = _a * y.a() + _b * y.b() + _c * y.c() + _d * y.d();
//  T bt = -_a * y.b() + _b * y.a() - _c * y.d() + _d * y.c();
//  T ct = -_a * y.c() + _b * y.d() + _c * y.a() - _d * y.b();
//  T dt = -_a * y.d() - _b * y.c() + _c * y.b() + _d * y.a();

//  _a = at / n2;
//  _b = bt / n2;
//  _c = ct / n2;
//  _d = dt / n2;

//  return *this;
//}
