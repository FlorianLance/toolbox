
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
#include "geometry/matrix3.hpp"
#include "utility/math.hpp"

namespace tool::geo {


template<typename acc>
struct OBB3{

    OBB3() = default;

    constexpr OBB3(const Pt3<acc> &p, const Vec3<acc> &s) noexcept : position(p), size(s){
    }

    constexpr OBB3(const Pt3<acc> &p, const Vec3<acc> &s, const Mat3<acc> &o) noexcept : position(p), size(s), orientation(o){
    }

    constexpr Vec3<acc> min() const noexcept{
        const Vec3<acc> p1 = position + size;
        const Vec3<acc> p2 = position - size;
        return Vec3<acc>(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()),std::min(p1.z(), p2.z()));
    }

    constexpr Vec3<acc> max() const noexcept{
        const Vec3<acc> p1 = position + size;
        const Vec3<acc> p2 = position - size;
        return Vec3<acc>(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()),std::max(p1.z(), p2.z()));
    }

    bool is_point_inside(const Pt3<acc> &p){

        const Vec3<acc> dir = p - position;

        for(int ii = 0; ii < 3; ++ii){
            const int id = ii*3;
            const acc distance = dot(dir,{orientation[id], orientation[id + 1], orientation[id + 2]});

            const bool equal1 = almost_equal<acc>(distance, size[ii]);
            const bool equal2 = almost_equal<acc>(distance, -size[ii]);

            if(distance > size[ii] && !equal1){
                return false;
            }
            if(distance < -size[ii] && !equal2){
                return false;
            }
        }
        return true;
    }

    Pt3<acc> position = {0,0,0};
    Vec3<acc> size = {1,1,1};
    Mat3<acc> orientation{};
};


//template <typename acc>
//static constexpr bool point_in_obb(const Pt3<acc> &p, const OBB3<acc> &obb) noexcept {

//    const Vec3<acc> dir = p - obb.position;
//    for(int ii = 0; ii < 3; ++ii){
//        const int id = ii*3;
//        const acc distance = dot(dir,{obb.orientation[id], obb.orientation[id + 1],obb.orientation[id + 2]});

//        const bool equal1 = almost_equal<acc>(distance, obb.size[ii]);
//        const bool equal2 = almost_equal<acc>(distance, -obb.size[ii]);

//        if(distance > obb.size[ii] && !equal1){
//            return false;
//        }
//        if(distance < -obb.size[ii] && !equal2){
//            return false;
//        }
//    }
//    return true;
//}


//template <typename acc>
//constexpr Pt3<acc> closest_point(const OBB3<acc> &obb, const Pt3<acc> &p) noexcept{ // TODO : manage float accuracy

//    Pt3<acc> res = p;
//    const Vec3<acc> dir = p - obb.position;

//    for(int ii = 0; ii < 3; ++ii){
//        int id = ii*3;
//        Vec3<acc> axis = {obb.orientation[id], obb.orientation[id + 1],obb.orientation[id + 2]};

//        acc distance = dot(dir,axis);
//        if(distance > obb.size[ii]){
//            distance = obb.size[ii];
//        }
//        if(distance < -obb.size[ii]){
//            distance = -obb.size[ii];
//        }

//        res += (axis*distance);

//    }

//    return res;
//}
}
