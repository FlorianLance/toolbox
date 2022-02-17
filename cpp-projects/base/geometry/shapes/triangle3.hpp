

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
#include "geometry/interval.hpp"
#include "geometry/point3.hpp"
#include "geometry/point2.hpp"

namespace tool::geo {


template<typename acc>
struct Triangle3{

    inline Triangle3(const Pt3<acc> &pa, const Pt3<acc> &pb, const Pt3<acc> &pc) : v({pa,pb,pc}){
    }

    inline Pt3<acc> a() const {return v[0];}
    inline Pt3<acc> b() const {return v[1];}
    inline Pt3<acc> c() const {return v[2];}

    inline Pt3<acc>& a(){return v[0];}
    inline Pt3<acc>& b(){return v[1];}
    inline Pt3<acc>& c(){return v[2];}

    std_a1<Pt3<acc>,3> v;
};

template<typename acc>
Interval<acc> interval(const Triangle3<acc> &t, const Vec3<acc> &axis){

    Interval<acc> res;
    res.min() = dot(axis, t.a());
    res.max() = res.min();

    for(int ii = 1; ii < t.v.size(); ++ii){
        acc value = dot(axis, t.v[ii]);
        res.min() = std::min(res.min(), value);
        res.max() = std::min(res.max(), value);
    }

    return res;
}

template <typename acc>
Pt3<acc> barycentric(const Pt3<acc> &p, const Triangle3<acc> &t){

    const Vec3<acc> ab = t.b() - t.a();
    const Vec3<acc> bc = t.c() - t.b();
    const Vec3<acc> ca = t.a() - t.c();

    Vec3<acc> v = ab - project(ab,vec(t.c(), t.b()));
    const acc a = 1 - (dot(v, vec(t.a(),p)) / dot(v,ab));

    v = bc - project(bc, vec(t.a(),t.c()));
    const acc b = 1 - (dot(v,vec(t.b(),p)) / dot(v, bc));

    v = ca - project(ca, ab);
    const acc c = 1 - (dot(v, vec(t.c(),p)) / dot(v, ca));

    return Pt3<acc>(a,b,c);
}


template <typename acc>
Pt3<acc> generate_point(const Pt2<acc> &factors, const Triangle3<acc> &t) noexcept{

    Vec3<acc> ab = vec(t.a(),t.b());
    Vec3<acc> ac = vec(t.a(),t.c());
    return t.a() + ab*factors[0] + ac*factors[1];
}


template <typename acc>
Pt3<acc> barycenter_to_point(const Pt3<acc> &barycenter, const Triangle3<acc> &t) noexcept{

    // TODO check x + y + z == 1
    return Pt3<acc>{
        barycenter.x()*t.a().x() + barycenter.y()*t.b().x() + barycenter.z()*t.c().x(),
        barycenter.x()*t.a().y() + barycenter.y()*t.b().y() + barycenter.z()*t.c().y(),
        barycenter.x()*t.a().z() + barycenter.y()*t.b().z() + barycenter.z()*t.c().z()
    };
}


struct TriIds{

    TriIds() = default;

    inline TriIds(size_t id1, size_t id2, size_t id3) : ids(Pt3<size_t>{id1,id2,id3}){
    }

    constexpr size_t id1() const{return ids.x();}
    constexpr size_t id2() const{return ids.y();}
    constexpr size_t id3() const{return ids.z();}    

    Pt3<size_t> ids;
};
}
