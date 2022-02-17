
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
//#include "geometry/point3.hpp"
//#include "geometry/ray3.hpp"
//#include "geometry/triangle3.hpp"
//#include "geometry/plane3.hpp"
//#include "geometry/aabb3.hpp"

/*namespace tool::geo {

template<typename acc>
struct RaycastResult {

    Vec3<acc> point = {0,0,0};
    Vec3<acc> normal = {0, 0, 1};
    acc t = acc{-1};
    bool hit = false;
};

template <typename acc>
acc raycast_plane(const Plane3<acc> &plane, const Ray3<acc> &ray){
    acc nd = dot(ray.direction, plane.normal);
    acc pn = dot(ray.origin, plane.normal);

    if(nd >= acc{0}){
        return acc{-1};
    }
    acc t = (plane.distance - pn) / nd;
    if(t >= acc{0}){
        return t;
    }

    return acc{-1};
}

template<typename acc>
bool raycast_plane(const Plane3<acc>& plane, const Ray3<acc>& ray, RaycastResult<acc>* outResult) {

    if(outResult != nullptr){
        *outResult = RaycastResult<acc>();
    }

    acc nd = dot(ray.direction, plane.normal);
    acc pn = dot(ray.origin,    plane.normal);

    // nd must be negative, and not 0
    // if nd is positive, the ray and plane normals
    // point in the same direction. No intersection.
    if (!(nd < 0)) {
        return false;
    }

    acc t = (plane.distance - pn) / nd;

    // t must be positive
    if (!(t < 0)) {
        if (outResult != 0) {
            outResult->t = t;
            outResult->hit = true;
            outResult->point = ray.origin + ray.direction * t;
            outResult->normal = normalize(plane.normal);
        }
        return true;
    }

    return false;
}

template <typename acc>
acc raycast_triangle(const Triangle3<acc> &tri, const Ray3<acc> &ray){

    Plane3<acc> plane = from_triangle(tri);
    acc t = raycast_plane(plane, ray);
    if(t < acc{0}){
        return t;
    }

    Pt3<acc> result = ray.origin + ray.direction * t;
    Pt3<acc> bary = barycentric(result, tri);
    if( bary.x() >= 0.f && bary.x() <= 1.f &&
        bary.y() >= 0.f && bary.y() <= 1.f &&
        bary.z() >= 0.f && bary.z() <= 1.f){
        return t;
    }
    return acc{-1};
}

template<typename acc>
bool raycast_triangle(const Triangle3<acc> &triangle, const Ray3<acc>& ray, RaycastResult<acc> *outResult){

    if(outResult != nullptr){
        *outResult = RaycastResult<acc>();
    }

    Plane3<acc> plane = from_triangle(triangle);
    RaycastResult<acc> planeResult;
    if (!raycast_plane(plane, ray, &planeResult)) {
        return false;
    }

    acc t = planeResult.t;
    Pt3<acc> result = ray.origin + ray.direction * t;
    Vec3<acc> barycentricPt = barycentric(result, triangle);
    if (!(barycentricPt.x() < 0) && !(barycentricPt.x() > 1) &&
        !(barycentricPt.y() < 0) && !(barycentricPt.y() > 1) &&
        !(barycentricPt.z() < 0) && !(barycentricPt.z() > 1)) {

        if (outResult != nullptr) {
            outResult->t = t;
            outResult->hit = true;
            outResult->point = ray.origin + ray.direction * t;
            outResult->normal = plane.normal;
        }

        return true;
    }

    return false;
}

template<typename acc>
bool raycast_aabb(const AABB3<acc>& aabb, const Ray3<acc>& ray, RaycastResult<acc>* outResult) {

    if(outResult != nullptr){
        *outResult = RaycastResult<acc>();
    }

    Pt3<acc> pmin = aabb.min();
    Pt3<acc> pmax = aabb.max();

    // Any component of direction could be 0!
    // Address this by using a small number, close to
    // 0 in case any of directions components are 0
    acc t1 = (pmin.x() - ray.origin.x()) / (almost_equal<acc>(ray.direction.x(), acc{0}) ? static_cast<acc>(0.00001) : ray.direction.x());
    acc t2 = (pmax.x() - ray.origin.x()) / (almost_equal<acc>(ray.direction.x(), acc{0}) ? static_cast<acc>(0.00001) : ray.direction.x());
    acc t3 = (pmin.y() - ray.origin.y()) / (almost_equal<acc>(ray.direction.y(), acc{0}) ? static_cast<acc>(0.00001) : ray.direction.y());
    acc t4 = (pmax.y() - ray.origin.y()) / (almost_equal<acc>(ray.direction.y(), acc{0}) ? static_cast<acc>(0.00001) : ray.direction.y());
    acc t5 = (pmin.z() - ray.origin.z()) / (almost_equal<acc>(ray.direction.z(), acc{0}) ? static_cast<acc>(0.00001) : ray.direction.z());
    acc t6 = (pmax.z() - ray.origin.z()) / (almost_equal<acc>(ray.direction.z(), acc{0}) ? static_cast<acc>(0.00001) : ray.direction.z());

    acc tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
    acc tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

    // if tmax < 0, ray is intersecting AABB
    // but entire AABB is behing it's origin
    if (tmax < 0) {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) {
        return false;
    }

    acc result = tmin;

    // If tmin is < 0, tmax is closer
    if (tmin < 0) {
        result = tmax;
    }

    if (outResult != nullptr) {
        outResult->t = result;
        outResult->hit = true;
        outResult->point = ray.origin + ray.direction * result;

        std_a1<Vec3<acc>,6> normals = {
            Vec3<acc>(-1, 0, 0),
            Vec3<acc>(1, 0, 0),
            Vec3<acc>(0, -1, 0),
            Vec3<acc>(0, 1, 0),
            Vec3<acc>(0, 0, -1),
            Vec3<acc>(0, 0, 1)
        };
        std_a1<acc,6> t = {t1, t2, t3, t4, t5, t6};
        for(size_t ii = 0; ii < normals.size(); ++ii){
            if(almost_equal<acc>(result, t[ii])){
                outResult->normal = normals[ii];
            }
        }
    }


    return true;
}
}
*/
