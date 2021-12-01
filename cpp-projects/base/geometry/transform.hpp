
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
#include "geometry/matrix4.hpp"
#include "geometry/quaternion.hpp"
#include "utility/vector.hpp"

namespace tool::geo {

    template<class acc>
    class Transform {

    private:

        Quat<acc> rotation    = {0,0,0,1};
        Vec3<acc> translation = {0,0,0};
        Vec3<acc> scaling     = {1,1,1};

        Vec3<acc> right     = {1,0,0};
        Vec3<acc> up        = {0,1,0};
        Vec3<acc> foward    = {0,0,1};

    public:

        Transform *parent = nullptr;
        std_v1<Transform*> children;


//        constexpr Vec3<acc> local_euler_angles()const{
//            return rotation.
//        }

//        Camera(){
//            right = geo::normalized(geo::cross(up, direction));
//        }

//        Camera(Pt3 p, Vec3 d, Vec3 u = {0.,1.,0.}) :
//            position(p),up(u),direction(d), rPosition(p), rUp(u),rDirection(d) {
//            right       = geo::normalized(geo::cross(up, direction));
//        }

//        void set_direction(double yaw, double pitch, double roll){
//            direction = Mat4::axis_angle(up, yaw).multiply_vector(direction);
//            direction = Mat4::axis_angle(right, pitch).multiply_vector(direction);
//            direction.normalize();
//            up      = geo::normalized(geo::cross(direction, right));
//            up      = Mat4::axis_angle(direction, roll).multiply_vector(up);
//            up.normalize();
//            right   = geo::normalized(geo::cross(up, direction));
//        }

//        void move_up(double amount){
//            position += up*amount;
//        }

//        void move_down(double amount){
//            position -= up*amount;
//        }

//        void move_front(double amount){
//            position += direction*amount;
//        }

//        void move_left(double amount){
//            position += geo::cross(direction, up)*amount;
//        }

//        void move_right(double amount){
//            position -= geo::cross(direction, up)*amount;
//        }

//        void move_back(double amount){
//            position -= direction*amount;
//        }

//        geo::Mat4<double> view(){
//            return Mat4::look_at(position, direction, up);
//        }

//        void reset(){
//            position    = rPosition;
//            direction   = rDirection;
//            up          = rUp;
//            right       = geo::normalized(geo::cross(up, direction));
//        }

//        Pt3 position   = {0.,0.,0.};
//        Vec3 up        = {0.,1.,0.};
//        Vec3 direction = {0.,0.,1.};
//        Vec3 right;


//        localEulerAngles	The rotation as Euler angles in degrees relative to the parent transform's rotation.
//        localPosition	Position of the transform relative to the parent transform.
//        localRotation	The rotation of the transform relative to the transform rotation of the parent.
//        localScale	The scale of the transform relative to the GameObjects parent.

//        DetachChildren	Unparents all children.
//        Find	Finds a child by n and returns it.
//        GetChild	Returns a transform child by index.
//        GetSiblingIndex	Gets the sibling index.
//        InverseTransformDirection	Transforms a direction from world space to local space. The opposite of Transform.TransformDirection.
//        InverseTransformPoint	Transforms position from world space to local space.
//        InverseTransformVector	Transforms a vector from world space to local space. The opposite of Transform.TransformVector.
//        IsChildOf	Is this transform a child of parent?
//        LookAt	Rotates the transform so the forward vector points at /target/'s current position.
//        Rotate	Use Transform.Rotate to rotate GameObjects in a variety of ways. The rotation is often provided as an Euler angle and not a Quaternion.
//        RotateAround	Rotates the transform about axis passing through point in world coordinates by angle degrees.
//        SetAsFirstSibling	Move the transform to the start of the local transform list.
//        SetAsLastSibling	Move the transform to the end of the local transform list.
//        SetParent	Set the parent of the transform.
//        SetPositionAndRotation	Sets the world space position and rotation of the Transform component.
//        SetSiblingIndex	Sets the sibling index.
//        TransformDirection	Transforms direction from local space to world space.
//        TransformPoint	Transforms position from local space to world space.
//        TransformVector	Transforms vector from local space to world space.
//        Translate	Moves the transform in the direction and distance of translation.



    };


    template<class acc>
    constexpr Mat4<acc> to_mat4(const Quat<acc> &q){

        const acc xx      = q.x * q.x;
        const acc xy      = q.x * q.y;
        const acc xz      = q.x * q.z;
        const acc xw      = q.x * q.w;

        const acc yy      = q.y * q.y;
        const acc yz      = q.y * q.z;
        const acc yw      = q.y * q.w;

        const acc zz      = q.z * q.z;
        const acc zw      = q.z * q.w;

        return {
            1 - 2 * ( yy + zz ),    2 * ( xy + zw ),        2 * ( xz - yw ),        0,
            2 * ( xy - zw ),        1 - 2 * ( xx + zz ),    2 * ( yz + xw ),        0,
            2 * ( xz + yw ),        2 * ( yz - xw ),        1 - 2 * ( xx + yy ),    0,
            0,                      0,                      0,                      1
        };
    }

    template<class acc>
    Quat<acc> to_quaternion(const Mat4<acc> &m){

        const auto t = m.trace();
        if(t < acc{0}){
            const auto s = sqrt(acc{1} + m(0,0) - m(1,1) - m(2,2))*acc{2};
            return {
                acc{0.25} *s,
                (m.at(1,0) + m.at(0,1))/s,
                (m.at(2,0) + m.at(0,2))/s,
                (m.at(1,2) - m.at(2,1))/s
            };
        }else{
            const auto s = acc{0.5} / sqrt(t);
            return {
                (m.at(1,2) - m.at(2,1))*s,
                (m.at(2,0) - m.at(0,2))*s,
                (m.at(0,1) - m.at(1,0))*s,
                acc{0.25} / s
            };
        }

//        T fourXSquaredMinus1 = m[0][0] - m[1][1] - m[2][2];
//        T fourYSquaredMinus1 = m[1][1] - m[0][0] - m[2][2];
//        T fourZSquaredMinus1 = m[2][2] - m[0][0] - m[1][1];
//        T fourWSquaredMinus1 = m[0][0] + m[1][1] + m[2][2];

//        int biggestIndex = 0;
//        T fourBiggestSquaredMinus1 = fourWSquaredMinus1;
//        if(fourXSquaredMinus1 > fourBiggestSquaredMinus1)
//        {
//            fourBiggestSquaredMinus1 = fourXSquaredMinus1;
//            biggestIndex = 1;
//        }
//        if(fourYSquaredMinus1 > fourBiggestSquaredMinus1)
//        {
//            fourBiggestSquaredMinus1 = fourYSquaredMinus1;
//            biggestIndex = 2;
//        }
//        if(fourZSquaredMinus1 > fourBiggestSquaredMinus1)
//        {
//            fourBiggestSquaredMinus1 = fourZSquaredMinus1;
//            biggestIndex = 3;
//        }

//        T biggestVal = sqrt(fourBiggestSquaredMinus1 + static_cast<T>(1)) * static_cast<T>(0.5);
//        T mult = static_cast<T>(0.25) / biggestVal;

//        switch(biggestIndex)
//        {
//        case 0:
//            return qua<T, Q>(biggestVal, (m[1][2] - m[2][1]) * mult, (m[2][0] - m[0][2]) * mult, (m[0][1] - m[1][0]) * mult);
//        case 1:
//            return qua<T, Q>((m[1][2] - m[2][1]) * mult, biggestVal, (m[0][1] + m[1][0]) * mult, (m[2][0] + m[0][2]) * mult);
//        case 2:
//            return qua<T, Q>((m[2][0] - m[0][2]) * mult, (m[0][1] + m[1][0]) * mult, biggestVal, (m[1][2] + m[2][1]) * mult);
//        case 3:
//            return qua<T, Q>((m[0][1] - m[1][0]) * mult, (m[2][0] + m[0][2]) * mult, (m[1][2] + m[2][1]) * mult, biggestVal);
//        default: // Silence a -Wswitch-default warning in GCC. Should never actually get here. Assert is just for sanity.
//            assert(false);
//            return qua<T, Q>(1, 0, 0, 0);
//        }
    }

}
