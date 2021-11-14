

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
#include "geometry/point4.hpp"
#include "geometry/matrix4.hpp"
#include "graphics/screen.hpp"



namespace tool::graphics{

struct CameraMatrices{
    using Mat3 = geo::Mat3d;
    using Mat4 = geo::Mat4d;

    Mat4 m;         // model matrix
    Mat4 v;         // view matrix
    Mat4 p;         // projection matrix

    Mat3 normal;    // normal matrix
    Mat4 mv;        // model*view matrix
    Mat4 mvp;       // model*view*projection matrix

    void update(){
        mv     = m * v;
        normal = normal_matrix(mv);
        mvp    = mv * p;
    }

    void update_m(const Mat4 &model){
        m = model;
        update();
    }

    void update_v(const Mat4 &view){
        v = view;
        update();
    }

    void update_p(const Mat4 &proj){
        p = proj;
        update();
    }

    void update_vp(const Mat4 &view, const Mat4 &proj){
        v = view;
        p = proj;
        update();
    }

    void update_mvp(const Mat4 &model, const Mat4 &view, const Mat4 &proj){
        m = model;
        v = view;
        p = proj;
        update();
    }

    static Mat3 normal_matrix(const Mat4 &modelView){
        const auto &mv = modelView;
        return {
            mv.at(0,0),mv.at(0,1),mv.at(0,2),
            mv.at(1,0),mv.at(1,1),mv.at(1,2),
            mv.at(2,0),mv.at(2,1),mv.at(2,2),
        };
    }
};

class Camera{
public:

    using Vec3 = geo::Vec3d;
    using Pt2  = geo::Pt2d;
    using Pt3  = geo::Pt3d;
    using Mat3 = geo::Mat3d;
    using Mat4 = geo::Mat4d;

    enum class Mode : int{
        Perspective, Orhtographic
    };

    Camera(Screen *screen, Pt3 position = {0.,0.,0.}, Vec3 direction = {0.,0.,1.}, Vec3 up = {0.,1.,0.}) :
          m_screen(screen),m_position(position),m_up(up),m_direction(direction), m_initPosition(position), m_initUp(up), m_initDirection(direction) {

        update_right();
        update_view();
        update_projection();
    }

    constexpr Pt3 position() const noexcept{
        return m_position;
    }

    constexpr Vec3 up() const noexcept{
        return m_up;
    }

    constexpr Vec3 direction() const noexcept{
        return m_direction;
    }

    constexpr Mat4 view() const noexcept{
        return m_view;
    }

    Mat4 look_at() const{
        return Mat4::LookAt(m_position, m_position+m_direction, m_up);
    }

    static Mat3 normal(const Mat4 &modelView){
        const auto &mv = modelView;
        return {
            mv.at(0,0),mv.at(0,1),mv.at(0,2),
            mv.at(1,0),mv.at(1,1),mv.at(1,2),
            mv.at(2,0),mv.at(2,1),mv.at(2,2),
        };
    }

    CameraMatrices generate_camera_matrices(){
        CameraMatrices camM;
        camM.v      = m_view;
        camM.p      = m_projection;
        camM.update();
        return camM;
    }

    constexpr Mat4 projection() const noexcept{
        return m_projection;
    }

    constexpr double fov() const noexcept{
        return m_fov;
    }

    constexpr Pt2 z_range() const noexcept{
        return m_zRange;
    }

    constexpr Screen *screen() const noexcept{
        return m_screen;
    }

    // view
    void move_up(double amount){
        m_position += m_up*amount;
        update_view();
    }

    void move_down(double amount){
        m_position -= m_up*amount;
        update_view();
    }

    void move_front(double amount){
        m_position += m_direction*amount;
        update_view();
    }

    void move_left(double amount){
        m_position -= geo::cross(m_direction, m_up)*amount;
        update_view();
    }

    void move_right(double amount){
        m_position += geo::cross(m_direction, m_up)*amount;
        update_view();
    }

    void move_back(double amount){
        m_position -= m_direction*amount;
        update_view();
    }

    void set_position(Pt3 position){
        m_position = position;
        update_view();
    }

    void set_up_vector(Vec3 up){
        m_up = normalize(up);
        update_right();
        update_view();
    }

    void set_direction(double yaw, double pitch, double roll){

        // compute direction vector
        m_direction = Mat4::axis_angle(m_up, yaw).multiply_vector(m_direction);
        m_direction = normalize(Mat4::axis_angle(m_right, pitch).multiply_vector(m_direction));
        // compute up vector
        update_up();
        m_up = Mat4::axis_angle(m_direction, roll).multiply_vector(m_up);
        // m_up.normalize();

        // update right vector
        update_right();
        // update view matrix
        update_view();
    }

    void set_direction(Vec3 direction, Vec3 up = {0.,1.,0.}){

        m_direction = normalize(direction);
        m_up        = normalize(up);

        // update right vector
        update_right();
        // update view matrix
        update_view();
    }

    // projection
    void set_screen(Screen *screen){
        m_screen = screen;
        update_projection();
    }

    void set_fov(double fov){
        m_fov = fov;
        if(m_mode == Mode::Perspective){
            update_projection();
        }
    }

    void set_range(double min, double max){
        m_zRange = {min,max};
        update_projection();
    }

    void reset_init_values(){
        m_position    = m_initPosition;
        m_direction   = m_initDirection;
        m_up          = m_initUp;
        update_right();
        update_view();
    }

    void update_projection(){
        if(m_screen != nullptr){
            const auto ratio = 1.*m_screen->width()/ m_screen->height();
            if(m_mode == Mode::Perspective){
                m_projection = Mat4::Perspective(deg_2_rad(m_fov), ratio, m_zRange.x(), m_zRange.y());
            }else if(m_mode == Mode::Orhtographic){

                m_projection = Mat4::Ortho(
                    -ratio, ratio,
                    -1, 1,
                    m_zRange.x(), m_zRange.y()
                );
            }
        }
    }

    void set_mode(Mode mode){
        m_mode = mode;
        update_projection();
    }

private :

    void update_right(){
        m_right = geo::normalize(geo::cross(m_up, m_direction));
    }

    void update_up(){
        m_up    = geo::normalize(geo::cross(m_direction, m_right));
    }

    void update_view(){
        m_view = Mat4::LookAt(m_position, m_position+m_direction, m_up);
    }

    Screen *m_screen = nullptr;

    Pt3 m_position   = {0.,0.,0.};
    Vec3 m_up        = {0.,1.,0.};
    Vec3 m_direction = {0.,0.,1.};
    Vec3 m_right;

    Pt3 m_initPosition   = {0.,0.,0.};
    Vec3 m_initUp        = {0.,1.,0.};
    Vec3 m_initDirection = {1.,0.,0.};

    Mat4 m_view;
    Mat4 m_projection;

    double m_fov = 60.0;
    Pt2 m_zRange = {0.1, 10000.};

    Mode m_mode = Mode::Perspective;
};
}

