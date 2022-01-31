
/*******************************************************************************
** Toolbox-3d-engine                                                          **
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

// base
#include "graphics/camera.hpp"

// opengl-utility
#include "opengl/buffer/framebuffer_object.hpp"
#include "opengl/gl_texture.hpp"
#include "opengl/drawer.hpp"

namespace tool::graphics {

class ImguiFboDrawer{

public:

    ImguiFboDrawer() : m_camera(&m_screen, {0,0,0}, {0,0,1}){
        m_camera.set_fov(60.);
    }

    void initialize_gl(const geo::Pt2<int> &size);
    void resize_texture(const geo::Pt2<int> &size);
    void update_viewport();
    void draw_texture(bool invert = false);

    inline void bind(){fbo.bind();}
    inline void unbind(){fbo.unbind();}
    inline graphics::Camera *camera(){return &m_camera;}

    double rotationSpeed = 0.05;
    float scrollSpeed = 0.1f;
    float movingSpeed = 0.05f;
    float translateSpeed = 0.01f;

    void update_texture_with_voxels(gl::ShaderProgram *shader, gl::CloudPointsDrawer *drawer, float sizePtsCloud, float gridVoxelSize);
    void update_texture_with_cloud(gl::ShaderProgram *shader, gl::CloudPointsDrawer *drawer, float sizePtsCloud);

private:

    void check_inputs();

    gl::FBO fbo;
    gl::Texture2D texture;
    gl::RBO depthTexture;

    graphics::Camera m_camera;
    graphics::Screen m_screen;
};
}
