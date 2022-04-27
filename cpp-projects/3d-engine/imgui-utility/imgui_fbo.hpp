
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

namespace tool::graphics {


class ImguiFboUiDrawer{

public:

    ImguiFboUiDrawer();

    void init();
    void resize(const geo::Pt2<int> &size);
    void draw();

    gl::TextureName texture_id()const{return m_texture.id();}

    void update_viewport();
    void set_gl_states(geo::Pt4f color = {0.0f, 0.0f, 0.0f, 1.0f});

    inline void bind(){m_fbo.bind();}
    inline void unbind(){m_fbo.unbind();}
    inline graphics::Camera *camera(){return &m_camera;}
    bool is_camera_updated()const{return m_cameraUpdated;}
    bool is_screen_updated()const{return m_screenUpdated;}
    void reset_states(){m_cameraUpdated=false;m_screenUpdated=false;}

    double rotationSpeed = 0.05;
    float scrollSpeed = 0.1f;
    float movingSpeed = 0.05f;
    float translateSpeed = 0.01f;
    bool invertTexture = true;

private:

    void restore_viewport();
    void check_inputs();

    GLint m_viewport[4];
    gl::FBO m_fbo;
    gl::Texture2D m_texture;
    gl::RBO m_depthTexture;
    graphics::Camera m_camera;       
    graphics::Screen m_screen;
    bool m_cameraUpdated = false;
    bool m_screenUpdated = false;
};
}
