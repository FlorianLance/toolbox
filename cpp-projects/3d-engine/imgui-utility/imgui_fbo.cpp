
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


#include "imgui_fbo.hpp"

// imgui
#include "imgui/imgui.h"

// local
#include "imgui-utility/imgui_std.hpp"

using namespace tool::graphics;


ImguiFboUiDrawer::ImguiFboUiDrawer() : m_camera(&m_screen, {0,0,0}, {0,0,1}){
    m_camera.set_fov(60.);
}

void ImguiFboUiDrawer::init(){
    m_fbo.clean();
    m_fbo.generate();
}

void ImguiFboUiDrawer::resize(const geo::Pt2<int> &size){

    if(m_texture.width() == size.x() && m_texture.height() == size.y()){
        return;
    }
    m_screenUpdated = true;

    m_screen.resize(size.x(), size.y());
    m_camera.update_projection();

    m_texture.clean();
    m_texture.init_image_8ui(size.x(),size.y(), 3);

    TextureOptions options;
    options.minFilter = TextureMinFilter::linear;
    options.magFilter = TextureMagFilter::linear;
    m_texture.set_texture_options(options);

    m_depthTexture.clean();
    m_depthTexture.generate();
    m_depthTexture.bind();
    m_depthTexture.set_data_storage(size.x(), size.y());

    m_fbo.attach_colors_textures({
        &m_texture
    });

    m_fbo.attach_depth_buffer(m_depthTexture);

    m_fbo.set_draw_buffers({
        tool::gl::FrameBufferAttachment::color0,
    });
}

void ImguiFboUiDrawer::update_viewport(){

    if(m_texture.id() == 0){
        return;
    }

    glGetIntegerv(GL_VIEWPORT, m_viewport);
    glViewport(0,0, m_texture.width(), m_texture.height());
}

void ImguiFboUiDrawer::reset_gl_states(geo::Pt4f color){
    glClearColor(color.x(),color.y(),color.z(),color.w());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
}

void ImguiFboUiDrawer::draw(){

    auto size    = ImGui::content_region_size_available();
    float scale  = std::min(1.f*size.y() / m_texture.height(),  1.f*size.x() / m_texture.width());
    ImVec2 sizeI = ImVec2(m_texture.width() * scale,m_texture.height() * scale);

    if(m_texture.id() == 0){
        ImGui::Text("Texture not initialized.");
    }else{
        if(invertTexture){
            ImGui::Image(m_texture.id(), sizeI,  ImVec2(0,1), ImVec2(1,0));
        }else{
            ImGui::Image(m_texture.id(), sizeI,  ImVec2(0,0), ImVec2(1,1));
        }
    }

    check_inputs();
}

void ImguiFboUiDrawer::restore_viewport(){
    // restore
    gl::FBO::unbind();
    glViewport(
        static_cast<GLsizei>(m_viewport[0]),
        static_cast<GLsizei>(m_viewport[1]),
        static_cast<GLsizei>(m_viewport[2]),
        static_cast<GLsizei>(m_viewport[3])
    );
}

void ImguiFboUiDrawer::check_inputs(){

    ImGuiIO& io = ImGui::GetIO();
    if(ImGui::IsItemHovered()){

        const double xoffset = io.MouseDelta.x;
        const double yoffset = -io.MouseDelta.y;
        const double wheel   = io.MouseWheel;

        if(ImGui::IsMouseDown(0)){
            m_camera.set_direction(rotationSpeed*xoffset,rotationSpeed*yoffset,0.);
            m_cameraUpdated = true;
        }
        if(ImGui::IsMouseDown(1)){
            m_camera.set_direction(0.,0.,rotationSpeed*xoffset);
            m_cameraUpdated = true;
        }
        if(ImGui::IsMouseDown(2)){
            m_camera.move_up(translateSpeed*yoffset);
            m_camera.move_right(translateSpeed*xoffset);
            m_cameraUpdated = true;
        }
        if(io.MouseWheel != 0.f){
            m_camera.move_front(scrollSpeed*wheel);
            m_cameraUpdated = true;
        }

        // up key
        if(ImGui::IsKeyDown(73)){
            m_camera.move_front(movingSpeed);
            m_cameraUpdated = true;
        }
        // down key
        if(ImGui::IsKeyDown(74)){
            m_camera.move_back(movingSpeed);
            m_cameraUpdated = true;
        }
        // left key
        if(ImGui::IsKeyDown(71)){
            m_camera.move_left(movingSpeed);
            m_cameraUpdated = true;
        }
        // right key
        if(ImGui::IsKeyDown(72)){
            m_camera.move_right(movingSpeed);
            m_cameraUpdated = true;
        }
        // R key
        if(ImGui::IsKeyPressed(17, false)){            
            m_camera.reset_init_values();
            m_cameraUpdated = true;
        }
    }
}

