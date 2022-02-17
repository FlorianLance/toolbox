
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

// local
#include "imgui/imgui.h"

using namespace tool::graphics;


void ImguiFboDrawer::initialize_gl(const geo::Pt2<int> &size){
    fbo.clean();
    fbo.generate();
    resize_texture(size);
    testCube.init(1.f);
}

void ImguiFboDrawer::resize_texture(const geo::Pt2<int> &size){

    if(texture.width() == size.x() && texture.height() == size.y()){
        return;
    }

    m_screen.resize(size.x(), size.y());
    m_camera.update_projection();


    fbo.bind();

    texture.clean();
    texture.init_image_8ui(size.x(),size.y(), 3);

    TextureOptions options;
    options.minFilter = TextureMinFilter::linear;
    options.magFilter = TextureMagFilter::linear;
    texture.set_texture_options(options);

    depthTexture.clean();
    depthTexture.generate();
    depthTexture.bind();
    depthTexture.set_data_storage(size.x(), size.y());

    fbo.attach_colors_textures({
        &texture
    });

    fbo.attach_depth_buffer(depthTexture);

    fbo.set_draw_buffers({
        tool::gl::FrameBufferAttachment::color0,
    });

    fbo.unbind();
}

void ImguiFboDrawer::update_viewport(){
    glViewport(0,0, texture.width(), texture.height());
}


void ImguiFboDrawer::draw_texture(bool invert){

    ImVec2 vMin    = ImGui::GetWindowContentRegionMin();
    ImVec2 vMax    = ImGui::GetWindowContentRegionMax();
    ImVec2 sizeW   = ImVec2(vMax.x-vMin.x, vMax.y-vMin.y);
    float scale    = std::min(1.f*sizeW.y / texture.height(),  1.f*sizeW.x / texture.width());
    ImVec2 sizeI   = ImVec2(static_cast<int>(texture.width() * scale),static_cast<int>(texture.height() * scale));
//    sizeI.x -= 0;
    sizeI.y -= 50;

    auto uv1     = ImVec2(0,0);
    auto uv2     = ImVec2(1,1);
    auto uv3     = ImVec2(0,1);
    auto uv4     = ImVec2(1,0);

    if(texture.id() == 0){
        ImGui::Text("Texture not initialized.");
    }else{
        if(invert){
            ImGui::Image(texture.id(), sizeI,  uv3, uv4);
        }else{
            ImGui::Image(texture.id(), sizeI,  uv1, uv2);
        }
    }

    check_inputs();

}

void ImguiFboDrawer::update_texture_with_voxels(gl::ShaderProgram *shader, gl::CloudPointsDrawer *drawer, float halfVoxelSize){

    // get current viewport
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    bind();
    update_viewport();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    geo::Mat4<double> model(true);

    shader->use();
    shader->set_uniform("view", camera()->view().conv<float>());
    shader->set_uniform("model", (model).conv<float>());
    shader->set_uniform("projection", camera()->projection().conv<float>());
    shader->set_uniform("enable_unicolor", false);
    shader->set_uniform("hSize", halfVoxelSize);
    drawer->draw();

    // restore
    gl::FBO::unbind();
    //        glViewport(0, 0, static_cast<GLsizei>(m_camera->screen()->width()), static_cast<GLsizei>(m_camera->screen()->height()));
    glViewport(
        static_cast<GLsizei>(viewport[0]),
        static_cast<GLsizei>(viewport[1]),
        static_cast<GLsizei>(viewport[2]),
        static_cast<GLsizei>(viewport[3])
    );
}

void ImguiFboDrawer::update_texture_with_cloud(gl::ShaderProgram *shader, gl::CloudPointsDrawer *drawer, float sizePtsCloud){

    // get current viewport
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    bind();
    update_viewport();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    geo::Mat4<double> model(true);

    shader->use();
    shader->set_uniform("view", camera()->view().conv<float>());
    shader->set_uniform("model", (model).conv<float>());
    shader->set_uniform("projection", camera()->projection().conv<float>());
    shader->set_uniform("size_pt", sizePtsCloud);
    shader->set_uniform("camera_position", camera()->position().conv<float>());
    shader->set_uniform("enable_unicolor", false);
    drawer->draw();

    // restore
    gl::FBO::unbind();
//    glViewport(0, 0, static_cast<GLsizei>(m_camera->screen()->width()), static_cast<GLsizei>(m_camera->screen()->height()));
    glViewport(
        static_cast<GLsizei>(viewport[0]),
        static_cast<GLsizei>(viewport[1]),
        static_cast<GLsizei>(viewport[2]),
        static_cast<GLsizei>(viewport[3])
    );
}

void ImguiFboDrawer::test_voxels(gl::ShaderProgram *shader, gl::ShaderProgram *solid, gl::CloudPointsDrawer *drawer, float halfVoxelSize){

    // get current viewport
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    bind();
    update_viewport();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    geo::Mat4<double> model(true);

    shader->use();
    shader->set_uniform("view", camera()->view().conv<float>());
    shader->set_uniform("model", (model).conv<float>());
    shader->set_uniform("projection", camera()->projection().conv<float>());
    shader->set_uniform("enable_unicolor", false);
    shader->set_uniform("hSize", halfVoxelSize);
    drawer->draw();

    solid->use();
    solid->set_uniform("view", camera()->view().conv<float>());
    solid->set_uniform("model", (model).conv<float>());
    solid->set_uniform("projection", camera()->projection().conv<float>());
    testCube.draw();

    // restore
    gl::FBO::unbind();
    //        glViewport(0, 0, static_cast<GLsizei>(m_camera->screen()->width()), static_cast<GLsizei>(m_camera->screen()->height()));
    glViewport(
        static_cast<GLsizei>(viewport[0]),
        static_cast<GLsizei>(viewport[1]),
        static_cast<GLsizei>(viewport[2]),
        static_cast<GLsizei>(viewport[3])
    );
}

void ImguiFboDrawer::test_cloud(gl::ShaderProgram *shader, gl::ShaderProgram *solid, gl::CloudPointsDrawer *drawer, float sizePtsCloud){

    // get current viewport
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    bind();
    update_viewport();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    geo::Mat4<double> model(true);

    shader->use();
    shader->set_uniform("view", camera()->view().conv<float>());
    shader->set_uniform("model", (model).conv<float>());
    shader->set_uniform("projection", camera()->projection().conv<float>());
    shader->set_uniform("size_pt", sizePtsCloud);
    shader->set_uniform("camera_position", camera()->position().conv<float>());
    shader->set_uniform("enable_unicolor", false);
    drawer->draw();

    solid->use();
    solid->set_uniform("view", camera()->view().conv<float>());
    solid->set_uniform("model", (model).conv<float>());
    solid->set_uniform("projection", camera()->projection().conv<float>());
    testCube.draw();

    // restore
    gl::FBO::unbind();
    //    glViewport(0, 0, static_cast<GLsizei>(m_camera->screen()->width()), static_cast<GLsizei>(m_camera->screen()->height()));
    glViewport(
        static_cast<GLsizei>(viewport[0]),
        static_cast<GLsizei>(viewport[1]),
        static_cast<GLsizei>(viewport[2]),
        static_cast<GLsizei>(viewport[3])
    );
}

void ImguiFboDrawer::test_boths(gl::ShaderProgram *shader1, gl::ShaderProgram *shader2, gl::CloudPointsDrawer *drawer1, gl::CloudPointsDrawer *drawer2, float sizePtsCloud, float halfVoxelSize){

    // get current viewport
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    bind();
    update_viewport();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    geo::Mat4<double> model(true);

    shader1->use();
    shader1->set_uniform("view", camera()->view().conv<float>());
    shader1->set_uniform("model", (model).conv<float>());
    shader1->set_uniform("projection", camera()->projection().conv<float>());
    shader1->set_uniform("size_pt", sizePtsCloud);
    shader1->set_uniform("camera_position", camera()->position().conv<float>());
    shader1->set_uniform("enable_unicolor", true);
    shader1->set_uniform("unicolor", geo::Pt4f{0.f,1,0,1.f});
    drawer1->draw();

    shader2->use();
    shader2->set_uniform("view", camera()->view().conv<float>());
    shader2->set_uniform("model", (model).conv<float>());
    shader2->set_uniform("projection", camera()->projection().conv<float>());
    shader2->set_uniform("enable_unicolor", true);
    shader2->set_uniform("hSize", halfVoxelSize);
    shader2->set_uniform("unicolor", geo::Pt4f{1.f,0,0,1.f});

    drawer2->draw();

    // restore
    gl::FBO::unbind();
    //    glViewport(0, 0, static_cast<GLsizei>(m_camera->screen()->width()), static_cast<GLsizei>(m_camera->screen()->height()));
    glViewport(
        static_cast<GLsizei>(viewport[0]),
        static_cast<GLsizei>(viewport[1]),
        static_cast<GLsizei>(viewport[2]),
        static_cast<GLsizei>(viewport[3])
    );
}

void ImguiFboDrawer::check_inputs(){

    ImGuiIO& io = ImGui::GetIO();
    if(ImGui::IsItemHovered()){

        const double xoffset = io.MouseDelta.x;
        const double yoffset = -io.MouseDelta.y;
        const double wheel   = io.MouseWheel;

        if(ImGui::IsMouseDown(0)){
            m_camera.set_direction(rotationSpeed*xoffset,rotationSpeed*yoffset,0.);
        }
        if(ImGui::IsMouseDown(1)){
            m_camera.set_direction(0.,0.,rotationSpeed*xoffset);
        }
        if(ImGui::IsMouseDown(2)){
            m_camera.move_up(translateSpeed*yoffset);
            m_camera.move_right(translateSpeed*xoffset);
        }
        if(io.MouseWheel != 0.f){
            m_camera.move_front(scrollSpeed*wheel);
        }

        // up key
        if(ImGui::IsKeyDown(73)){
            m_camera.move_front(movingSpeed);
        }
        // down key
        if(ImGui::IsKeyDown(74)){
            m_camera.move_back(movingSpeed);
        }
        // left key
        if(ImGui::IsKeyDown(71)){
            m_camera.move_left(movingSpeed);
        }
        // right key
        if(ImGui::IsKeyDown(72)){
            m_camera.move_right(movingSpeed);
        }
        // R key
        if(ImGui::IsKeyPressed(17, false)){
            m_camera.reset_init_values();
        }
    }
}
