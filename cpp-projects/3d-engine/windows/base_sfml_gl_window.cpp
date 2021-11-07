
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

#include "base_sfml_gl_window.hpp"

// std
#include <string>
#include <thread>

// imgui
#include "imgui/imgui.h"

// base
#include "utility/benchmark.hpp"

// opengl-utility
#include "opengl/utility/glew_utility.hpp"
#include "opengl/vao.hpp"

using namespace tool::gl;
using namespace tool::geo;
using namespace tool::graphics;

BaseSfmlGlWindow::BaseSfmlGlWindow(std::string title, Pt2<unsigned int> size,std::optional<sf::ContextSettings> context) ://, std::optional<sf::Style> style) :
      m_title(title), m_screen(size.x(),size.y()), m_camera(&m_screen) {

    if(context.has_value()){
        glContext = context.value();
    }else{
        glContext.depthBits         = 24;
        glContext.stencilBits       = 8;
        glContext.antialiasingLevel = 4;
        glContext.majorVersion      = 4;
        glContext.minorVersion      = 6;
        glContext.attributeFlags    = sf::ContextSettings::Attribute::Debug;
    }
}

BaseSfmlGlWindow::~BaseSfmlGlWindow(){
    clean();
    m_scene.close();
}

bool BaseSfmlGlWindow::init(){

    // sfml
    init_sfml_window();

    // glew
    if(!init_glew()){
        m_scene.close();
        return false;
    }
    display_glew_info();

    // init imgui
    m_scene.setActive(true);

    initialize_gl();
    ImGui::SFML::Init(m_scene);


    return true;
}

void BaseSfmlGlWindow::start(){

    if(!m_glInitialized){
        return;
    }

    m_scene.setActive();

    startL = std::chrono::high_resolution_clock::now();

    bool running = true;
    while(running){

        Bench::start("main_loop");

        currentFrame = std::chrono::high_resolution_clock::now();

        sf::Event event;
        while (m_scene.pollEvent(event)){

            ImGui::SFML::ProcessEvent(event);

            switch (event.type) {
            case sf::Event::Closed:
                running = false;
                break;
            case sf::Event::Resized:
                base_resize_windows(event.size);
                break;
            case sf::Event::KeyPressed:
                keyboard_keypress_event(event);
                break;
            case sf::Event::KeyReleased:
                keyboard_keyrelease_event(event);
                break;
            case sf::Event::MouseButtonPressed:
                mouse_button_event(event);
                break;
            case sf::Event::MouseButtonReleased:
                mouse_button_event(event);
                break;
            case sf::Event::MouseMoved:
                mouse_moved_event(event);
                break;
            case sf::Event::MouseWheelScrolled:
                mouse_scroll_event(event);
                break;
            default:
                break;
            }
        }

        pre_update();

        // clean buffers
        m_scene.clear(sf::Color::White);

        // set gl states for drawing sfml/imgui
        m_scene.pushGLStates();
        update_sfml();
        m_scene.popGLStates();

        // draw opengl
        update_gl();

        // set gl states for drawing sfml/imgui
        VAO::unbind();
        m_scene.pushGLStates();

        // draw gui
        base_update_imgui();
        m_scene.popGLStates();

        // display scene
        m_scene.display();

        post_update();

        // sleep for fps
        frameDuration = std::chrono::high_resolution_clock::now()-currentFrame;
        if((timePerFrame-frameDuration).count() > 0){
            std::this_thread::sleep_for(timePerFrame-frameDuration);
        }

        Bench::stop();
        Bench::display();
    }

    ImGui::SFML::Shutdown();
}

bool BaseSfmlGlWindow::init_sfml_window(){

    // close previously opened window
    if(m_scene.isOpen()){
        m_scene.close();
    }

    // create window
    m_scene.create(sf::VideoMode(m_screen.width(), m_screen.height()), m_title, sf::Style::Default, glContext);
    m_scene.setFramerateLimit(framerate);

    return true;
}

void BaseSfmlGlWindow::base_update_imgui(){

    ImGui::SFML::Update(m_scene, deltaClock.restart());

    update_imgui();

    imguiHover = ImGui::IsAnyItemHovered() ||
            ImGui::IsWindowHovered()  ||
            ImGui::IsAnyItemFocused() ||
            ImGui::IsAnyItemActive();

    ImGui::SFML::Render(m_scene);
}

void BaseSfmlGlWindow::base_resize_windows(sf::Event::SizeEvent size){
    m_screen = graphics::Screen{size.width,size.height};
    m_camera.update_projection();
    glViewport(0, 0, static_cast<GLsizei>(m_screen.width()), static_cast<GLsizei>(m_screen.height()));
    resize_windows();
}

void BaseSfmlGlWindow::mouse_button_event(sf::Event mouseButtonEvent){
    if(!imguiHover){
        update_camera_with_mouse_button_event(mouseButtonEvent);
    }
}

void BaseSfmlGlWindow::mouse_moved_event(sf::Event mouseMovedEvent){
    if(!imguiHover){
        update_camera_with_mouse_moved_event(mouseMovedEvent);
    }
}

void BaseSfmlGlWindow::mouse_scroll_event(sf::Event mouseScrollEvent){
    if(!imguiHover){
        update_camera_with_mouse_scroll_event(mouseScrollEvent);
    }
}

void BaseSfmlGlWindow::keyboard_keypress_event(sf::Event keyPressEvent){
    if(!imguiHover){
        update_camera_with_keyboardpress_event(keyPressEvent);
    }
}

void BaseSfmlGlWindow::update_camera_with_mouse_button_event(sf::Event mouseButtonEvent){

    switch (mouseButtonEvent.type) {
    case sf::Event::MouseButtonPressed:
        switch (mouseButtonEvent.mouseButton.button) {
        case sf::Mouse::Button::Left:
            mouseLeftClickPressed = true;
            break;
        case sf::Mouse::Button::Right:
            mouseRightClickPressed = true;
            break;
        case sf::Mouse::Button::Middle:
            mouseMiddleClickPressed = true;
            break;
        default:
            break;
        }
        break;
    case sf::Event::MouseButtonReleased:
        switch (mouseButtonEvent.mouseButton.button) {
        case sf::Mouse::Button::Left:
            mouseLeftClickPressed = false;
            break;
        case sf::Mouse::Button::Right:
            mouseRightClickPressed = false;
            break;
        case sf::Mouse::Button::Middle:
            mouseMiddleClickPressed = false;
            break;
        default:
            break;
        }

        lastX = -1;
        lastY = -1;
        break;
    default:
        break;
    }
}

void BaseSfmlGlWindow::update_camera_with_keyboardpress_event(sf::Event keyPressEvent){

    switch (keyPressEvent.key.code) {
    case sf::Keyboard::Key::Up:
        m_camera.move_front(m_cameraSpeed);
        break;
    case sf::Keyboard::Key::Left:
        m_camera.move_left(m_cameraSpeed);
        break;
    case sf::Keyboard::Key::Right:
        m_camera.move_right(m_cameraSpeed);
        break;
    case sf::Keyboard::Key::Down:
        m_camera.move_back(m_cameraSpeed);
        break;
    case sf::Keyboard::Key::R:
        m_camera.reset_init_values();
        m_camera.set_direction(0.,0.,0.);
        break;
    case sf::Keyboard::Key::O:
        m_camera.set_mode(Camera::Mode::Orhtographic);
        break;
    case sf::Keyboard::Key::P:
        m_camera.set_mode(Camera::Mode::Perspective);
        break;
    default:
        break;
    }
}

void BaseSfmlGlWindow::update_camera_with_mouse_scroll_event(sf::Event mouseScrollEvent){

    m_camera.move_front(static_cast<double>(mouseScrollEvent.mouseWheelScroll.delta) *0.05);
}

void BaseSfmlGlWindow::update_camera_with_mouse_moved_event(sf::Event mouseMovedEvent){

    if(lastX < 0.){
        lastX = mouseMovedEvent.mouseMove.x;
        lastY = mouseMovedEvent.mouseMove.y;
    }

    double xoffset = mouseMovedEvent.mouseMove.x - lastX;
    double yoffset = mouseMovedEvent.mouseMove.y - lastY;
    lastX = mouseMovedEvent.mouseMove.x;
    lastY = mouseMovedEvent.mouseMove.y;

    double sensitivity = 0.05;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    if(mouseLeftClickPressed){
        m_camera.set_direction(xoffset,-yoffset,0.);
    }else if(mouseMiddleClickPressed){
        m_camera.move_up(-0.1*yoffset);
        m_camera.move_right(0.1*xoffset);
    }else if(mouseRightClickPressed){
        m_camera.set_direction(0.,0.,xoffset);
    }
}
