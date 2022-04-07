
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
#include "utility/logger.hpp"

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

    resize_windows();

    return true;
}

void BaseSfmlGlWindow::start(){

    if(!m_glInitialized){
        return;
    }

    m_scene.setActive();

    startL = std::chrono::high_resolution_clock::now();

    running = true;
    while(running){

        const auto &io = ImGui::GetIO();
        imguiMouse    = io.WantCaptureMouse;
        imguiKeyboard = io.WantCaptureKeyboard;

        Bench::start("main_loop");

        currentFrame = std::chrono::high_resolution_clock::now();

        // retrieve sfml events
        sf::Event event;

        while (m_scene.pollEvent(event)){

            ImGui::SFML::ProcessEvent(event);
            switch (event.type) {
            case sf::Event::Closed: // The window requested to be closed
                running = false;
                break;
            case sf::Event::Resized: // The window was resized
                Logger::message("resize_event-");
                base_resize_windows(event.size);
                break;
            case sf::Event::KeyPressed: // A key was pressed
                keyboard_keypress_event(event.key);
                break;
            case sf::Event::KeyReleased: // A key was released
                keyboard_keyrelease_event(event.key);
                break;
            case sf::Event::MouseButtonPressed: // A mouse button was pressed
                mouse_button_pressed_event(event.mouseButton);
                break;
            case sf::Event::MouseButtonReleased: // A mouse button was released
                mouse_button_released_event(event.mouseButton);
                break;
            case sf::Event::MouseMoved: // The mouse cursor moved
                mouse_moved_event(event.mouseMove);
                break;
            case sf::Event::MouseWheelScrolled: // The mouse wheel was scrolled
                mouse_wheel_scroll_event(event.mouseWheelScroll);
                break;
            case sf::Event::MouseEntered: // The mouse cursor entered the area of the window
                break;
            case sf::Event::MouseLeft: // The mouse cursor left the area of the window
                break;
            case sf::Event::JoystickButtonPressed: // A joystick button was pressed
                break; // event.joystickButton
            case sf::Event::JoystickButtonReleased: // A joystick button was released
                break; // event.joystickButton
            case sf::Event::JoystickMoved: // The joystick moved along an axis
                break; // event.joystickMove
            case sf::Event::JoystickConnected: // A joystick was connected
                break; // event.joystickConnect
            case sf::Event::JoystickDisconnected: // A joystick was disconnected
                break; // event.joystickConnect
            default:
                break;
            }
            // LostFocus,     < The window lost the focus (no data)
            // GainedFocus,   < The window gained the focus (no data)
            // TextEntered,   < A character was entered (data in event.text)
            // TouchBegan,    < A touch event began (data in event.touch)
            // TouchMoved,    < A touch moved (data in event.touch)
            // TouchEnded,    < A touch event ended (data in event.touch)
            // SensorChanged, < A sensor value changed (data in event.sensor)
        }

        if(imguiMouse){
            mouseLeftClickPressed   = false;
            mouseRightClickPressed  = false;
            mouseMiddleClickPressed = false;
        }
        if(imguiKeyboard){

        }

        // update
        pre_update();
        update();
        post_update();

        m_scene.clear(sf::Color::White);

        // draw opengl
        draw_gl();

        // ubind vao after drawing opengl
        VAO::unbind();

        // store gl states
        m_scene.pushGLStates();
        {
            // update sfml
            ImGui::SFML::Update(m_scene, deltaClock.restart());

            // imgui
            imguiMouse = false;
            draw_imgui();
            ImGui::EndFrame();

            // render sfml scene
            draw_sfml();
            ImGui::SFML::Render(m_scene);
            m_scene.display();
        }
        // restore gl states
        m_scene.popGLStates();


        // sleep for fps
        frameDuration = std::chrono::high_resolution_clock::now()-currentFrame;
        if((timePerFrame-frameDuration).count() > 0){
            std::this_thread::sleep_for(timePerFrame-frameDuration);
        }
        Bench::stop();
        // Bench::display();
    }

    clean();

    ImGui::SFML::Shutdown();
}

//void BaseSfmlGlWindow::check_hovering_imgui(){
//    const auto &io = ImGui::GetIO();
//    imguiHover = io.WantCaptureMouse;
//}


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

void BaseSfmlGlWindow::base_resize_windows(sf::Event::SizeEvent size){
    m_screen = graphics::Screen{size.width,size.height};
    m_camera.update_projection();
    glViewport(0, 0, static_cast<GLsizei>(m_screen.width()), static_cast<GLsizei>(m_screen.height()));
    resize_windows();
}

void BaseSfmlGlWindow::mouse_button_pressed_event(sf::Event::MouseButtonEvent event){
    if(!imguiMouse){
        update_camera_with_mouse_button_event(event, true);
    }
}

void BaseSfmlGlWindow::mouse_button_released_event(sf::Event::MouseButtonEvent event){
    if(!imguiMouse){
        update_camera_with_mouse_button_event(event, false);
    }
}


void BaseSfmlGlWindow::mouse_moved_event(sf::Event::MouseMoveEvent event){
    if(!imguiMouse){
        update_camera_with_mouse_moved_event(event);
    }
}

void BaseSfmlGlWindow::mouse_wheel_scroll_event(sf::Event::MouseWheelScrollEvent event){
    if(!imguiMouse){
        if(!mouseMiddleClickPressed){
            update_camera_with_mouse_scroll_event(event);
        }
    }
}

void BaseSfmlGlWindow::keyboard_keypress_event(sf::Event::KeyEvent event){

    if(!imguiMouse && !imguiKeyboard){
        update_camera_with_keyboardpress_event(event);
    }
}

//void BaseSfmlGlWindow::check_imgui_inputs(){

////    if (!ImGui::IsItemHovered()){
////        return;
////    }

//    ImGuiIO& io = ImGui::GetIO();
//    if(io.WantCaptureMouse){

//        auto mPos = geo::Pt2f{io.MousePos.x, io.MousePos.y};
//        auto preMPos = geo::Pt2f{io.MousePosPrev.x, io.MousePosPrev.y};
//        if(square_norm(mPos - preMPos) != 0){
//            Logger::message(std::format("%f %f\n", io.MouseDelta.x, io.MouseDelta.y));
//        }

////        if(io.MousePos - io.MousePosPrev){

////        }


////        ImGui::Text("Mouse delta: (%g, %g)", io.MouseDelta.x, io.MouseDelta.y);
////        ImGui::Text("Mouse down:");     for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (io.MouseDownDuration[i] >= 0.0f)   { ImGui::SameLine(); ImGui::Text("b%d (%.02f secs)", i, io.MouseDownDuration[i]); }
////        ImGui::Text("Mouse clicked:");  for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (ImGui::IsMouseClicked(i))          { ImGui::SameLine(); ImGui::Text("b%d", i); }
////        ImGui::Text("Mouse dblclick:"); for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (ImGui::IsMouseDoubleClicked(i))    { ImGui::SameLine(); ImGui::Text("b%d", i); }
////        ImGui::Text("Mouse released:"); for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++) if (ImGui::IsMouseReleased(i))         { ImGui::SameLine(); ImGui::Text("b%d", i); }
////        ImGui::Text("Mouse wheel: %.1f", io.MouseWheel);
//    }
//}

void BaseSfmlGlWindow::update_camera_with_mouse_button_event(sf::Event::MouseButtonEvent event, bool pressed){


    if(pressed){
        switch (event.button) {
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
    }else{

        switch (event.button) {
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
    }
}

void BaseSfmlGlWindow::update_camera_with_keyboardpress_event(sf::Event::KeyEvent event){

    switch (event.code) {
    case sf::Keyboard::Key::Up:
        m_camera.move_front(cameraMovingSpeed);
        break;
    case sf::Keyboard::Key::Left:
        m_camera.move_left(cameraMovingSpeed);
        break;
    case sf::Keyboard::Key::Right:
        m_camera.move_right(cameraMovingSpeed);
        break;
    case sf::Keyboard::Key::Down:
        m_camera.move_back(cameraMovingSpeed);
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

void BaseSfmlGlWindow::update_camera_with_mouse_scroll_event(sf::Event::MouseWheelScrollEvent event){

    if(event.wheel == 0){
        m_camera.move_front(static_cast<double>(event.delta) * cameraScrollSpeed);
    }
}

void BaseSfmlGlWindow::update_camera_with_mouse_moved_event(sf::Event::MouseMoveEvent event){

    if(lastX < 0.){
        lastX = event.x;
        lastY = event.y;
    }

    double xoffset = event.x - lastX;
    double yoffset = event.y - lastY;
    lastX = event.x;
    lastY = event.y;

    double sensitivity = cameraRotationSpeed;
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

void BaseSfmlGlWindow::clean(){
}
