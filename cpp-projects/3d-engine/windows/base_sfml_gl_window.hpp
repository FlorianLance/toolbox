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


// std
#include <chrono>
#include <optional>

// glew
#include <GL/glew.h>

// sfml
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include "imgui-sfml/imgui-SFML.h"

// base
#include "graphics/camera.hpp"
#include "graphics/screen.hpp"


namespace tool::graphics {

class BaseSfmlGlWindow{

public:
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<long long, std::ratio<1,1000000000>>>;

    BaseSfmlGlWindow(
            std::string title,
            geo::Pt2<unsigned int> size,
            std::optional<sf::ContextSettings> context//,
            //std::optional<sf::Style> style
    );
    ~BaseSfmlGlWindow();

    bool init();
    void start();        

    float elapsed_secondes() const{
        return std::chrono::duration_cast<std::chrono::milliseconds>(currentFrame-startL).count()*0.001f;
    }

protected:

    // init
    virtual bool initialize_gl() = 0;

    // clean
    virtual void clean(){}

    // resize
    virtual void resize_windows(){}

    // update
    virtual void pre_update(){}
    virtual void update_gl(){}
    virtual void update_sfml(){}
    virtual void update_imgui(){}
    virtual void post_update(){}

    // events
    virtual void mouse_button_event(sf::Event mouseButtonEvent);
    virtual void mouse_moved_event(sf::Event mouseMovedEvent);
    virtual void keyboard_keyrelease_event(sf::Event keyReleaseEvent){static_cast<void>(keyReleaseEvent);}
    virtual void mouse_scroll_event(sf::Event mouseScrollEvent);
    virtual void keyboard_keypress_event(sf::Event keyPressEvent);

    // camera
    virtual void update_camera_with_mouse_button_event(sf::Event mouseButtonEvent);
    virtual void update_camera_with_keyboardpress_event(sf::Event keyPressEvent);
    virtual void update_camera_with_mouse_scroll_event(sf::Event mouseScrollEvent);
    virtual void update_camera_with_mouse_moved_event(sf::Event mouseMovedEvent);

private:

    bool init_sfml_window();
    void base_update_imgui();
    void base_resize_windows(sf::Event::SizeEvent size);

protected:
    // opengl
    bool m_glInitialized = false;
    sf::RenderWindow m_scene;
    sf::ContextSettings glContext;

    // window
    std::string m_title = "Base SFML GL window";
    graphics::Screen m_screen;

    // camera
    double m_cameraSpeed = 0.2;
    graphics::Camera m_camera;

    // time
    TimePoint startL;
    TimePoint currentFrame;
    int framerate = 60;
    sf::Clock deltaClock;

    std::chrono::duration<double, std::milli> frameDuration;
    std::chrono::duration<double, std::milli> timePerFrame{1000./framerate};

    // inputs
    // # mouse
    bool mouseLeftClickPressed = false;
    bool mouseRightClickPressed = false;
    bool mouseMiddleClickPressed = false;
    int lastX=-1, lastY=-1;
    bool imguiHover = false;
};



}

