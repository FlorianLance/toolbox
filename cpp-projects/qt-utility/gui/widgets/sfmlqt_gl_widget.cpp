
/*******************************************************************************
** Toolbox-qt-utility                                                         **
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

#include "sfmlqt_gl_widget.hpp"

#ifdef Q_WS_X11
    #include <Qt/qx11info_x11.h>
    #include <X11/Xlib.h>
#endif

// std
#include <iostream>

// Qt
#include <QDebug>
#include <QKeyEvent>
#include <QtMath>
#include <QApplication>
#include <QThread>

// base
#include "geometry/point3.hpp"
#include "geometry/matrix4.hpp"
#include "utility/vector.hpp"

using namespace tool::ui;
using namespace tool::geo;


bool tool::gl::ColoredCloudShader::init(){

    // init shader
    const char* vertexShaderSource =
        R"shaderDef(
            #version 430 core

            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec3 aColor;

            // model
            uniform mat4 model;
            uniform mat4 view;
            uniform mat4 projection;

            // camera
            uniform vec3 camera_position;

            // cloud
            uniform float size_pt = 3.f;

            // color
            uniform vec4 unicolor = vec4(1,0,0,1);
            uniform bool enable_unicolor = false;
            out vec4 color;

            void main(){
                vec4 p = view * model * vec4(aPos, 1.0);
                gl_Position = projection*p;
                color = enable_unicolor ? unicolor : vec4(aColor, 1.0);
                float l = sqrt(length(p.xyz-camera_position.xyz));
                gl_PointSize = size_pt/(l);
            }
        )shaderDef";

    const char* fragmentShaderSource =
        R"shaderDef(
            #version 430 core

            out vec4 FragColor;
            in vec4 color;

            void main(){
                FragColor = color;
            }

        )shaderDef";

    return load_from_source_code({{Shader::Type::vertex, vertexShaderSource}, {Shader::Type::fragment,fragmentShaderSource}});
}

bool tool::gl::ColoredMeshShader::init(){

    // init shader
    const char* vertexShaderSource =
        R"shaderDef(
            #version 430 core

            // layout
            layout (location = 0) in vec3 aPos;
            layout (location = 6) in vec4 aColor;

            // uniform
            uniform mat4 model;
            uniform mat4 view;
            uniform mat4 projection;
            uniform vec4 unicolor = vec4(1,1,0,1);
            uniform bool enable_unicolor = false;

            // out
            out vec4 color;
            void main(){
                gl_Position = projection*view * model * vec4(aPos, 1.0);
                color = enable_unicolor ? unicolor : aColor;
            }

        )shaderDef";

    const char* fragmentShaderSource =
        R"shaderDef(
            #version 430 core

            out vec4 FragColor;
            in vec4 color;

            void main(){
                FragColor = color;
            }

        )shaderDef";

    return load_from_source_code({{Shader::Type::vertex, vertexShaderSource}, {Shader::Type::fragment,fragmentShaderSource}});
}


SfmlQtGlWidget::SfmlQtGlWidget(QWidget* Parent) : QWidget(Parent), model(geo::Mat4d(true)), myInitialized(false), camera(&m_screen){

    // rendering in widget
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_NoSystemBackground);

    // allow keyboard inputs
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
//    QApplication::setKeyboardInputInterval(0);

    this->installEventFilter(this);
}

SfmlQtGlWidget::~SfmlQtGlWidget(){
    renderTimer.stop();
}

void SfmlQtGlWidget::set_model(Mat4<double> m){
    model = m;
}

void SfmlQtGlWidget::showEvent(QShowEvent*){

    if (myInitialized){
        return;
    }

    // set context
    sf::ContextSettings settings;
    settings.majorVersion = 4;
    settings.minorVersion = 6;
    settings.depthBits = 24;
    settings.stencilBits = 8;
    settings.antialiasingLevel = 4;

    // init window
    sf::Window::create(reinterpret_cast<sf::WindowHandle>(winId()),settings);

    // init glew
    glewInit();

    // init gl
    OnInit();

    // connections
    connect(&renderTimer, &QTimer::timeout, this, [&]{
       repaint();
    });

    //  timer
    renderTimer.start(1000/60);
    myInitialized = true;

    // init camera
    camera.set_direction(0.,0.,0.);
}

QPaintEngine* SfmlQtGlWidget::paintEngine() const{
    return nullptr;
}

void SfmlQtGlWidget::paintEvent(QPaintEvent*){
    OnUpdate();
    display();
}

void SfmlQtGlWidget::resizeEvent(QResizeEvent *event){
    Q_UNUSED(event)
    m_screen.resize(static_cast<unsigned int>(width()),static_cast<unsigned int>(height()));
    camera.update_projection();
    onResize();
}

void SfmlQtGlWidget::mousePressEvent(QMouseEvent *event){
    mouseLeftClickPressed   = event->button() == Qt::MouseButton::LeftButton;
    mouseMiddleClickPressed = event->button() == Qt::MouseButton::MiddleButton;
    mouseRighClickPressed   = event->button() == Qt::MouseButton::RightButton;
}
void SfmlQtGlWidget::mouseReleaseEvent(QMouseEvent *event){

    if(event->button() == Qt::MouseButton::LeftButton){
        mouseLeftClickPressed = false;
    }else if(event->button() == Qt::MouseButton::MiddleButton){
        mouseMiddleClickPressed = false;
    }else if(event->button() == Qt::MouseButton::RightButton){
        mouseRighClickPressed = false;
    }

    lastX = -1.;
    lastY = -1.;
}


void SfmlQtGlWidget::mouseMoveEvent(QMouseEvent *event){

    if(lastX < 0.){
        lastX = event->pos().x();
        lastY = event->pos().y();
    }

    double xoffset = event->pos().x() - lastX;
    double yoffset = event->pos().y() - lastY;
    lastX = event->pos().x();
    lastY = event->pos().y();

    double sensitivity = 0.05;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    if(mouseLeftClickPressed){
        camera.set_direction(xoffset,-yoffset,0.);
    }else if(mouseMiddleClickPressed){
        camera.move_up(-0.1*yoffset);
        camera.move_right(0.1*xoffset);
    }else if(mouseRighClickPressed){
        camera.set_direction(0.,0.,xoffset);
    }
    update();
}

void SfmlQtGlWidget::wheelEvent(QWheelEvent *event){
    camera.move_front(event->angleDelta().y()*0.001);   
    update();
}


void SfmlQtGlWidget::keyPressEvent(QKeyEvent *event){

    isKeyPressed = true;

    if(event->key() == Qt::Key_Up){
        camera.move_front(cameraSpeed);
    }
    if(event->key() == Qt::Key_Left){
        camera.move_left(cameraSpeed);
    }
    if(event->key() == Qt::Key_Right){
        camera.move_right(cameraSpeed);
    }
    if(event->key() == Qt::Key_Down){
        camera.move_back(cameraSpeed);
    }
    if(event->key() == Qt::Key_R){
        camera.reset_init_values();
        camera.set_direction(0.,0.,0.);
    }

    update();
}

void SfmlQtGlWidget::keyReleaseEvent(QKeyEvent *event){
    Q_UNUSED(event)
    isKeyPressed = false;
}

void SfmlQtGlWidget::init_axes(){
    axis.init();
}

void SfmlQtGlWidget::init_grid(){
    grid.init();
}

void SfmlQtGlWidget::draw_axes(){

    meshShader.use();
    meshShader.set_uniform("view",         camera.view().conv<float>());
    meshShader.set_uniform("model",        model.conv<float>());
    meshShader.set_uniform("projection",   camera.projection().conv<float>());
    meshShader.set_uniform("enable_unicolor", false);
    axis.draw();
}

void SfmlQtGlWidget::draw_grid(){

    meshShader.use();
    meshShader.set_uniform("view",         camera.view().conv<float>());
    meshShader.set_uniform("model",        model.conv<float>());
    meshShader.set_uniform("projection",   camera.projection().conv<float>());
    meshShader.set_uniform("enable_unicolor", true);
    meshShader.set_uniform("unicolor", geo::Pt4f{0,1,1,1});
    grid.draw();
}


bool SfmlQtGlWidget::eventFilter(QObject *object, QEvent *event){
    Q_UNUSED(event)
    if(object != this){
        return false;
    }
//    //        if(event->type() == QEvent::)
//    if(isKeyPressed){
//        //            keyPressEvent(static_cast<QKeyEvent *>(event));
//    }

//    if (event->type() == QEvent::KeyPress) {
//        //            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
//        //            if (keyEvent->key() == Qt::Key_W) {
//        //                return true;
//        //            } else {
//        //                return false;
//        //            }
//    }
    return false;
}
