
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
#pragma once

// std
#include <array>

// sfml
#include <SFML/Graphics.hpp>

// Qt
#include <QTimer>
#include <QWidget>
#include <QMatrix4x4>
#include <QKeyEvent>

// base
#include "geometry/matrix4.hpp"
#include "utility/vector.hpp"
#include "graphics/camera.hpp"

// opengl-utility
#include "opengl/vao.hpp"
#include "opengl/buffer/vertex_buffer_object.hpp"
#include "opengl/buffer/element_buffer_object.hpp"
#include "opengl/shader/shader.hpp"
#include "opengl/gl_draw.hpp"

// local
#include "wrapper/qt_gl_wrapper.hpp"


namespace tool::gl{

class ColoredCloudShader : public ShaderProgram{
public :
    bool init();
};
class ColoredMeshShader : public ShaderProgram{
public :
    bool init();
};


class DrawColoredMesh{
public:

    void generate(){

        // generate
        if(!generated){
            vao.generate();
            pos.generate();
            col.generate();
            ebo.generate();
            generated = true;
        }
    }

    void init(geo::Pt3f *points, geo::Pt3f *colors, geo::Pt3<std::uint32_t> *triangles, size_t sizePts, size_t sizeTri){

        if(!generated){
            return;
        }

        count = VerticesCount{GLsizei(sizeTri*3)};

        // update buffers
        vao.bind();

        pos.load_data(FloatData{reinterpret_cast<float*>(points)}, SizeData{GLsizeiptr(sizePts * 3 * sizeof(float))});
        pos.attrib(AttriIndex{0}, AttriSize{3}, AttriType{GL_FLOAT}, Stride{3 * sizeof(float)});
        col.load_data(FloatData{reinterpret_cast<float*>(colors)}, SizeData{GLsizeiptr(sizePts * 3 * sizeof(float))});
        col.attrib(AttriIndex{1}, AttriSize{3}, AttriType{GL_FLOAT}, Stride{3 * sizeof(float)});

        ebo.bind();
        ebo.load_data(UintData{reinterpret_cast<std::uint32_t*>(triangles)}, SizeData{GLsizeiptr(sizeTri*3*sizeof (std::uint32_t))});

        initialized = true;
    }

    void draw(){

        if(!initialized || count.v == 0){
            return;
        }

        vao.bind();
        draw_triangles_with_ebo(count);
    }

    VAO vao;
    VBO pos;
    VBO col;
    EBO ebo;

    VerticesCount count;
    bool generated = false;
    bool initialized = false;
};


class DrawJointMesh{

public:
    void init(){

        const float vertices[] = {
            -0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f,  0.5f, -0.5f,
             0.5f,  0.5f, -0.5f,
            -0.5f,  0.5f, -0.5f,
            -0.5f, -0.5f, -0.5f,

            -0.5f, -0.5f,  0.5f,
             0.5f, -0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f,  0.5f,
            -0.5f, -0.5f,  0.5f,

            -0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f, -0.5f,
            -0.5f, -0.5f, -0.5f,
            -0.5f, -0.5f, -0.5f,
            -0.5f, -0.5f,  0.5f,
            -0.5f,  0.5f,  0.5f,

             0.5f,  0.5f,  0.5f,
             0.5f,  0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, -0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,

            -0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, -0.5f,  0.5f,
             0.5f, -0.5f,  0.5f,
            -0.5f, -0.5f,  0.5f,
            -0.5f, -0.5f, -0.5f,

            -0.5f,  0.5f, -0.5f,
             0.5f,  0.5f, -0.5f,
             0.5f,  0.5f,  0.5f,
             0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f,  0.5f,
            -0.5f,  0.5f, -0.5f,
        };

        vao.generate();
        vbo.generate();

        // update buffers
        vao.bind();
//        vbo.bind();
        vbo.load_data(FloatData{vertices}, SizeData{sizeof (vertices)});
        vbo.attrib(AttriIndex{0}, AttriSize{3}, AttriType{GL_FLOAT}, Stride{3 * sizeof(float)}, AttribOffset{reinterpret_cast<GLvoid*>(0* sizeof(float))});
//        VBO::unbind();
    }

    void draw(){
        vao.bind();
        draw_triangles(VerticesCount{36});
    }

    VAO vao;
    VBO vbo;
};


class DrawAxis{

public:

    void init(){

        const std_v1<geo::Pt3f> axisPts = {
            {0.f,0.f,0.f},{1.f,0.f,0.f},
            {0.f,0.f,0.f},{0.f,1.f,0.f},
            {0.f,0.f,0.f},{0.f,0.f,1.f}
        };
        const std_v1<geo::Col4f> axisColors = {
            geo::Col4f{1.f,0.f,0.f,1.f},geo::Col4f{1.f,0.f,0.f,1.f},
            geo::Col4f{0.f,1.f,0.f,1.f},geo::Col4f{0.f,1.f,0.f,1.f},
            geo::Col4f{0.f,0.f,1.f,1.f},geo::Col4f{0.f,0.f,1.f,1.f}
        };

        vao.generate();
        pos.generate();
        col.generate();

        // update buffers
        vao.bind();

        pos.load_data(FloatData{reinterpret_cast<const float*>(axisPts.data())}, SizeData{static_cast<GLsizei>(sizeof(geo::Pt3f)*axisPts.size())});
        pos.attrib(AttriIndex{0}, AttriSize{3}, AttriType{GL_FLOAT}, Stride{3 * sizeof(float)});

        col.load_data(FloatData{reinterpret_cast<const float*>(axisColors.data())},SizeData{static_cast<GLsizei>(sizeof(geo::Pt3f)*axisColors.size())});
        col.attrib(AttriIndex{1}, AttriSize{4}, AttriType{GL_FLOAT}, Stride{4 * sizeof(float)});
    }

    void draw(){
        vao.bind();
        draw_lines(VerticesCount{6});
    }

    VAO vao;
    VBO pos, col;
};

class DrawGrid{

public:

    void init(){

        std_v1<geo::Pt3f> linesPts;
        for(int ii = 0; ii < 100; ++ii){
            linesPts.emplace_back(geo::Pt3f{-10.f,0.f,-10.f+ii*0.2f});
            linesPts.emplace_back(geo::Pt3f{+10.f,0.f,-10.f+ii*0.2f});
        }
        for(int ii = 0; ii < 100; ++ii){
            linesPts.emplace_back(geo::Pt3f{-10.f+ii*0.2f,0.f,-10.f});
            linesPts.emplace_back(geo::Pt3f{-10.f+ii*0.2f,0.f,+10.f});
        }

        vao.generate();
        pos.generate();

        // update buffers
        vao.bind();

        pos.load_data(FloatData{reinterpret_cast<const float*>(linesPts.data())},SizeData{static_cast<GLsizei>(sizeof(geo::Pt3f)*linesPts.size())});
        pos.attrib(AttriIndex{0}, AttriSize{3}, AttriType{GL_FLOAT}, Stride{3 * sizeof(float)});
    }

    void draw(){
        vao.bind();
        draw_lines(VerticesCount{400});
    }

    VAO vao;
    VBO pos;
};


}

namespace tool::ui {


class SfmlQtGlWidget : public QWidget, public sf::RenderWindow{
public :

    SfmlQtGlWidget(QWidget* Parent = nullptr);
    virtual ~SfmlQtGlWidget();

    void set_model(geo::Mat4<double> m);

protected:

    // Qt
    QPaintEngine* paintEngine() const;
    bool eventFilter(QObject *object, QEvent *event);

    // # events
    void showEvent(QShowEvent*);
    void paintEvent(QPaintEvent*);
    void resizeEvent(QResizeEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);

    // sfml
    virtual void OnInit()   = 0;
    virtual void OnUpdate() = 0;
    virtual void onResize() = 0;

    // axes
    void init_axes();
    void draw_axes();

    // grid
    void init_grid();
    void draw_grid();

public :



    // time
    sf::Time currrentTime;
    sf::Clock clock;

    // inputs
    bool mouseLeftClickPressed = false;    
    bool mouseMiddleClickPressed = false;
    bool mouseRighClickPressed = false;
    double lastX=-1., lastY=-1.;

    // time
    float deltaTime = 0.0f;	// Time between current frame and last frame
    float lastFrame = 0.0f; // Time of last frame

    // camera
    graphics::Screen m_screen = {1200,1000};
    double cameraSpeed = 0.05;
    graphics::Camera camera;

    // transformations
    geo::Mat4<double> model;

    // objects to draw
    gl::DrawAxis axis;
    gl::DrawGrid grid;

    // shaders
    gl::ColoredCloudShader cloudShader;
    gl::ColoredMeshShader meshShader;


private :

    bool myInitialized = false;
    bool isKeyPressed = false;

    QTimer renderTimer;
};}
