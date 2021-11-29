
/*******************************************************************************
** Toolbox-opengl-utility                                                     **
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
#include <vector>

// base
#include "geometry/mesh.hpp"

// local
#include "buffer/vertex_buffer_object.hpp"
#include "buffer/element_buffer_object.hpp"
#include "buffer/texture_buffer_object.hpp"
#include "vao.hpp"

namespace tool::gl{


class Drawable{
public:

    virtual ~Drawable(){
        clean();
    }

    virtual void render() const = 0;
    virtual void render_adjacency() const{}
    virtual void render_patches() const{}
    virtual void render_instances(int count, int baseId = 0) const {
        static_cast<void>(count);
        static_cast<void>(baseId);

    }
    virtual void clean(){}

    constexpr GLuint indices_nb() const noexcept{ return nIndices;}

protected:
    GLsizei nIndices = 0;
};


class PointMesh : public Drawable{

protected:


    VBO pointsB;
    VBO colorsB;
    VAO vao;

    bool buffersInitialized = false;

    void init_buffers(
        GLuint size,
        geo::Pt3f *points,
        geo::Pt3f *colors = nullptr
    );

    void init_buffers(
        GLuint size,
        geo::Pt2f *points,
        geo::Pt3f *colors = nullptr
    );

    void init_buffers(
        GLuint size,
        geo::Pt3<int> *voxels,
        geo::Pt3f *colors = nullptr
    );


public:

    virtual void render() const override;
    virtual void render_patches() const override;
    virtual void clean() override;


};


class LineMesh : public Drawable{
protected:

    VBO pointsB;
    VBO colorsB;
    EBO indicesB;
    VAO vao;

    bool buffersInitialized = false;

    void init_buffers(
        std_v1<GLuint>  *indices,
        std_v1<GLfloat> *points,
        std_v1<GLfloat> *colors = nullptr
    );

public:


    virtual void render() const override;
    virtual void clean() override;
};

class TriangleMesh : public Drawable {

public:

    bool hasTexCoord = false;
    bool hasTangents = false;
    bool hasNormals  = false;
    bool hasBones    = false;
    bool hasColors   = false;

protected:

    VBO pointsB;
    VBO normalsB;
    VBO tangentsB;
    VBO colorsB;
    VBO texCoordsB;
    VBO bonesB;
    EBO indicesB;
    VAO vao;

    bool buffersInitialized = false;

    void init_buffers(
        std_v1<GLuint>  *indices,
        std_v1<GLfloat> *points,
        std_v1<GLfloat> *normals   = nullptr,
        std_v1<GLfloat> *texCoords = nullptr,
        std_v1<GLfloat> *tangents  = nullptr,
        std_v1<GLfloat> *colors    = nullptr
    );

    void init_buffers(
        std_v1<geo::TriIds> *indices,
        std_v1<geo::Pt3f> *points,
        std_v1<geo::Pt3f> *normals   = nullptr,
        std_v1<geo::Pt2f> *texCoords = nullptr,
        std_v1<geo::Pt4f> *tangents  = nullptr,
        std_v1<geo::BoneData> *bones = nullptr,
        std_v1<geo::Pt4f> *colors    = nullptr
    );

//    void init_buffers(
//        std_v1<geo::TriIds> *indices,
//        std_v1<geo::Pt3f> *points,
//        std_v1<geo::Pt3f> *normals   = nullptr,
//        std_v1<geo::Pt3f> *colors    = nullptr,
//        std_v1<geo::Pt4f> *tangents  = nullptr,
//        std_v1<geo::BoneData> *bones = nullptr
//    );

public:

    virtual void render() const override;
    virtual void render_adjacency() const override;
    virtual void clean() override;
};


}
