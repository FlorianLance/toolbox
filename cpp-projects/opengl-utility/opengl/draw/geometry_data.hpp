
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
#include "opengl/buffer/vertex_buffer_object.hpp"
#include "opengl/buffer/element_buffer_object.hpp"
#include "opengl/buffer/texture_buffer_object.hpp"
#include "opengl/vao.hpp"

namespace tool::gl{

class GeometryData{
public:

    VAO vao;
    VBO pointsB;
    GLsizei nIndices = 0;

    virtual void render() const = 0;
    virtual void render_adjacency() const{}
    virtual void render_patches() const{}
    virtual void render_instances(int count, int baseId = 0) const {
        static_cast<void>(count);
        static_cast<void>(baseId);

    }

    virtual void clean() = 0;

protected:
    bool buffersInitialized = false;   
};


class PointMeshData : public GeometryData{
public:
    void init_buffers(
        GLuint size,
        const geo::Pt3f *points,
        const geo::Pt3f *colors = nullptr
    );

    void init_buffers(
        GLuint size,
        const geo::Pt2f *points,
        const geo::Pt3f *colors = nullptr
    );

    void init_buffers(
        GLuint size,
        const geo::Pt3<int> *voxels,
        const geo::Pt3f *colors = nullptr
    );

    void render() const override;
    void render_patches() const override;

    void clean() override;

protected:

    VBO colorsB;
};


class LineMeshData : public GeometryData{
public:

    void init_buffers(
        std_v1<GLuint>  *indices,
        std_v1<GLfloat> *points,
        std_v1<GLfloat> *colors = nullptr
    );

    void render() const override;
    void clean() override;

protected:

    VBO colorsB;
    EBO indicesB;
};

class TriangleMeshData : public GeometryData {
public:

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

    void render() const override;
    void render_adjacency() const override;
    void clean() override;

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

};

}
