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

// glew
#include <GL/glew.h>

// base
#include "geometry/matrix3.hpp"
#include "geometry/matrix4.hpp"
#include "geometry/mesh.hpp"

// local
#include "drawable.hpp"

namespace tool::gl{

class FullscreenQuad : public TriangleMesh{
public:
    FullscreenQuad();
};

class Torus : public TriangleMesh{
public:
    Torus(GLfloat outerRadius, GLfloat innerRadius, GLuint nsides, GLuint nrings);
};

class Teapot : public TriangleMesh{
private:

    void generate_patches(std_v1<GLfloat> & p, std_v1<GLfloat> & n, std_v1<GLfloat> & tc, std_v1<GLuint> & el, int grid);
    void build_patch_reflect(int patchNum,std_v1<GLfloat> & B, std_v1<GLfloat> & dB,std_v1<GLfloat> & v, std_v1<GLfloat> & n,std_v1<GLfloat> & tc, std_v1<GLuint> & el,int &index, int &elIndex, int &tcIndex, int grid,bool reflectX, bool reflectY);
    void build_patch(geo::Vec3f patch[][4], std_v1<GLfloat> & B, std_v1<GLfloat> & dB, std_v1<GLfloat> & v, std_v1<GLfloat> & n,std_v1<GLfloat> & tc, std_v1<GLuint> & el, int &index, int &elIndex, int &tcIndex, int grid, geo::Mat3f reflect, bool invertNormal);
    void get_patch( int patchNum, geo::Vec3f patch[][4], bool reverseV );
    void compute_basis_functions( std_v1<GLfloat> & B, std_v1<GLfloat> & dB, int grid );
    geo::Vec3f evaluate( int gridU, int gridV, std_v1<GLfloat> & B, geo::Vec3f patch[][4] );
    geo::Vec3f evaluate_normal(  int gridU, int gridV, std_v1<GLfloat> & B, std_v1<GLfloat> & dB, geo::Vec3f patch[][4] );
    void move_lid(int grid, std_v1<GLfloat> & p, const geo::Mat4f &lidTransform);

public:
    Teapot(int grid, const geo::Mat4f &lidTransform = geo::Mat4f(true));
};

class Grid : public LineMesh{
public:
    Grid(GLfloat width, GLfloat height, GLuint nbX, GLuint nbY);
};

class Axes : public LineMesh{
public:
    Axes(GLfloat length);
};

class Frustum : public LineMesh{
public:

    Frustum();
    void orient( const geo::Pt3f &pos, const geo::Pt3f &at, const geo::Pt3f &up);
    void set_perspective( float fovy, float ar, float nearDist, float farDist );

    geo::Mat4f view_matrix() const;
    geo::Mat4f inverse_view_matrix() const;
    geo::Mat4f projection_matrix() const;
    geo::Pt3f origin() const;

private:

    float mNear, mFar, fovy, ar;
    geo::Pt3f m_position;
    geo::Mat4f m_view;
};


class Plane : public TriangleMesh{
public:
    Plane(GLfloat xsize, GLfloat zsize, size_t xdivs, size_t zdivs, GLfloat smax = 1.0f, GLfloat tmax = 1.0f);
};

class Skybox : public TriangleMesh{
public:
    Skybox(GLfloat size);
};

class Cube : public TriangleMesh{
public:
    Cube(GLfloat side);
};

class Sphere : public TriangleMesh{
public:
    Sphere(GLfloat radius, GLuint nSlices, GLuint nStacks);
};

class Mesh : public TriangleMesh{
public:
    Mesh(geo::Mesh<float> *mesh);
};

class Cloud : public PointMesh{
public:
    Cloud(size_t size, geo::Pt2f *vertices, geo::Pt3f *colors);
    Cloud(std_v1<geo::Pt2f> *vertices, std_v1<geo::Pt3f> *colors);

    Cloud(size_t size, geo::Pt3f *vertices, geo::Pt3f *colors);
    Cloud(std_v1<geo::Pt3f> *vertices, std_v1<geo::Pt3f> *colors);
};

class Voxels : public PointMesh{
public :
    Voxels(size_t size, geo::Pt3<int> *voxels, geo::Pt3f *colors);
};

}
