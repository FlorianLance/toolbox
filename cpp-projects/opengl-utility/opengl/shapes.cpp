
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

#include "shapes.hpp"

// base
#include "utility/constants.hpp"
#include "utility/vector.hpp"
//#include "geometry/matrix3.hpp"

using namespace tool;
using namespace tool::gl;
using namespace tool::geo;


Torus::Torus(GLfloat outerRadius, GLfloat innerRadius, GLuint nsides, GLuint nrings){

    GLuint faces = nsides * nrings;
    size_t nVerts  = nsides * (nrings+1);   // One extra ring to duplicate first ring

    std_v1<GLfloat> points(3 * nVerts);
    std_v1<GLfloat> normals(3 * nVerts);
    std_v1<GLfloat> textCoords(2 * nVerts);
    std_v1<GLuint> elements(6 * faces);

    // Generate the vertex data
    float ringFactor = two_PI<float> / nrings;
    float sideFactor = two_PI<float> / nsides;
    size_t idx = 0, tidx = 0;
    for( GLuint ring = 0; ring <= nrings; ring++ ) {
        float u = ring * ringFactor;
        float cu = cos(u);
        float su = sin(u);
        for( GLuint side = 0; side < nsides; side++ ) {
            float v = side * sideFactor;
            float cv = cos(v);
            float sv = sin(v);
            float r = (outerRadius + innerRadius * cv);
            points[idx] = r * cu;
            points[idx + 1] = r * su;
            points[idx + 2] = innerRadius * sv;
            normals[idx] = cv * cu * r;
            normals[idx + 1] = cv * su * r;
            normals[idx + 2] = sv * r;
            textCoords[tidx] = u / two_PI<float>;
            textCoords[tidx + 1] = v / two_PI<float>;
            tidx += 2;
            // Normalize
            float len = sqrt( normals[idx] * normals[idx] +
                              normals[idx+1] * normals[idx+1] +
                              normals[idx+2] * normals[idx+2] );
            normals[idx] /= len;
            normals[idx+1] /= len;
            normals[idx+2] /= len;
            idx += 3;
        }
    }

    idx = 0;
    for( GLuint ring = 0; ring < nrings; ring++ ) {
        GLuint ringStart = ring * nsides;
        GLuint nextRingStart = (ring + 1) * nsides;
        for( GLuint side = 0; side < nsides; side++ ) {
            GLuint nextSide = (side+1) % nsides;
            // The quad
            elements[idx] = (ringStart + side);
            elements[idx+1] = (nextRingStart + side);
            elements[idx+2] = (nextRingStart + nextSide);
            elements[idx+3] = ringStart + side;
            elements[idx+4] = nextRingStart + nextSide;
            elements[idx+5] = (ringStart + nextSide);
            idx += 6;
        }
    }

    init_buffers(&elements, &points, &normals, &textCoords);
}

Axes::Axes(GLfloat length){

    std_v1<GLuint> indices = {
        0, 1,
        2, 3,
        4, 5
    };

    std_v1<GLfloat> points = {
        0.f,0.f,0.f, length,0.f,0.f,
        0.f,0.f,0.f, 0.f,length,0.f,
        0.f,0.f,0.f, 0.f,0.f,length
    };

    std_v1<GLfloat> colors = {
        1.f,0.f,0.f,1.f, 1.f,0.f,0.f,1.f,
        0.f,1.f,0.f,1.f, 0.f,1.f,0.f,1.f,
        0.f,0.f,1.f,1.f, 0.f,0.f,1.f,1.f
    };

    init_buffers(&indices, &points, &colors);
}


Grid::Grid(GLfloat width, GLfloat height, GLuint nbX, GLuint nbY){

    std_v1<GLuint> indices;
    indices.reserve(nbX*2+nbY*2);

    std_v1<GLfloat> points;
    points.reserve((nbX*2+nbY*2)*3);

    const GLfloat lX = nbX*width;
    const GLfloat lY = nbY*height;

    const GLfloat minX = -lX*0.5f;
    const GLfloat maxX = +lX*0.5f;
    for(GLuint ii = 0; ii < nbX; ++ii){
        indices.emplace_back(static_cast<GLuint>(points.size()));
        points.emplace_back(minX);
        points.emplace_back(0.f);
        points.emplace_back(minX+ii*width);

        indices.emplace_back(static_cast<GLuint>(points.size()));
        points.emplace_back(maxX);
        points.emplace_back(0.f);
        points.emplace_back(minX+ii*width);
    }

    const GLfloat minY = -lY*0.5f;
    const GLfloat maxY = +lY*0.5f;
    for(GLuint ii = 0; ii < nbY; ++ii){

        indices.emplace_back(static_cast<GLuint>(points.size()));
        points.emplace_back(minY+ii*height);
        points.emplace_back(0.f);
        points.emplace_back(minY);

        indices.emplace_back(static_cast<GLuint>(points.size()));
        points.emplace_back(minY+ii*height);
        points.emplace_back(0.f);
        points.emplace_back(maxY);
    }

    init_buffers(&indices, &points);
}



Skybox::Skybox(GLfloat size){

    GLfloat side2 = size * 0.5f;

    std_v1<GLfloat> p = {
        // Front
       -side2, -side2, side2, side2, -side2, side2, side2,  side2, side2,  -side2,  side2, side2,
       // Right
        side2, -side2, side2, side2, -side2, -side2, side2,  side2, -side2, side2,  side2, side2,
       // Back
       -side2, -side2, -side2, -side2,  side2, -side2, side2,  side2, -side2, side2, -side2, -side2,
       // Left
       -side2, -side2, side2, -side2,  side2, side2, -side2,  side2, -side2, -side2, -side2, -side2,
       // Bottom
       -side2, -side2, side2, -side2, -side2, -side2, side2, -side2, -side2, side2, -side2, side2,
       // Top
       -side2,  side2, side2, side2,  side2, side2, side2,  side2, -side2, -side2,  side2, -side2
    };

    // We don't shade a sky box, so normals aren't used.
    std_v1<GLfloat> n(p.size(), 0.0f);

    std_v1<GLuint> el = {
        0,2,1,0,3,2,
        4,6,5,4,7,6,
        8,10,9,8,11,10,
        12,14,13,12,15,14,
        16,18,17,16,19,18,
        20,22,21,20,23,22
    };

    init_buffers(&el, &p, &n);
}


Cube::Cube(GLfloat side){

    GLfloat side2 = side / 2.0f;

    std_v1<GLfloat> p = {
        // Front
       -side2, -side2, side2, side2, -side2, side2, side2,  side2, side2,  -side2,  side2, side2,
       // Right
        side2, -side2, side2, side2, -side2, -side2, side2,  side2, -side2, side2,  side2, side2,
       // Back
       -side2, -side2, -side2, -side2,  side2, -side2, side2,  side2, -side2, side2, -side2, -side2,
       // Left
       -side2, -side2, side2, -side2,  side2, side2, -side2,  side2, -side2, -side2, -side2, -side2,
       // Bottom
       -side2, -side2, side2, -side2, -side2, -side2, side2, -side2, -side2, side2, -side2, side2,
       // Top
       -side2,  side2, side2, side2,  side2, side2, side2,  side2, -side2, -side2,  side2, -side2
    };

    std_v1<GLfloat> n = {
        // Front
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
        // Right
        1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        // Back
        0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f,
        // Left
        -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
        // Bottom
        0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f,
        // Top
        0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f
    };

    std_v1<GLfloat> tex = {
        // Front
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,
        // Right
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,
        // Back
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,
        // Left
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,
        // Bottom
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,
        // Top
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f
    };

    std_v1<GLuint> el = {
        0,1,2,0,2,3,
        4,5,6,4,6,7,
        8,9,10,8,10,11,
        12,13,14,12,14,15,
        16,17,18,16,18,19,
        20,21,22,20,22,23
    };

    init_buffers(&el, &p, &n, &tex);
}


gl::Mesh::Mesh(geo::Mesh<float> *mesh){

    init_buffers(
        &mesh->triIds,
        &mesh->vertices,
        (mesh->normals.size()  != 0) ? &mesh->normals   : nullptr,
        (mesh->tCoords.size()  != 0) ? &mesh->tCoords   : nullptr,
        (mesh->tangents.size() != 0) ? &mesh->tangents  : nullptr,
        (mesh->bones.size()    != 0) ? &mesh->bones     : nullptr,
        (mesh->colors.size()   != 0) ? &mesh->colors    : nullptr
    );
}

Cloud::Cloud(size_t size, const Pt2f *vertices, const Pt3f *colors){
    init_buffers(static_cast<GLuint>(size), vertices, colors);
}

Cloud::Cloud(std_v1<Pt2f> *vertices, std_v1<Pt3f> *colors){
    init_buffers(static_cast<GLuint>(vertices->size()), vertices->data(), (colors != nullptr) ? (colors->data()) : nullptr);
}

Cloud::Cloud(size_t size, const Pt3f *vertices, const Pt3f *colors){
    init_buffers(static_cast<GLuint>(size), vertices, colors);
}

Cloud::Cloud(std_v1<geo::Pt3f> *vertices, std_v1<geo::Pt3f> *colors){
    init_buffers(static_cast<GLuint>(vertices->size()), vertices->data(), (colors != nullptr) ? (colors->data()) : nullptr);
}

Voxels::Voxels(size_t size, geo::Pt3<int> *voxels, Pt3f *colors){
    init_buffers(static_cast<GLuint>(size), voxels, colors);
}


Plane::Plane(GLfloat xsize, GLfloat zsize, size_t xdivs, size_t zdivs, GLfloat smax, GLfloat tmax){

    size_t nPoints = (xdivs + 1) * (zdivs + 1);
    std_v1<GLfloat> p(3 * nPoints);
    std_v1<GLfloat> n(3 * nPoints);
    std_v1<GLfloat> tex(2 * nPoints);
    std_v1<GLfloat> tang(4 * nPoints);
    std_v1<GLuint> el(6 * xdivs * zdivs);

    float x2 = xsize / 2.0f;
    float z2 = zsize / 2.0f;
    float iFactor = static_cast<float>(zsize) / zdivs;
    float jFactor = static_cast<float>(xsize) / xdivs;
    float texi = smax / xdivs;
    float texj = tmax / zdivs;
    float x, z;
    size_t vidx = 0, tidx = 0;

    for( size_t ii = 0; ii <= zdivs; ii++ ) {
        z = iFactor * ii - z2;
        for( size_t jj = 0; jj <= xdivs; jj++ ) {
            x = jFactor * jj - x2;
            p[vidx] = x;
            p[vidx+1] = 0.0f;
            p[vidx+2] = z;
            n[vidx] = 0.0f;
            n[vidx+1] = 1.0f;
            n[vidx+2] = 0.0f;

            tex[tidx] = jj * texi;
            tex[tidx+1] = (zdivs - ii) * texj;

            vidx += 3;
            tidx += 2;
        }
    }

    for (size_t ii = 0; ii < nPoints; ii++) {
        tang[ii * 4 + 0] = 1.0f;
        tang[ii * 4 + 1] = 0.0f;
        tang[ii * 4 + 2] = 0.0f;
        tang[ii * 4 + 3] = 1.0f;
    }

    GLuint rowStart, nextRowStart;
    size_t idx = 0;
    for( size_t ii = 0; ii < zdivs; ii++ ) {
        rowStart = static_cast<GLuint>( ii * (xdivs+1) );
        nextRowStart = static_cast<GLuint>( (ii+1) * (xdivs+1));
        for( size_t jj = 0; jj < xdivs; jj++ ) {
            el[idx]   = static_cast<GLuint>(rowStart + jj);
            el[idx+1] = static_cast<GLuint>(nextRowStart + jj);
            el[idx+2] = static_cast<GLuint>(nextRowStart + jj + 1);
            el[idx+3] = static_cast<GLuint>(rowStart + jj);
            el[idx+4] = static_cast<GLuint>(nextRowStart + jj + 1);
            el[idx+5] = static_cast<GLuint>(rowStart + jj + 1);
            idx += 6;
        }
    }

    init_buffers(&el, &p, &n, &tex, &tang);
}

Sphere::Sphere(GLfloat radius, GLuint nSlices, GLuint nStacks){

    GLuint nVerts = (nSlices+1) * (nStacks + 1);
    GLuint elements = (nSlices * 2 * (nStacks-1) ) * 3;

    // Verts
    std_v1<GLfloat> p(3 * nVerts);
    // Normals
    std_v1<GLfloat> n(3 * nVerts);
    // Tex coords
    std_v1<GLfloat> tex(2 * nVerts);
    // Elements
    std_v1<GLuint> el(elements);

    // Generate positions and normals
    GLfloat theta, phi;
    GLfloat thetaFac = two_PI<GLfloat>/ nSlices;
    GLfloat phiFac = PI<GLfloat>/ nStacks;
    GLfloat nx, ny, nz, s, t;
    GLuint idx = 0, tIdx = 0;
    for( GLuint ii = 0; ii <= nSlices; ii++ ) {

        theta = ii * thetaFac;
        s = static_cast<GLfloat>(ii) / nSlices;

        for( GLuint jj = 0; jj <= nStacks; jj++ ) {
            phi = jj * phiFac;
            t = static_cast<GLfloat>(jj) / nStacks;
            nx = sinf(phi) * cosf(theta);
            ny = sinf(phi) * sinf(theta);
            nz = cosf(phi);
            p[idx] = radius * nx; p[idx+1] = radius * ny; p[idx+2] = radius * nz;
            n[idx] = nx; n[idx+1] = ny; n[idx+2] = nz;
            idx += 3;
            tex[tIdx] = s;
            tex[tIdx+1] = t;
            tIdx += 2;
        }
    }

    // Generate the element list
    idx = 0;
    for( GLuint ii = 0; ii < nSlices; ii++ ) {
        GLuint stackStart = ii * (nStacks + 1);
        GLuint nextStackStart = (ii+1) * (nStacks+1);
        for( GLuint jj = 0; jj < nStacks; jj++ ) {
            if( jj == 0 ) {
                el[idx] = stackStart;
                el[idx+1] = stackStart + 1;
                el[idx+2] = nextStackStart + 1;
                idx += 3;
            } else if( jj == nStacks - 1) {
                el[idx] = stackStart + jj;
                el[idx+1] = stackStart + jj + 1;
                el[idx+2] = nextStackStart + jj;
                idx += 3;
            } else {
                el[idx] = stackStart + jj;
                el[idx+1] = stackStart + jj + 1;
                el[idx+2] = nextStackStart + jj + 1;
                el[idx+3] = nextStackStart + jj;
                el[idx+4] = stackStart + jj;
                el[idx+5] = nextStackStart + jj + 1;
                idx += 6;
            }
        }
    }

    init_buffers(&el, &p, &n, &tex);
}

FullscreenQuad::FullscreenQuad(){

    std_v1<GLfloat> p = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f
    };

    std_v1<GLfloat> tex = {
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
    };

    std_v1<GLuint> el = {
        0,1,2,0,2,3
    };

    init_buffers(&el, &p, nullptr, &tex);
}



namespace TeapotData {
static constexpr int patchdata[][16] =
    {
        /* rim */
        {102, 103, 104, 105, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
        /* body */
        {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27},
        {24, 25, 26, 27, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40},
        /* lid */
        {96, 96, 96, 96, 97, 98, 99, 100, 101, 101, 101, 101, 0, 1, 2, 3,},
        {0, 1, 2, 3, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117},
        /* bottom */
        {118, 118, 118, 118, 124, 122, 119, 121, 123, 126, 125, 120, 40, 39, 38, 37},
        /* handle */
        {41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56},
        {53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 28, 65, 66, 67},
        /* spout */
        {68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83},
        {80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95}
};

static constexpr float cpdata[][3] =
    {
        {0.2f, 0.f, 2.7f},
        {0.2f, -0.112f, 2.7f},
        {0.112f, -0.2f, 2.7f},
        {0.f, -0.2f, 2.7f},
        {1.3375f, 0.f, 2.53125f},
        {1.3375f, -0.749f, 2.53125f},
        {0.749f, -1.3375f, 2.53125f},
        {0.f, -1.3375f, 2.53125f},
        {1.4375f, 0.f, 2.53125f},
        {1.4375f, -0.805f, 2.53125f},
        {0.805f, -1.4375f, 2.53125f},
        {0.f, -1.4375f, 2.53125f},
        {1.5f, 0.f, 2.4f},
        {1.5f, -0.84f, 2.4f},
        {0.84f, -1.5f, 2.4f},
        {0.f, -1.5f, 2.4f},
        {1.75f, 0.f, 1.875f},
        {1.75f, -0.98f, 1.875f},
        {0.98f, -1.75f, 1.875f},
        {0.f, -1.75f, 1.875f},
        {2.f, 0.f, 1.35f},
        {2.f, -1.12f, 1.35f},
        {1.12f, -2.f, 1.35f},
        {0.f, -2.f, 1.35f},
        {2.f, 0.f, 0.9f},
        {2.f, -1.12f, 0.9f},
        {1.12f, -2.f, 0.9f},
        {0.f, -2.f, 0.9f},
        {-2.f, 0.f, 0.9f},
        {2.f, 0.f, 0.45f},
        {2.f, -1.12f, 0.45f},
        {1.12f, -2.f, 0.45f},
        {0.f, -2.f, 0.45f},
        {1.5f, 0.f, 0.225f},
        {1.5f, -0.84f, 0.225f},
        {0.84f, -1.5f, 0.225f},
        {0.f, -1.5f, 0.225f},
        {1.5f, 0.f, 0.15f},
        {1.5f, -0.84f, 0.15f},
        {0.84f, -1.5f, 0.15f},
        {0.f, -1.5f, 0.15f},
        {-1.6f, 0.f, 2.025f},
        {-1.6f, -0.3f, 2.025f},
        {-1.5f, -0.3f, 2.25f},
        {-1.5f, 0.f, 2.25f},
        {-2.3f, 0.f, 2.025f},
        {-2.3f, -0.3f, 2.025f},
        {-2.5f, -0.3f, 2.25f},
        {-2.5f, 0.f, 2.25f},
        {-2.7f, 0.f, 2.025f},
        {-2.7f, -0.3f, 2.025f},
        {-3.f, -0.3f, 2.25f},
        {-3.f, 0.f, 2.25f},
        {-2.7f, 0.f, 1.8f},
        {-2.7f, -0.3f, 1.8f},
        {-3.f, -0.3f, 1.8f},
        {-3.f, 0.f, 1.8f},
        {-2.7f, 0.f, 1.575f},
        {-2.7f, -0.3f, 1.575f},
        {-3.f, -0.3f, 1.35f},
        {-3.f, 0.f, 1.35f},
        {-2.5f, 0.f, 1.125f},
        {-2.5f, -0.3f, 1.125f},
        {-2.65f, -0.3f, 0.9375f},
        {-2.65f, 0.f, 0.9375f},
        {-2.f, -0.3f, 0.9f},
        {-1.9f, -0.3f, 0.6f},
        {-1.9f, 0.f, 0.6f},
        {1.7f, 0.f, 1.425f},
        {1.7f, -0.66f, 1.425f},
        {1.7f, -0.66f, 0.6f},
        {1.7f, 0.f, 0.6f},
        {2.6f, 0.f, 1.425f},
        {2.6f, -0.66f, 1.425f},
        {3.1f, -0.66f, 0.825f},
        {3.1f, 0.f, 0.825f},
        {2.3f, 0.f, 2.1f},
        {2.3f, -0.25f, 2.1f},
        {2.4f, -0.25f, 2.025f},
        {2.4f, 0.f, 2.025f},
        {2.7f, 0.f, 2.4f},
        {2.7f, -0.25f, 2.4f},
        {3.3f, -0.25f, 2.4f},
        {3.3f, 0.f, 2.4f},
        {2.8f, 0.f, 2.475f},
        {2.8f, -0.25f, 2.475f},
        {3.525f, -0.25f, 2.49375f},
        {3.525f, 0.f, 2.49375f},
        {2.9f, 0.f, 2.475f},
        {2.9f, -0.15f, 2.475f},
        {3.45f, -0.15f, 2.5125f},
        {3.45f, 0.f, 2.5125f},
        {2.8f, 0.f, 2.4f},
        {2.8f, -0.15f, 2.4f},
        {3.2f, -0.15f, 2.4f},
        {3.2f, 0.f, 2.4f},
        {0.f, 0.f, 3.15f},
        {0.8f, 0.f, 3.15f},
        {0.8f, -0.45f, 3.15f},
        {0.45f, -0.8f, 3.15f},
        {0.f, -0.8f, 3.15f},
        {0.f, 0.f, 2.85f},
        {1.4f, 0.f, 2.4f},
        {1.4f, -0.784f, 2.4f},
        {0.784f, -1.4f, 2.4f},
        {0.f, -1.4f, 2.4f},
        {0.4f, 0.f, 2.55f},
        {0.4f, -0.224f, 2.55f},
        {0.224f, -0.4f, 2.55f},
        {0.f, -0.4f, 2.55f},
        {1.3f, 0.f, 2.55f},
        {1.3f, -0.728f, 2.55f},
        {0.728f, -1.3f, 2.55f},
        {0.f, -1.3f, 2.55f},
        {1.3f, 0.f, 2.4f},
        {1.3f, -0.728f, 2.4f},
        {0.728f, -1.3f, 2.4f},
        {0.f, -1.3f, 2.4f},
        {0.f, 0.f, 0.f},
        {1.425f, -0.798f, 0.f},
        {1.5f, 0.f, 0.075f},
        {1.425f, 0.f, 0.f},
        {0.798f, -1.425f, 0.f},
        {0.f, -1.5f, 0.075f},
        {0.f, -1.425f, 0.f},
        {1.5f, -0.84f, 0.075f},
        {0.84f, -1.5f, 0.075f}
};

}


void Teapot::generate_patches(std_v1<GLfloat> &p, std_v1<GLfloat> &n, std_v1<GLfloat> &tc, std_v1<GLuint> &el, int grid){

    std::vector<GLfloat> B(4*(grid+1));  // Pre-computed Bernstein basis functions
    std::vector<GLfloat> dB(4*(grid+1)); // Pre-computed derivitives of basis functions

    int idx = 0, elIndex = 0, tcIndex = 0;

    // Pre-compute the basis functions  (Bernstein polynomials)
    // and their derivatives
    compute_basis_functions(B, dB, grid);

    // Build each patch
    // The rim
    build_patch_reflect(0, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, true, true);
    // The body
    build_patch_reflect(1, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, true, true);
    build_patch_reflect(2, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, true, true);
    // The lid
    build_patch_reflect(3, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, true, true);
    build_patch_reflect(4, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, true, true);
    // The bottom
    build_patch_reflect(5, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, true, true);
    // The handle
    build_patch_reflect(6, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, false, true);
    build_patch_reflect(7, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, false, true);
    // The spout
    build_patch_reflect(8, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, false, true);
    build_patch_reflect(9, B, dB, p, n, tc, el, idx, elIndex, tcIndex, grid, false, true);
}

void Teapot::build_patch_reflect(
    int patchNum, std_v1<GLfloat> &B, std_v1<GLfloat> &dB, std_v1<GLfloat> &v,
    std_v1<GLfloat> &n, std_v1<GLfloat> &tc, std_v1<GLuint> &el,
    int &index, int &elIndex, int &tcIndex, int grid, bool reflectX, bool reflectY){

    geo::Vec3f patch[4][4];
    geo::Vec3f patchRevV[4][4];
    get_patch(patchNum, patch, false);
    get_patch(patchNum, patchRevV, true);

    // Patch without modification
    build_patch(patch, B, dB, v, n, tc, el,
               index, elIndex, tcIndex, grid, geo::Mat3f(true), true);

    // Patch reflected in x
    if( reflectX ) {
        build_patch(patchRevV, B, dB, v, n, tc, el, index, elIndex, tcIndex, grid,
            geo::Mat3f{
                -1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            },
            false
        );
    }

    // Patch reflected in y
    if( reflectY ) {
        build_patch(patchRevV, B, dB, v, n, tc, el, index, elIndex, tcIndex, grid,
            geo::Mat3f{
                1.0f, 0.0f, 0.0f,
                0.0f, -1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            },
            false
        );
    }

    // Patch reflected in x and y
    if( reflectX && reflectY ) {
        build_patch(patch, B, dB, v, n, tc, el, index, elIndex, tcIndex, grid,
            geo::Mat3f{
                -1.0f, 0.0f, 0.0f,
                0.0f, -1.0f, 0.0f,
                0.0f, 0.0f, 1.0f
            },
            true
        );
    }
}

void Teapot::build_patch(geo::Vec3f patch[][4],
    std_v1<GLfloat> &B, std_v1<GLfloat> &dB, std_v1<GLfloat> &v,
    std_v1<GLfloat> &n, std_v1<GLfloat> &tc, std_v1<GLuint> &el,
    int &index, int &elIndex, int &tcIndex, int grid, geo::Mat3f reflect, bool invertNormal){

    int startIndex = index / 3;
    float tcFactor = 1.0f / grid;

    for( int i = 0; i <= grid; i++ ){

        for( int j = 0 ; j <= grid; j++){

            Vec3f pt = reflect * evaluate(i,j,B,patch);
            Vec3f norm = reflect * evaluate_normal(i,j,B,dB,patch);
            if( invertNormal ){
                norm = -norm;
            }

            v[index]    = pt.x();
            v[index+1]  = pt.y();
            v[index+2]  = pt.z();

            n[index]    = norm.x();
            n[index+1]  = norm.y();
            n[index+2]  = norm.z();

            tc[tcIndex] = i * tcFactor;
            tc[tcIndex+1] = j * tcFactor;

            index += 3;
            tcIndex += 2;
        }
    }

    for( int i = 0; i < grid; i++ ){

        int iStart = i * (grid+1) + startIndex;
        int nextiStart = (i+1) * (grid+1) + startIndex;

        for( int j = 0; j < grid; j++){

            el[elIndex] = iStart + j;
            el[elIndex+1] = nextiStart + j + 1;
            el[elIndex+2] = nextiStart + j;

            el[elIndex+3] = iStart + j;
            el[elIndex+4] = iStart + j + 1;
            el[elIndex+5] = nextiStart + j + 1;

            elIndex += 6;
        }
    }
}

void Teapot::get_patch(int patchNum, geo::Vec3f patch[][4], bool reverseV){

    for( int u = 0; u < 4; u++) {          // Loop in u direction
        for( int v = 0; v < 4; v++ ) {     // Loop in v direction
            if( reverseV ) {
                patch[u][v] = Vec3f{
                    TeapotData::cpdata[TeapotData::patchdata[patchNum][u*4+(3-v)]][0],
                    TeapotData::cpdata[TeapotData::patchdata[patchNum][u*4+(3-v)]][1],
                    TeapotData::cpdata[TeapotData::patchdata[patchNum][u*4+(3-v)]][2]
                };
            } else {
                patch[u][v] = Vec3f{
                    TeapotData::cpdata[TeapotData::patchdata[patchNum][u*4+v]][0],
                    TeapotData::cpdata[TeapotData::patchdata[patchNum][u*4+v]][1],
                    TeapotData::cpdata[TeapotData::patchdata[patchNum][u*4+v]][2]
                };
            }
        }
    }
}

void Teapot::compute_basis_functions(std_v1<GLfloat> &B, std_v1<GLfloat> &dB, int grid){

    float inc = 1.0f / grid;
    for( int i = 0; i <= grid; i++ ){

        float t = i * inc;
        float tSqr = t * t;
        float oneMinusT = (1.0f - t);
        float oneMinusT2 = oneMinusT * oneMinusT;

        B[i*4 + 0] = oneMinusT * oneMinusT2;
        B[i*4 + 1] = 3.0f * oneMinusT2 * t;
        B[i*4 + 2] = 3.0f * oneMinusT * tSqr;
        B[i*4 + 3] = t * tSqr;

        dB[i*4 + 0] = -3.0f * oneMinusT2;
        dB[i*4 + 1] = -6.0f * t * oneMinusT + 3.0f * oneMinusT2;
        dB[i*4 + 2] = -3.0f * tSqr + 6.0f * t * oneMinusT;
        dB[i*4 + 3] = 3.0f * tSqr;
    }
}

Vec3f Teapot::evaluate(int gridU, int gridV, std_v1<GLfloat> &B, Vec3f patch[][4]){

    Vec3f p{0.0f,0.0f,0.0f};
    for( int i = 0; i < 4; i++) {
        for( int j = 0; j < 4; j++) {
            p += patch[i][j] * B[gridU*4+i] * B[gridV*4+j];
        }
    }
    return p;
}

Vec3f Teapot::evaluate_normal(int gridU, int gridV, std_v1<GLfloat> &B, std_v1<GLfloat> &dB, Vec3f patch[][4]){

    Vec3f du{0.0f,0.0f,0.0f};
    Vec3f dv{0.0f,0.0f,0.0f};

    for( int i = 0; i < 4; i++) {
        for( int j = 0; j < 4; j++) {
            du += patch[i][j] * dB[gridU*4+i] * B[gridV*4+j];
            dv += patch[i][j] * B[gridU*4+i] * dB[gridV*4+j];
        }
    }

    auto norm = geo::cross(du, dv);
    if(geo::norm(norm) != 0.0f){
        norm = geo::normalize(norm);
    }
    return norm;
}

void Teapot::move_lid(int grid, std_v1<GLfloat> &p, const Mat4f &lidTransform){

    static_cast<void>(lidTransform);
    int start = 3 * 12 * (grid+1) * (grid+1);
    int end = 3 * 20 * (grid+1) * (grid+1);

    for( int i = start; i < end; i+=3 ){

        geo::Vec4f vert = geo::Vec4f{p[i], p[i+1], p[i+2], 1.0f };
//        vert    = lidTransform.multiply_vector(vert); //lidTransform * vert;
        p[i]    = vert.x();
        p[i+1]  = vert.y();
        p[i+2]  = vert.z();
    }
}

Teapot::Teapot(int grid, const geo::Mat4f &lidTransform){

    int verts = 32 * (grid + 1) * (grid + 1);
    int faces = grid * grid * 32;
    std::vector<GLfloat> p( verts * 3 );
    std::vector<GLfloat> n( verts * 3 );
    std::vector<GLfloat> tc( verts * 2 );
    std::vector<GLuint> el( faces * 6 );

    generate_patches( p, n, tc, el, grid );
    move_lid(grid, p, lidTransform);

    init_buffers(&el,&p, &n, &tc);
}

Frustum::Frustum(){
    orient( {0.0f,0.0f,1.0f}, {0.0f,0.0f,0.0f}, {0.0f,1.0f,0.0f});
    set_perspective(50.0f, 1.0f, 0.5f, 100.0f);
}

void Frustum::orient(const Pt3f &pos, const Pt3f &at, const Pt3f &up){
    m_view  = Mat4f::LookAt(pos, at, up);
    m_position = pos;
}

void Frustum::set_perspective(float fovy, float ar, float nearDist, float farDist){

    this->fovy = fovy;
    this->ar = ar;
    this->mNear = nearDist;
    this->mFar = farDist;

    const float dy = mNear * tanf( deg_2_rad(fovy) / 2.0f );
    const float dx = ar * dy;
    const float fdy = mFar * tanf( deg_2_rad(fovy) / 2.0f );
    const float fdx = ar * fdy;

    std::vector<GLfloat> vertices = {
        0,0,0,
        dx, dy, -mNear,

        -dx, dy, -mNear,
        -dx, -dy, -mNear,
        dx, -dy, -mNear,

        fdx, fdy, -mFar,
        -fdx, fdy, -mFar,
        -fdx, -fdy, -mFar,
        fdx, -fdy, -mFar
    };

    std::vector<GLuint> indices = {
        0, 5, 0, 6, 0, 7, 0, 8,
        // The near plane
        1, 2, 2, 3, 3, 4, 4, 1,
        // The far plane
        5,6,6,7,7,8,8,5
    };

    init_buffers(&indices, &vertices);
}

Mat4f Frustum::view_matrix() const{    
    return m_view;
}

Mat4f Frustum::inverse_view_matrix() const{
    return m_view.inverse();
}

Mat4f Frustum::projection_matrix() const{
    return Mat4f::Perspective(deg_2_rad(fovy), ar, mNear, mFar);
}

Pt3f Frustum::origin() const{
    return m_position;
}

