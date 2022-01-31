
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

// local
#include "opengl/buffer/texture_buffer_object.hpp"

namespace tool::gl{

class Texture2D : public TBO{

public:

    Texture2D() : TBO(TextureMode::texture_2d){}

    void load_texture(graphics::Texture2D *texture, TextureOptions options = {});
    void load_projected_texture(graphics::Texture2D *texture);
    void load_data(float *data, GLsizei w, GLsizei h, int nbChannels, bool useInternal16Bits, TextureOptions options = {});

    void init_render(GLsizei w, GLsizei h, int nbChannels = 3);
    void init_hdr_render(GLsizei w, GLsizei h, int nbChannels = 3);

    void init_image_8ui(GLsizei w, GLsizei h, int nbChannels);
    void update_image_8ui(GLubyte *data, GLsizei w, GLsizei h, GLint xOffset, GLint yOffset);

    void init_image_32ui(GLsizei w, GLsizei height, int nbChannels);
    void update_image_32ui(GLuint *data, GLsizei w, GLsizei h, GLint xOffset, GLint yOffset);

    void init_image_32f(GLsizei w, GLsizei h, int nbChannels);
    void update_image_32f(GLfloat *data, GLsizei w, GLsizei h, GLint xOffset, GLint yOffset);

    void init_or_update_8ui(GLsizei w, GLsizei h, int numChannels, geo::Pt3<uint8_t> *data);

};

class CubeMap : public TBO{
public:
    CubeMap() : TBO(TextureMode::cubemap){}
    void load_textures(std_a1<graphics::Texture2D,6> *textures);
};

class GeometryTexture2D : public TBO{

public:

    GeometryTexture2D() : TBO(TextureMode::texture_2d){}

    void init_position(GLsizei w, GLsizei h);
    void init_color(GLsizei w, GLsizei h);
    void init_ao(GLsizei w, GLsizei h);
};

class GeometryMultisampleTexture2D : public TBO{

    GeometryMultisampleTexture2D() : TBO(TextureMode::texture_2d_multisample){}

    void init_position(GLsizei w, GLsizei h, GLsizei samples = 4);
    void init_color(GLsizei w, GLsizei h, GLsizei samples = 4);
};

}


