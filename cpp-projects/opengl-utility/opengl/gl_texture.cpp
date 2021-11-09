

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



#include "gl_texture.hpp"

using namespace tool;
using namespace tool::gl;


void Texture2D::load_texture(tool::graphics::Texture2D *texture, TextureOptions options){

    TBO::generate();
    TBO::set_texture_options(options);
    TBO::load_texture(texture);
}

void CubeMap::load_textures(std_a1<tool::graphics::Texture2D, 6> *textures){

    TBO::generate();

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::linear;
    options.wrapS     = TextureWrapMode::clamp_to_edge;
    options.wrapT     = TextureWrapMode::clamp_to_edge;
    options.wrapR     = TextureWrapMode::clamp_to_edge;
    TBO::set_texture_options(options);

    TBO::load_texture(textures);
}

void Texture2D::load_projected_texture(tool::graphics::Texture2D *texture){

    TBO::generate();

    TextureOptions options;
    options.wrapS       = TextureWrapMode::clamp_to_border;
    options.wrapT       = TextureWrapMode::clamp_to_border;
    options.minFilter   = TextureMinFilter::linear;
    options.magFilter   = TextureMagFilter::linear;
    TBO::set_texture_options(options);

    TBO::load_texture(texture);
}

void Texture2D::init_render(GLsizei width, GLsizei height, int nbChannels){
    TBO::generate();
    TBO::init_data_unsigned_8_bits(width, height, 1, nbChannels);
}

void Texture2D::init_hdr_render(GLsizei width, GLsizei height, int nbChannels){
    TBO::generate();
    TBO::init_data_float_32_bits(width, height, 1, nbChannels);
}

void Texture2D::init_image_8ui(GLsizei width, GLsizei height, int nbChannels){
    TBO::generate();
    TBO::init_data_unsigned_8_bits(width, height, 1, nbChannels);
}

void Texture2D::init_image_32ui(GLsizei width, GLsizei height, int nbChannels){
    TBO::generate();
    TBO::init_data_unsigned_32_bits(width, height, 1, nbChannels);
}

void Texture2D::init_image_32f(GLsizei width, GLsizei height, int nbChannels){
    TBO::generate();
    TBO::init_data_float_32_bits(width, height, 1, nbChannels);
}

void Texture2D::update_image_8ui(GLubyte *data, GLsizei width, GLsizei height, GLint xOffset, GLint yOffset){
    TBO::update_data_unsigned_8_bits(data, width, height, 1, xOffset, yOffset, 0);
}

void Texture2D::update_image_32ui(GLuint *data, GLsizei width, GLsizei height, GLint xOffset, GLint yOffset){
    TBO::update_data_unsigned_32_bits(data, width, height, 1, xOffset, yOffset, 0);
}

void Texture2D::update_image_32f(GLfloat *data, GLsizei width, GLsizei height, GLint xOffset, GLint yOffset){
    TBO::update_data_float_32_bits(data, width, height, 1, xOffset, yOffset, 0);
}

void Texture2D::load_data(float *data, GLsizei width, GLsizei height, int nbChannels, bool useInternal16Bits, TextureOptions options){

    TBO::generate();
    TBO::set_texture_options(options);

    if(useInternal16Bits){
        TBO::load_data_float_16_bits(data, width, height, 1, nbChannels);
    }else{
        TBO::load_data_float_32_bits(data, width, height, 1, nbChannels);
    }
}


void GeometryTexture2D::init_position(GLsizei width, GLsizei height){

    TBO::generate();

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::nearest;
    options.wrapS = TextureWrapMode::clamp_to_edge; // test
    options.wrapT = TextureWrapMode::clamp_to_edge; // test
    options.maxLevel  = 0;

    TBO::set_texture_options(options);

    TBO::init_data_float_32_bits(width, height, 1, 3);
}

void GeometryTexture2D::init_color(GLsizei width, GLsizei height){

    TBO::generate();

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::nearest;
    options.maxLevel  = 0;
    TBO::set_texture_options(options);

    TBO::init_data_unsigned_8_bits(width, height, 1, 3);
}

void GeometryTexture2D::init_ao(GLsizei width, GLsizei height){

    TBO::generate();

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::nearest;
    options.maxLevel  = 0;
    TBO::set_texture_options(options);

    TBO::init_data_float_16_bits(width, height, 1, 1);
}

void GeometryMultisampleTexture2D::init_color(GLsizei width, GLsizei height, GLsizei samples){

    TBO::generate();

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::nearest;
    options.maxLevel  = 0;
    TBO::set_texture_options(options);

    TBO::init_multisample_data_unsigned_8_bits(width, height, 1, 3, samples);
}

void GeometryMultisampleTexture2D::init_position(GLsizei width, GLsizei height, GLsizei samples){

    TBO::generate();

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::nearest;
    options.maxLevel  = 0;
    TBO::set_texture_options(options);

    TBO::init_multisample_data_float_32_bits(width, height, 1, 3, samples);
}
