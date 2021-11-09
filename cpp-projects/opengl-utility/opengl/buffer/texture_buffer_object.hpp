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
#include <array>

// base
#include "geometry/point3.hpp"
#include "utility/vector_utility.hpp"
#include "graphics/texture.hpp"

// local
#include "opengl/utility/gl_utility.hpp"

namespace tool::gl{

using TextureType       = graphics::TextureType;
using TextureOptions    = graphics::TextureOptions;

using TextureMinFilter = graphics::TextureMinFilter;
using TMinFiltGl = std::tuple<graphics::TextureMinFilter,unsigned int>;
static constexpr TupleArray<TextureMinFilter::SizeEnum, TMinFiltGl> textureMinFiltersGl ={{
    TMinFiltGl{TextureMinFilter::nearest,                 GL_NEAREST},
    TMinFiltGl{TextureMinFilter::linear,                  GL_LINEAR},
    TMinFiltGl{TextureMinFilter::nearest_mipmap_nearest,  GL_NEAREST_MIPMAP_NEAREST},
    TMinFiltGl{TextureMinFilter::lLinear_mimmap_nearest,  GL_LINEAR_MIPMAP_NEAREST},
    TMinFiltGl{TextureMinFilter::nearest_mipmap_linear,   GL_NEAREST_MIPMAP_LINEAR},
    TMinFiltGl{TextureMinFilter::linear_mipmap_linear,    GL_LINEAR_MIPMAP_LINEAR},
}};

[[maybe_unused]] static unsigned int to_gl(TextureMinFilter t) {
    return textureMinFiltersGl.at<0,1>(t);
}

using TextureMagFilter = graphics::TextureMagFilter;
using TMagFiltGl = std::tuple<graphics::TextureMagFilter,unsigned int>;
static constexpr TupleArray<TextureMagFilter::SizeEnum, TMagFiltGl> textureMagFiltersGl ={{
    TMagFiltGl{TextureMagFilter::nearest,             GL_NEAREST},
    TMagFiltGl{TextureMagFilter::linear,              GL_LINEAR},
}};

[[maybe_unused]] static unsigned int to_gl(TextureMagFilter t) {
    return textureMagFiltersGl.at<0,1>(t);
}

using TextureMode = graphics::TextureMode;
using TTexModeGl = std::tuple<graphics::TextureMode, GLenum>;
static constexpr TupleArray<TextureMode::SizeEnum, TTexModeGl> textureModesGl ={{
    TTexModeGl{TextureMode::texture_1d,                   GL_TEXTURE_1D},
    TTexModeGl{TextureMode::texture_2d,                   GL_TEXTURE_2D},
    TTexModeGl{TextureMode::texture_3d,                   GL_TEXTURE_3D},
    TTexModeGl{TextureMode::rectangle,                    GL_TEXTURE_RECTANGLE},
    TTexModeGl{TextureMode::buffer,                       GL_TEXTURE_BUFFER},
    TTexModeGl{TextureMode::cubemap,                      GL_TEXTURE_CUBE_MAP},
    TTexModeGl{TextureMode::texture_1d_array,             GL_TEXTURE_1D_ARRAY},
    TTexModeGl{TextureMode::texture_2d_array,             GL_TEXTURE_2D_ARRAY},
    TTexModeGl{TextureMode::cubemap_array,                GL_TEXTURE_CUBE_MAP_ARRAY},
    TTexModeGl{TextureMode::texture_2d_multisample,       GL_TEXTURE_2D_MULTISAMPLE},
    TTexModeGl{TextureMode::texture_2d_multisample_array, GL_TEXTURE_2D_MULTISAMPLE_ARRAY}
}};

[[maybe_unused]] static GLenum to_gl(TextureMode t) {
    return textureModesGl.at<0,1>(t);
}

using TextureWrapMode = graphics::TextureWrapMode;
using TTexWrapModeGl = std::tuple<graphics::TextureWrapMode, unsigned int>;
static constexpr TupleArray<TextureWrapMode::SizeEnum, TTexWrapModeGl> textureWrapModesGl ={{
    TTexWrapModeGl{TextureWrapMode::clamp_to_edge,        GL_CLAMP_TO_EDGE},
    TTexWrapModeGl{TextureWrapMode::clamp_to_border,      GL_CLAMP_TO_BORDER},
    TTexWrapModeGl{TextureWrapMode::mirrored_repeat,      GL_MIRRORED_REPEAT},
    TTexWrapModeGl{TextureWrapMode::repeat,               GL_REPEAT},
    TTexWrapModeGl{TextureWrapMode::mirror_clamp_to_edge, GL_MIRROR_CLAMP_TO_EDGE},
}};

[[maybe_unused]] static unsigned int to_gl(TextureWrapMode t) {
    return textureWrapModesGl.at<0,1>(t);
}

struct TBO{

    TBO() = default;
    TBO(TextureMode mode) : m_mode(mode){}
    TBO(const TBO&) = delete;
    TBO& operator=(const TBO&) = delete;
    TBO(TBO&& other) = default;
    TBO& operator=(TBO&& other) = default;

    ~TBO(){
        clean();
    }

    constexpr TextureName id() const{return m_id;}

    void clean();

    bool bind();
    void bind_image(GLuint unit, GLint level = 0, GLboolean layered = GL_FALSE, GLint layer = 0, GLenum access = GL_READ_WRITE);

    static bool bind_textures(std_v1<TextureName> textures, GLuint first = 0);
    static bool bind_image_textures(std_v1<TextureName> textures, GLuint first = 0);

    static void unbind_textures(GLuint first, GLuint count);

    void set_texture_options(TextureOptions options = {});
    void get_hdr_texture_data(std::vector<GLfloat> &data);

    constexpr GLsizei nb_channels() const{return m_nbChannels;}

    void debug_generate();

protected:

    void generate();

    // init
    void init_data_unsigned_8_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels);
    void init_data_unsigned_32_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels);
    void init_data_float_16_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels);
    void init_data_float_32_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels);
    void init_multisample_data_unsigned_8_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels, GLsizei samples);
    void init_multisample_data_float_16_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels, GLsizei samples);
    void init_multisample_data_float_32_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels, GLsizei samples);
    // init & load
    void load_texture(graphics::Texture2D *texture);
    void load_texture(std_a1<graphics::Texture2D,6> *textures);
    void load_data_unsigned_8_bits(GLubyte *data, GLsizei width, GLsizei height, GLsizei depth, int nbChannels);
    void load_data_float_16_bits(GLfloat *data, GLsizei width, GLsizei height, GLsizei depth, int nbChannels);
    void load_data_float_32_bits(GLfloat *data, GLsizei width, GLsizei height, GLsizei depth, int nbChannels);
    // update
    void update_data_unsigned_8_bits(GLubyte *data, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset);
    void update_data_unsigned_32_bits(GLuint *data, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset);
    void update_data_float_32_bits(GLfloat *data, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset);

    TextureName m_id = 0;

    GLenum m_internalFormat;
    GLenum m_format;
    GLenum m_type;

    int m_nbChannels=0;
    GLsizei m_width=0;
    GLsizei m_height=0;
    GLsizei m_depth=0;
    int m_levelsNb = 1;

private:

    void gl_texture_storage();
    void gl_texture_storage_2d();
    void gl_texture_storage_3d();

//    void gl_texture_multisample_storage(GLsizei samples = 4, GLboolean fixedsamplelocations = GL_TRUE);
//    void gl_texture_multisample_storage_2d(GLsizei samples = 4, GLboolean fixedsamplelocations = GL_TRUE);
//    void gl_texture_multisample_storage_3d(GLsizei samples = 4, GLboolean fixedsamplelocations = GL_TRUE);

    void gl_texture_sub_image   (void *data, GLint level, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset);
    void gl_texture_sub_image_2d(void *data, GLint level, GLsizei width, GLsizei height, GLint xOffset, GLint yOffset);
    void gl_texture_sub_image_3d(void *data, GLint level, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset);

    TextureMode m_mode;
};
}


