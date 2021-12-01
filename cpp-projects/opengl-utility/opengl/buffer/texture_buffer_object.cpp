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
#include "texture_buffer_object.hpp"


// base
#include "geometry/point4.hpp"
#include "utility/vector.hpp"

using namespace tool;
using namespace tool::gl;

void TBO::generate(){

    if(m_id != 0){
        std::cerr << "TBO already generated: " << m_id << "\n";
        return;
    }
    glCreateTextures(to_gl(m_mode), 1, &m_id);
}

void TBO::clean(){

    if(m_id == 0){
        return;
    }
    glDeleteBuffers(1, &m_id);

    m_id = 0;
}

bool TBO::bind(){
    return bind_textures({m_id});
}

void TBO::bind_image(GLuint unit, GLint level, GLboolean layered, GLint layer, GLenum access){

    glBindImageTexture(
        unit, // GLuint unit,
        m_id, // GLuint texture,
        level, // GLint level,
        layered, // GLboolean layered,
        layer,   // GLint layer,
        access,  // GLenum access,
        m_internalFormat // GLenum format
    );
//    return bind_image_textures({m_id});
}


void TBO::unbind_textures(GLuint first, GLuint count){
    std_v1<TextureName> textures(count);
    std::fill(std::begin(textures), std::end(textures), 0);
    glBindTextures(first, static_cast<GLsizei>(textures.size()), textures.data());
}

bool TBO::bind_textures(std_v1<TextureName> textures, GLuint first){

    for(const auto &n : textures){
        if(n==0){
            std::cerr << "TBO not generated, cannot bind it.\n";
            return false;
        }
    }

    // bind one or more named textures to a sequence of consecutive texture units
    glBindTextures(first, static_cast<GLsizei>(textures.size()), textures.data());
    return true;
}

bool TBO::bind_image_textures(std_v1<TextureName> textures, GLuint first){

    for(const auto &n : textures){
        if(n==0){
            std::cerr << "TBO not generated, cannot bind it.\n";
            return false;
        }
    }

    // bind one or more named texture images to a sequence of consecutive image units
    glBindImageTextures(first, static_cast<GLsizei>(textures.size()), textures.data());
    return true;
}

void TBO::set_texture_options(TextureOptions options){
    glTextureParameteri(m_id, GL_TEXTURE_WRAP_S,     to_gl(options.wrapS));
    glTextureParameteri(m_id, GL_TEXTURE_WRAP_T,     to_gl(options.wrapT));
    glTextureParameteri(m_id, GL_TEXTURE_WRAP_R,     to_gl(options.wrapR));
    glTextureParameteri(m_id, GL_TEXTURE_MIN_FILTER, to_gl(options.minFilter));
    glTextureParameteri(m_id, GL_TEXTURE_MAG_FILTER, to_gl(options.magFilter));
    glTextureParameteri(m_id, GL_TEXTURE_MAX_LEVEL,  options.maxLevel);
    glTextureParameterfv(m_id, GL_TEXTURE_BORDER_COLOR, options.borderColor.v.data());
}

void TBO::init_data_unsigned_8_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels){

    m_width          = width;
    m_height         = height;
    m_depth          = depth;
    m_type           = GL_UNSIGNED_BYTE;
    m_levelsNb       = 1;

    if(nbChannels == 3){
        m_format = GL_RGB;//_INTEGER;
        m_internalFormat = GL_RGB8;
    }else if(nbChannels == 4){
        m_format = GL_RGBA;//_INTEGER;
        m_internalFormat = GL_RGBA8;
    }else if(nbChannels == 1){
        m_format = GL_R;//ED_INTEGER;
        m_internalFormat = GL_R8;
    }else if(nbChannels == 2){
        m_format = GL_RG;//_INTEGER;
        m_internalFormat = GL_RG8;
    }else{
        std::cerr << "TBO::Channels nb not managed.\n";
        return;
    }
    gl_texture_storage();
}

void TBO::init_data_unsigned_32_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels){

    m_width          = width;
    m_height         = height;
    m_depth          = depth;
    m_type           = GL_UNSIGNED_INT;
    m_levelsNb       = 1;

    if(nbChannels == 3){
        m_format =  GL_RGB_INTEGER;
        m_internalFormat = GL_RGB32UI;
    }else if(nbChannels == 4){
        m_format =  GL_RGBA_INTEGER;
        m_internalFormat = GL_RGBA32UI;
    }else if(nbChannels == 1){
        m_format = GL_RED_INTEGER;
        m_internalFormat = GL_R32UI;
    }else if(nbChannels == 2){
        m_format =  GL_RG_INTEGER;
        m_internalFormat = GL_RG32UI;
    }else{
        std::cerr << "TBO::Channels nb not managed.\n";
        return;
    }

    gl_texture_storage();
}

void TBO::init_data_float_16_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels){

    m_width          = width;
    m_height         = height;
    m_depth          = depth;
    m_type           = GL_FLOAT;
    m_levelsNb       = 1;

    if(nbChannels == 3){
        m_format = GL_RGB;
        m_internalFormat = GL_RGB16F;
    }else if(nbChannels == 4){
        m_format = GL_RGBA;
        m_internalFormat = GL_RGBA16F;
    }else if(nbChannels == 1){
        m_format = GL_R;
        m_internalFormat = GL_R16F;
    }else if(nbChannels == 2){
        m_format = GL_RG;
        m_internalFormat = GL_RG16F;
    }else{
        std::cerr << "TBO::Channels nb not managed.\n";
        return;
    }
    gl_texture_storage();
}

void TBO::init_data_float_32_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels){

    m_width          = width;
    m_height         = height;
    m_depth          = depth;
    m_type           = GL_FLOAT;
    m_levelsNb       = 1;

    if(nbChannels == 3){
        m_format = GL_RGB;
        m_internalFormat = GL_RGB32F;
    }else if(nbChannels == 4){
        m_format = GL_RGBA;
        m_internalFormat = GL_RGBA32F;
    }else if(nbChannels == 1){
        m_format = GL_R;
        m_internalFormat = GL_R32F;
    }else if(nbChannels == 2){
        m_format = GL_RG;
        m_internalFormat = GL_RG32F;
    }else{
        std::cerr << "TBO::Channels nb not managed.\n";
        return;
    }
    gl_texture_storage();
}

void TBO::init_multisample_data_unsigned_8_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels, GLsizei samples){

    m_width          = width;
    m_height         = height;
    m_depth          = depth;
    m_type           = GL_UNSIGNED_BYTE;
    m_levelsNb       = 1;

    if(nbChannels == 3){
        m_format = GL_RGB;
        m_internalFormat = GL_RGB8;
    }else if(nbChannels == 4){
        m_format = GL_RGBA;
        m_internalFormat = GL_RGBA8;
    }else if(nbChannels == 1){
        m_format = GL_R;
        m_internalFormat = GL_R8;
    }else if(nbChannels == 2){
        m_format = GL_RG;
        m_internalFormat = GL_RG8;
    }else{
        std::cerr << "TBO::Channels nb not managed.\n";
        return;
    }

    glTextureStorage2DMultisample(
        m_id,               // GLuint texture,
        samples,            // GLsizei samples,
        m_internalFormat,   // GLenum internalformat,
        m_width,            // GLsizei width,
        m_height,           // GLsizei height,
        GL_TRUE             // GLboolean fixedsamplelocations
    );
}

void TBO::init_multisample_data_float_16_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels, GLsizei samples){

    m_width          = width;
    m_height         = height;
    m_depth          = depth;
    m_type           = GL_FLOAT;
    m_levelsNb       = 1;

    if(nbChannels == 3){
        m_format = GL_RGB;
        m_internalFormat = GL_RGB16F;
    }else if(nbChannels == 4){
        m_format = GL_RGBA;
        m_internalFormat = GL_RGBA16F;
    }else if(nbChannels == 1){
        m_format = GL_R;
        m_internalFormat = GL_R16F;
    }else if(nbChannels == 2){
        m_format = GL_RG;
        m_internalFormat = GL_RG16F;
    }else{
        std::cerr << "TBO::Channels nb not managed.\n";
        return;
    }

    glTextureStorage2DMultisample(
        m_id,               // GLuint texture,
        samples,            // GLsizei samples,
        m_internalFormat,   // GLenum internalformat,
        m_width,            // GLsizei width,
        m_height,           // GLsizei height,
        GL_TRUE             // GLboolean fixedsamplelocations
    );
}

void TBO::init_multisample_data_float_32_bits(GLsizei width, GLsizei height, GLsizei depth, int nbChannels, GLsizei samples){

    m_width          = width;
    m_height         = height;
    m_depth          = depth;
    m_type           = GL_FLOAT;
    m_levelsNb       = 1;

    if(nbChannels == 3){
        m_format = GL_RGB;
        m_internalFormat = GL_RGB32F;
    }else if(nbChannels == 4){
        m_format = GL_RGBA;
        m_internalFormat = GL_RGBA32F;
    }else if(nbChannels == 1){
        m_format = GL_R;
        m_internalFormat = GL_R32F;
    }else if(nbChannels == 2){
        m_format = GL_RG;
        m_internalFormat = GL_RG32F;
    }else{
        std::cerr << "TBO::Channels nb not managed.\n";
        return;
    }

    glTextureStorage2DMultisample(
        m_id,               // GLuint texture,
        samples,            // GLsizei samples,
        m_internalFormat,   // GLenum internalformat,
        m_width,            // GLsizei width,
        m_height,           // GLsizei height,
        GL_TRUE             // GLboolean fixedsamplelocations
    );
}

void TBO::load_data_unsigned_8_bits(GLubyte *data, GLsizei width, GLsizei height, GLsizei depth, int nbChannels){
    init_data_unsigned_8_bits(width, height, depth, nbChannels);    
    GLint level = 0;
    gl_texture_sub_image(data, level, m_width, m_height, m_depth, 0, 0, 0);
}

void TBO::load_data_float_16_bits(GLfloat *data, GLsizei width, GLsizei height, GLsizei depth, int nbChannels){
    init_data_float_16_bits(width, height, depth, nbChannels);
    GLint level = 0;
    gl_texture_sub_image(data, level, m_width, m_height, m_depth, 0, 0, 0);
}

void TBO::load_data_float_32_bits(GLfloat *data, GLsizei width, GLsizei height, GLsizei depth, int nbChannels){
    init_data_float_32_bits(width, height, depth, nbChannels);    
    GLint level = 0;
    gl_texture_sub_image(data, level, m_width, m_height, m_depth, 0, 0, 0);
}

void TBO::update_data_unsigned_8_bits(GLubyte *data, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset){
    GLint level = 0;
    gl_texture_sub_image(data, level, width, height, depth, xOffset, yOffset, zOffset);
}

void TBO::update_data_unsigned_32_bits(GLuint *data, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset){
    GLint level = 0;
    gl_texture_sub_image(data, level, width, height, depth, xOffset, yOffset, zOffset);
}

void TBO::update_data_float_32_bits(GLfloat *data, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset){
    GLint level = 0;
    gl_texture_sub_image(data, level, width, height, depth, xOffset, yOffset, zOffset);
}


void TBO::get_hdr_texture_data(std::vector<GLfloat> &data){

    GLsizei size = m_width * m_height * m_depth * m_nbChannels;
    if(data.size() != static_cast<size_t>(size)){
        data.resize(size);
    }

    glGetTextureImage(
        id(),
        0,
        m_format,
        m_type,
        size,
        data.data()
    );

    return;

    // retrieve a sub-region of a texture image from a texture object
    glGetTextureSubImage(
        id(),       // GLuint texture, Specifies the name of the source texture object.
                    //      Must be GL_TEXTURE_1D, GL_TEXTURE_1D_ARRAY, GL_TEXTURE_2D, GL_TEXTURE_2D_ARRAY, GL_TEXTURE_3D, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_ARRAY
                    //      or GL_TEXTURE_RECTANGLE. In specific, buffer and multisample textures are not permitted.
        0,          // GLint level: Specifies the level-of-detail number. Level 0 is the base image level. Level n is the nth mipmap reduction image.
        0,          // GLint xoffset: Specifies a texel offset in the x direction within the texture array.
        0,          // GLint yoffset: Specifies a texel offset in the y direction within the texture array.
        0,          // GLint zoffset: Specifies a texel offset in the z direction within the texture array.
        m_width,    // GLsizei width: Specifies the width of the texture subimage.
        m_height,   // GLsizei height: Specifies the height of the texture subimage.
        m_depth,    // GLsizei depth: Specifies the depth of the texture subimage.
        m_format,   // GLenum  format: Specifies the format of the pixel data.
                    //      The following symbolic values are accepted: GL_RED, GL_RG, GL_RGB, GL_BGR, GL_RGBA, GL_BGRA, GL_DEPTH_COMPONENT and GL_STENCIL_INDEX.
        m_type,     // GLenum type: Specifies the data type of the pixel data. The following symbolic values are accepted:
                    //      GL_UNSIGNED_BYTE, GL_BYTE, GL_UNSIGNED_SHORT, GL_SHORT, GL_UNSIGNED_INT, GL_INT, GL_FLOAT, GL_UNSIGNED_BYTE_3_3_2,
                    //      GL_UNSIGNED_BYTE_2_3_3_REV, GL_UNSIGNED_SHORT_5_6_5, GL_UNSIGNED_SHORT_5_6_5_REV, GL_UNSIGNED_SHORT_4_4_4_4,
                    //      GL_UNSIGNED_SHORT_4_4_4_4_REV, GL_UNSIGNED_SHORT_5_5_5_1, GL_UNSIGNED_SHORT_1_5_5_5_REV, GL_UNSIGNED_INT_8_8_8_8,
                    //      GL_UNSIGNED_INT_8_8_8_8_REV, GL_UNSIGNED_INT_10_10_10_2, and GL_UNSIGNED_INT_2_10_10_10_REV.
        static_cast<GLsizei>(size), // GLsizei bufSize: Specifies the size of the buffer to receive the retrieved pixel data.
        data.data() // void *pixels:    Returns the texture subimage. Should be a pointer to an array of the type specified by type.
                );
}

void TBO::debug_generate(){
    generate();
}


void TBO::load_texture(tool::graphics::Texture2D *texture){

    const auto size = texture->size();
    if(texture->is_hdr()){
        load_data_float_32_bits(reinterpret_cast<float*>(texture->data()), size.x(), size.y(), 1, texture->nb_channels());
    }else{
        load_data_unsigned_8_bits(texture->data(), size.x(), size.y(), 1, texture->nb_channels());
    }

    // mipmap
    glGenerateTextureMipmap(m_id);
}


void TBO::load_texture(std_a1<graphics::Texture2D, 6> *textures){

    const auto size  = (*textures)[0].size();
    if((*textures)[0].is_hdr()){
        init_data_float_32_bits(size.x(), size.y(), 1,  (*textures)[0].nb_channels());
    }else{
        init_data_unsigned_8_bits(size.x(), size.y(), 1,  (*textures)[0].nb_channels());
    }

    for(GLint face = 0; face < 6; ++face){

        glTextureSubImage3D(
            m_id,                       // GLuint texture
            0,                          // GLint level
            0,                          // GLint xoffset
            0,                          // GLint yoffset
            face,                       // GLint zoffset,
            m_width,                    // GLsizei width
            m_height,                   // GLsizei height
            m_depth,                    // GLsizei depth,
            m_format,                   // GLenum format
            m_type,                     // GLenum type
            (*textures)[face].data()    // const void *pixels
        );
    }
}

void TBO::gl_texture_storage(){

    if(m_mode == TextureMode::texture_2d){
        gl_texture_storage_2d();
    }else if(m_mode ==  TextureMode::texture_3d){
        gl_texture_storage_3d();
    }else if(m_mode == TextureMode::cubemap){
        gl_texture_storage_2d();
    }else{
        // ...
        std::cerr << "Texture mode not managed.\n";
    }
}

void TBO::gl_texture_storage_2d(){

    glTextureStorage2D (
        m_id,               // GLuint texture
        m_levelsNb,         // GLsizei levels
        m_internalFormat,   // GLenum internalformat
        m_width,            // GLsizei width
        m_height            // GLsizei height
    );
}

void TBO::gl_texture_storage_3d(){

    glTextureStorage3D (
        m_id,               // GLuint texture
        m_levelsNb,         // GLsizei levels
        m_internalFormat,   // GLenum internalformat
        m_width,            // GLsizei width
        m_height,           // GLsizei height
        m_depth             // GLsizei depth
    );
}


void TBO::gl_texture_sub_image(void *data, GLint level, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset){

    if(m_mode == TextureMode::texture_2d){
        gl_texture_sub_image_2d(data, level, width, height, xOffset, yOffset);
    }else if(m_mode ==  TextureMode::texture_3d){
        gl_texture_sub_image_3d(data, level, width, height, depth, xOffset, yOffset, zOffset);
    }else if(m_mode == TextureMode::cubemap){
        gl_texture_sub_image_3d(data, level, width, height, depth, 0, 0, zOffset);
    }else{
        // ...
        std::cerr << "Texture mode not managed.\n";
    }
}

void TBO::gl_texture_sub_image_2d(void *data, GLint level, GLsizei width, GLsizei height, GLint xOffset, GLint yOffset){

    // If a non-zero named buffer object is bound to the GL_PIXEL_UNPACK_BUFFER target (see glBindBuffer)
    // while a texture image is specified, pixels is treated as a byte offset into the buffer object's data store.
    glTextureSubImage2D (
        m_id,               // GLuint texture
        level,              // GLint level
        xOffset,            // GLint xoffset
        yOffset,            // GLint yoffset
        width,              // GLsizei width
        height,             // GLsizei height
        m_format,           // GLenum format
        m_type,             // GLenum type
        data                // const void *pixels
    );
}

void TBO::gl_texture_sub_image_3d(void *data, GLint level, GLsizei width, GLsizei height, GLsizei depth, GLint xOffset, GLint yOffset, GLint zOffset){

    glTextureSubImage3D (
        m_id,               // GLuint texture
        level,              // GLint level
        xOffset,            // GLint xoffset
        yOffset,            // GLint yoffset
        zOffset,            // GLint zoffset
        width,              // GLsizei width
        height,             // GLsizei height
        depth,              // GLsizei depth
        m_format,           // GLenum format
        m_type,             // GLenum type
        data                // const void *pixels
    );
}


//void TBO::gl_texture_multisample_storage(GLsizei samples, GLboolean fixedsamplelocations){

//    if(m_mode == TextureMode::texture_2d_multisample){

//    }else if(m_mode ==  TextureMode::texture_3d){

//    }else if(m_mode == TextureMode::cubemap){

//    }else{
//        // ...
//        std::cerr << "Texture mode not managed.\n";
//    }
//}

//void TBO::gl_texture_multisample_storage_2d(GLsizei samples, GLboolean fixedsamplelocations){

//}

//void TBO::gl_texture_multisample_storage_3d(GLsizei samples, GLboolean fixedsamplelocations){

//}
