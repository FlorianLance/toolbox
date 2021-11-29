
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
#include <unordered_map>

// local
#include "glew_utility.hpp"

namespace tool::gl {

using TextureName = GLuint;

struct UintData{
    const GLuint *v;
};
struct IntData{
    const GLint *v;
};
struct FloatData{
    const GLfloat *v;
};
struct SizeData{
    const GLsizeiptr v;
};
struct DrawUsage{
    const GLenum v;
};
struct BufferUsage{
    // GL_DYNAMIC_STORAGE_BIT: The contents of the data store may be updated after creation through calls to glBufferSubData.
    // GL_MAP_READ_BIT: The data store may be mapped by the client for read access and a pointer in the client's address space obtained that may be read from.
    // GL_MAP_WRITE_BIT: The data store may be mapped by the client for write access and a pointer in the client's address space obtained that may be written through.
    // GL_MAP_PERSISTENT_BIT: The client may request that the server read from or write to the buffer while it is mapped.
    // The client's pointer to the data store remains valid so long as the data store is mapped, even during execution of drawing or dispatch commands.
    // GL_MAP_COHERENT_BIT:
    // GL_CLIENT_STORAGE_BIT:
    const GLenum v;
};


struct AttriIndex{
    const GLuint v;
};
struct AttriSize{
    const GLint v;
};
struct AttriType{
    const GLenum v;
};
struct Stride{
    const GLsizei v;
};
struct RelativeOffset{
    const GLuint v;
};

struct AttribOffset{
    const GLvoid *v;
};


struct DrawMode{
    const GLenum v;
};
struct VerticesCount{
    GLsizei v;
};


enum class GlType : std::uint8_t{    
    bool_t =0, bvec2_t, bvec3_t, bvec4_t,
    int_t, ivec2_t, ivec3_t, ivec4_t,
    uint_t, uivec2_t, uivec3_t, uivec4_t,
    float_t, fvec2_t, fvec3_t, fvec4_t,
    double_t, dvec2_t, dvec3_t, dvec4_t,
    fmat2x3_t, fmat2x4_t, fmat3x2_t, fmat3x4_t, fmat4x2_t, fma4x3_t, fmat2x2_t, fmat3x3_t, fmat4x4_t,
    dmat2x3_t, dmat2x4_t, dmat3x2_t, dmat3x4_t, dmat4x2_t, dma4x3_t, dmat2x2_t, dmat3x3_t, dmat4x4_t,
    sampler1d_t, sampler2d_t, sampler3d_t,
    samplerCube_t,
    sampler1dShadow_t, sampler2dShadow_t,
    sampler1dArray_t, sampler2dArray_t,
    sampler1dArrayShadow_t, sampler2dArrayShadow_t,
    sampler2dMS_t, sampler2dMSArray_t,samplerCubeShadow_t,
    samplerBuffer_t, sampler2dRect_t, sampler2dRectShadow_t,
    isampler1d_t, isampler2d_t, isampler3d_t, isamplerCube_t,
    isampler1dArray_t, isampler2dArray_t,
    isampler2dMS_t, isampler2dMSArray_t,
    isamplerBuffer_t, isampler2DRect_t,
    usampler1d_t, usampler2d_t, usampler3d_t,
    usamplerCube_t, usampler1dArray_t, usampler2dArray_t,
    usampler2dMS_t, usampler2dMSArray_t, usamplerBuffer,
    unknow, SizeEnum
};

static constexpr std::array<std::tuple<GlType, GLenum, const char*>,static_cast<size_t>(GlType::SizeEnum)> glTypeMapping ={{

    {GlType::bool_t,    GL_BOOL,        "bool"},
    {GlType::bvec2_t,   GL_BOOL_VEC2,   "bvec2"},
    {GlType::bvec3_t,   GL_BOOL_VEC3,   "bvec3"},
    {GlType::bvec4_t,   GL_BOOL_VEC4,   "bvec4"},

    {GlType::int_t,     GL_INT,        "int"},
    {GlType::ivec2_t,   GL_INT_VEC2,   "ivec2"},
    {GlType::ivec3_t,   GL_INT_VEC3,   "ivec3"},
    {GlType::ivec4_t,   GL_INT_VEC4,   "ivec4"},

    {GlType::uint_t,    GL_UNSIGNED_INT,        "uint"},
    {GlType::uivec2_t,  GL_UNSIGNED_INT_VEC2,   "uivec2"},
    {GlType::uivec3_t,  GL_UNSIGNED_INT_VEC3,   "uivec3"},
    {GlType::uivec4_t,  GL_UNSIGNED_INT_VEC4,   "uivec4"},

    {GlType::float_t,   GL_FLOAT,        "float"},
    {GlType::fvec2_t,   GL_FLOAT_VEC2,   "fvec2"},
    {GlType::fvec3_t,   GL_FLOAT_VEC3,   "fvec3"},
    {GlType::fvec4_t,   GL_FLOAT_VEC4,   "fvec4"},

    {GlType::double_t,  GL_FLOAT,        "double"},
    {GlType::dvec2_t,   GL_FLOAT_VEC2,   "dvec2"},
    {GlType::dvec3_t,   GL_FLOAT_VEC3,   "dvec3"},
    {GlType::dvec4_t,   GL_FLOAT_VEC4,   "dvec4"},

    {GlType::fmat2x2_t, GL_FLOAT_MAT2,   "fmat2x2"},
    {GlType::fmat3x3_t, GL_FLOAT_MAT3,   "fmat3x3"},
    {GlType::fmat4x4_t, GL_FLOAT_MAT4,   "fmat4x4"},

    {GlType::dmat2x2_t, GL_DOUBLE_MAT2,   "dmat2x2"},
    {GlType::dmat3x3_t, GL_DOUBLE_MAT3,   "dmat3x3"},
    {GlType::dmat4x4_t, GL_DOUBLE_MAT4,   "dmat4x4"},

    {GlType::sampler2d_t,       GL_SAMPLER_2D,   "sampler 2D"},
    {GlType::sampler2dShadow_t, GL_SAMPLER_2D_SHADOW,   "sampler 2D shadow"},
}};

[[maybe_unused]] static constexpr GlType get_type(GLenum t) {
    for (auto& m : glTypeMapping) {
        if (t == std::get<1>(m)) {
            return std::get<0>(m);
        }
    }
    return GlType::unknow;
}

[[maybe_unused]] static constexpr const char* get_name(GlType t) {
    for (auto& m : glTypeMapping) {
        if (t == std::get<0>(m)) {
            return std::get<2>(m);
        }
    }
    return "?";
}

[[maybe_unused]] static constexpr const char* get_name(GLenum t) {
    for (auto& m : glTypeMapping) {
        if (t == std::get<1>(m)) {
            return std::get<2>(m);
        }
    }
    return "?";
}

// uniform
struct UniformLocation{
    GLint v;
};
struct UniformType{
    GlType v;
};

// uniform block
struct UniformBlockLocation{
    GLuint v;
};
struct UniformBlockSize{
    GLint v;
};

struct UniformBlockElementInfo{
    GLuint index;
    GLint offset;
};


struct UniformBlockInfo{

    UniformBlockSize size;
    UniformBlockLocation location;
    UniformBlockElementInfo info;

    std::unordered_map<std::string,UniformBlockElementInfo> elements;

    GLint get_offset(const std::string &elementName) const{
        if(elements.count(elementName) == 0){
            std::cerr << "[GL] Error " << elementName << " not found in block uniform info.\n";
            return {};
        }
        return elements.at(elementName).offset;
    }

    std::vector<GLint> get_offsets(const std::vector<std::string> &elementsNames) const{

        std::vector<GLint> offsets;
        offsets.reserve(elementsNames.size());
        for(const auto &name : elementsNames){
            if(elements.count(name) == 0){
                std::cerr << "[GL] Error " << name << " not found in block uniform info.\n";
                return {};
            }
            offsets.emplace_back(elements.at(name).offset);
        }
        return offsets;
    }

};


// sampler
struct Sampler2D{
    int v;
};
struct Sampler2DShadow{
    int v;
};
//static std::vector<SBS> generate_buffers(size_t count){
//    std::vector<SBS> buffers(count);
//    glCreateBuffers(count, &buffers[0].m_id);
//    return buffers;
//}


}
