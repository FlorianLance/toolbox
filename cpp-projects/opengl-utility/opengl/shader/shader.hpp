

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
#include <string>
#include <memory>
#include <optional>

// glew
#include <GL/glew.h>

// glm
//#include <glm/glm.hpp>

// base
#include "utility/tuple_array.hpp"
#include "utility/vector_utility.hpp"
#include "geometry/point2.hpp"
#include "geometry/matrix4.hpp"
#include "graphics/camera.hpp"

// local
#include "opengl/utility/gl_utility.hpp"

namespace tool::gl {

class Shader{
public:

    enum class Type : std::uint8_t{
        vertex,fragment,geometry,tess_control,tess_eval,compute,
        SizeEnum
    };

    using Name = std::string_view;
    using TShader = std::tuple<Type,GLenum,Name>;
    static constexpr TupleArray<Type::SizeEnum, TShader> shaders = {{
        TShader{Type::vertex,          GL_VERTEX_SHADER,           "vertex"},
        TShader{Type::fragment,        GL_FRAGMENT_SHADER,         "fragment"},
        TShader{Type::geometry,        GL_GEOMETRY_SHADER,         "geometry"},
        TShader{Type::tess_control,    GL_TESS_CONTROL_SHADER,     "tess_control"},
        TShader{Type::tess_eval,       GL_TESS_EVALUATION_SHADER,  "tess_eval"},
        TShader{Type::compute,         GL_COMPUTE_SHADER,          "compute"},
    }};

    static constexpr GLenum get_gl_type(Type t) {
        return shaders.at<0,1>(t);
    }
    static constexpr std::string_view get_name(Type t) {
        return shaders.at<0,2>(t);
    }

    static std::unordered_map<std::string, Shader::Type> extensions;
};

class ShaderProgram {

public:

    ShaderProgram() = default;
    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&& other) = default;
    ShaderProgram& operator=(ShaderProgram&& other) = default;

    void clean();

    constexpr GLuint id() const{return m_id;}

    // input
    bool load_from_files(const std_v1<std::string> &shadersPaths);
    bool load_from_source_code(const std_v1<std::pair<Shader::Type, std::string>> &shadersSourceCode);

    constexpr bool is_linked() const {return m_linked;}
    constexpr unsigned int get_handle() const{ return m_id;}

    bool link();
    void find_uniforms_location();
    void use();
    static void unbind();

    // uniforms
    bool is_uniform_valid(const char* name) const;
    std::optional<UniformLocation> get_uniform_location(const char* name) const;
    bool check_uniform_type(const char *name, UniformType type) const;
    bool set_uniform(const char *name, bool value);
    bool set_uniform(const char *name, int value);
    bool set_uniform(const char *name, float value);
    bool set_uniform(const char *name, unsigned int value);
    bool set_uniform(const char *name, const std_v1<float> values);
    bool set_uniform(const char *name, const std_v1<geo::Vec3f> values);
    bool set_uniform(const char *name, Sampler2D value);
    bool set_uniform(const char *name, Sampler2DShadow value);
    bool set_uniform(const char *name, geo::Vec2f values);
    bool set_uniform(const char *name, geo::Vec3f values);
//    bool set_uniform(const char *name, glm::vec3 values);

    bool set_uniform(const char *name, geo::Vec4f values);
    bool set_uniform(const char *name, geo::Mat3f values, bool transpose=false);
    bool set_uniform(const char *name, geo::Mat4f values, bool transpose=false);
    bool set_uniform(const char *name, float *values, bool transpose=false);
    bool set_uniform(const char *name, const std_v1<geo::Mat4f> &values, bool transpose=false);
//    bool set_uniform(const char *name, glm::mat4 values, bool transpose=false);

    void set_model_matrix(const geo::Mat4d &model);
    void set_camera_matrices_uniforms(const graphics::CameraMatrices &cameraM);

    // uniforms
    std::vector<std::string> attribs;
    std::unordered_map<std::string, std::pair<UniformType, UniformLocation>> uniforms;
    std::unordered_map<std::string, UniformBlockInfo> uniformBlocks;

    void debug_display();

    std::string loaded_files_names_to_str() const;

private:

    void detach_and_delete_shaders();
    bool init_shader(GLuint &shader, Shader::Type shaderType, const char *sourceCode);
    bool init_shader_program(std_v1<GLuint> shaders);

    bool m_linked = false;
    GLuint m_id = 0;
    char m_infoLog[512];
    std_v1<std::string> m_loadedShadersFileNames;

    static constexpr const char* MVM = "ModelViewMatrix";
    static constexpr const char* MM  = "ModelMatrix";
    static constexpr const char* NM  = "NormalMatrix";
    static constexpr const char* MVP = "MVP";
};

}


