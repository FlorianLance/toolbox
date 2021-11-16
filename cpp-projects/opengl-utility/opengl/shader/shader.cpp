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
#include "shader.hpp"

// std
#include <iostream>
#include <fstream>
#include <sstream>
#include <type_traits>
#include <filesystem>
#include <format>

// base
#include "utility/logger.hpp"
#include "utility/string_utility.hpp"

using namespace tool;
using namespace tool::gl;
using namespace tool::geo;


std::unordered_map<std::string, Shader::Type> Shader::extensions = {
    {".vs",         Shader::Type::vertex},
    {".vert",       Shader::Type::vertex},
    {"_vert.glsl",  Shader::Type::vertex},
    {".vert.glsl",  Shader::Type::vertex},
    {".gs",         Shader::Type::geometry},
    {".geom",       Shader::Type::geometry},
    {".geom.glsl",  Shader::Type::geometry },
    {".tcs",        Shader::Type::tess_control},
    {".tcs.glsl",   Shader::Type::tess_control},
    {".tes",        Shader::Type::tess_eval},
    {".tes.glsl",   Shader::Type::tess_eval},
    {".fs",         Shader::Type::fragment},
    {".frag",       Shader::Type::fragment},
    {"_frag.glsl",  Shader::Type::fragment},
    {".frag.glsl",  Shader::Type::fragment},
    {".cs",         Shader::Type::compute},
    {".cs.glsl",    Shader::Type::compute}
};


void ShaderProgram::use() {
    if(m_id <= 0 || !m_linked){
        Logger::error("Shader has not been linked.\n");
        return;
    }
    glUseProgram(m_id);
}

void ShaderProgram::unbind(){glUseProgram(0);}

bool ShaderProgram::is_uniform_valid(const char *name) const{
    return uniforms.count(name) != 0;
}

std::optional<UniformLocation> ShaderProgram::get_uniform_location(const char *name) const{
    if(is_uniform_valid(name)){
        return uniforms.at(name).second;
    }
//    std::cerr << "Cannot find uniform location for: " << name << " uniform\n";
    return {};
}

bool ShaderProgram::check_uniform_type(const char *name, UniformType type) const{
    if(uniforms.at(name).first.v == type.v){
        return true;
    }
    Logger::error(std::format("Invalid value for {} uniform {} instead of {}.\n", name, get_name(uniforms.at(name).first.v), get_name(type.v)));
    return false;
}

bool ShaderProgram::set_uniform(const char *name, bool value) {
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::bool_t})){
            glProgramUniform1i(m_id, location.value().v, value);
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, int value){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::int_t})){            
            glProgramUniform1i(m_id, location.value().v, value);
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, Sampler2D value){

    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::sampler2d_t})){
            glProgramUniform1i(m_id, location.value().v, value.v);
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, Sampler2DShadow value){

    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::sampler2dShadow_t})){
            glProgramUniform1i(m_id, location.value().v, value.v);
            return true;
        }
    }
    return false;
}

//bool ShaderProgram::set_uniform(const char *name, glm::vec3 values){
//    if(auto location = get_uniform_location(name); location.has_value()){
//        if(check_uniform_type(name, UniformType{GlType::fvec3_t})){
//            glProgramUniform3fv(m_id, location.value().v, 1, &values[0]);
//            return true;
//        }
//    }
//    return false;
//}


bool ShaderProgram::set_uniform(const char *name, float value){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::float_t})){
            glProgramUniform1f(m_id, location.value().v, value);
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, unsigned int value){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::uint_t})){
            glProgramUniform1ui(m_id, location.value().v, value);
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, const std_v1<float> values){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::float_t})){
            glProgramUniform1fv(m_id, location.value().v, static_cast<GLsizei>(values.size()), values.data());
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, const std_v1<Vec3f> values){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fvec3_t})){
            glProgramUniform3fv(m_id, location.value().v, static_cast<GLsizei>(values.size()), values.data()->v.data());
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, Vec2<float> values){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fvec2_t})){
            glProgramUniform2f(m_id, location.value().v, values.x(), values.y());
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, Vec3<float> values){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fvec3_t})){
            glProgramUniform3f(m_id, location.value().v, values.x(), values.y(), values.z());
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, Vec4<float> values){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fvec4_t})){
            glProgramUniform4f(m_id, location.value().v, values.x(), values.y(), values.z(), values.w());
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, Mat3<float> values, bool transpose){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fmat3x3_t})){
            glProgramUniformMatrix3fv(m_id, location.value().v, 1, transpose ? GL_TRUE : GL_FALSE, values.array.data());
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, Mat4<float> values, bool transpose){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fmat4x4_t})){
            glProgramUniformMatrix4fv(m_id, location.value().v, 1, transpose ? GL_TRUE : GL_FALSE, values.array.data());            
            return true;
        }
    }
    return false;
}

bool ShaderProgram::set_uniform(const char *name, float *values, bool transpose){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fmat4x4_t})){
            glProgramUniformMatrix4fv(m_id, location.value().v, 1, transpose ? GL_TRUE : GL_FALSE, values);
            return true;
        }
    }
    return false;
}

//bool ShaderProgram::set_uniform(const char *name, glm::mat4 values, bool transpose){
//    if(auto location = get_uniform_location(name); location.has_value()){
//        if(check_uniform_type(name, UniformType{GlType::fmat4x4_t})){
//            glProgramUniformMatrix4fv(m_id, location.value().v, 1, transpose ? GL_TRUE : GL_FALSE, &values[0][0]);
//            return true;
//        }
//    }
//    return false;
//}

bool ShaderProgram::set_uniform(const char *name, const std_v1<Mat4f> &values, bool transpose){
    if(auto location = get_uniform_location(name); location.has_value()){
        if(check_uniform_type(name, UniformType{GlType::fmat4x4_t})){
            glProgramUniformMatrix4fv(m_id, location.value().v, static_cast<GLsizei>(values.size()), transpose ? GL_TRUE : GL_FALSE, values.data()->array.data());
            return true;
        }
    }
    return false;
}

void ShaderProgram::set_model_matrix(const Mat4d &model){
    set_uniform(MM,  model.conv<float>());
}

void ShaderProgram::set_camera_matrices_uniforms(const graphics::CameraMatrices &camM){
    set_uniform(MVM, camM.mv.conv<float>());
    set_uniform(NM,  camM.normal.conv<float>());
    set_uniform(MVP, camM.mvp.conv<float>());
    set_model_matrix(camM.m);
}

void ShaderProgram::debug_display(){

    Logger::message("[shader files]:\n");
    for(const auto& file : m_loadedShadersFileNames){
        Logger::message(std::format("  -{}\n", file));
    }
    Logger::message("[attribs]:\n");
    for(const auto& attrib : attribs){
        Logger::message(std::format("  -{}\n", attrib));
    }
    Logger::message("[uniforms]:\n");
    for(const auto& uniform : uniforms){
        Logger::message(std::format("  -[{}][{}]\n", uniform.first, get_name(uniform.second.first.v)));
    }
    Logger::message("[uniforms blocks]:\n");
    for(const auto& uniformBlock : uniformBlocks){
        Logger::message(std::format("  -[{}][size: {}][loc: {}]\n", uniformBlock.first, uniformBlock.second.size.v, uniformBlock.second.location.v));
        for(const auto &element : uniformBlock.second.elements){
            Logger::message(std::format("    -[name: {}][index: {}][offset: {}]\n", element.first, element.second.index, element.second.offset));
        }
    }
}

std::string ShaderProgram::loaded_files_names_to_str() const{

    if(m_loadedShadersFileNames.size() == 0){
        return "[]";
    }

    std::string filesStr = "[";

    for (const auto &file : m_loadedShadersFileNames){
        filesStr += file + ";";
    }
    filesStr[filesStr.size()-1] = ']';
    return filesStr;
}

bool ShaderProgram::link(){

    if(m_linked){
        return true;
    }

    if(m_id <= 0){
        Logger::error("Program has not been compiled.\n");
        return false;
    }

    glLinkProgram(m_id);

    int status = 0;
    glGetProgramiv(m_id, GL_LINK_STATUS, &status);

    if(status == GL_FALSE) {
        glGetProgramInfoLog(m_id, 512, nullptr, m_infoLog);
        Logger::error(std::format("Program link failed: {}\n", m_infoLog));
        return m_linked = false;
    }

    find_uniforms_location();

    return m_linked = true;
}

void ShaderProgram::find_uniforms_location(){

    // clean
    attribs.clear();
    uniforms.clear();
    uniformBlocks.clear();    

    // get active vertex input attributes
    GLint numAttribs;
    glGetProgramInterfaceiv(m_id, GL_PROGRAM_INPUT, GL_ACTIVE_RESOURCES, &numAttribs);
    GLenum propertiesAttribs[] = {GL_NAME_LENGTH, GL_TYPE, GL_LOCATION};
    for(GLuint ii = 0; ii < static_cast<GLuint>(numAttribs); ++ii){

        // get properties
        GLint results[3];
        glGetProgramResourceiv(m_id, GL_PROGRAM_INPUT, ii, 3, propertiesAttribs, 3, nullptr, results);

        // get name
        std_v1<GLchar> nameArray(static_cast<size_t>(results[0]));
        glGetProgramResourceName(m_id, GL_PROGRAM_INPUT, ii, static_cast<GLint>(nameArray.size()), nullptr, nameArray.data());
        attribs.emplace_back(std::string(std::begin(nameArray), std::end(nameArray) - 1));

        // GL_NAME_LENGTH                          Any except GL_ATOMIC_COUNTER_BUFFER and GL_TRANSFORM_FEEDBACK_BUFFER
        // GL_TYPE                                 GL_UNIFORM, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT, GL_TRANSFORM_FEEDBACK_VARYING, GL_BUFFER_VARIABLE
        // GL_ARRAY_SIZE                           GL_UNIFORM, GL_BUFFER_VARIABLE, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT, VERTEX_SUBROUTINE_UNIFORM, GL_TESS_CONTROL_SUBROUTINE_UNIFORM, GL_TESS_EVALUATION_SUBROUTINE_UNIFORM, GL_GEOMETRY_SUBROUTINE_UNIFORM, GL_FRAGMENT_SUBROUTINE_UNIFORM, GL_COMPUTE_SUBROUTINE_UNIFORM, GL_TRANSFORM_FEEDBACK_VARYING                                                                                                                                                                                                                                                                      GL_OFFSET 	GL_UNIFORM, GL_BUFFER_VARIABLE, GL_TRANSFORM_FEEDBACK_VARYING
        // GL_BLOCK_INDEX                          GL_UNIFORM, GL_BUFFER_VARIABLE
        // GL_ARRAY_STRIDE                         GL_UNIFORM, GL_BUFFER_VARIABLE
        // GL_MATRIX_STRIDE                        GL_UNIFORM, GL_BUFFER_VARIABLE
        // GL_IS_ROW_MAJOR                         GL_UNIFORM, GL_BUFFER_VARIABLE
        // GL_ATOMIC_COUNTER_BUFFER_INDEX          GL_UNIFORM
        // GL_TEXTURE_BUFFER                       none
        // GL_BUFFER_BINDING                       GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_TRANSFORM_FEEDBACK_BUFFER
        // GL_BUFFER_DATA_SIZE                     GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_BUFFER, GL_SHADER_STORAGE_BLOCK
        // GL_NUM_ACTIVE_VARIABLES                 GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_TRANSFORM_FEEDBACK_BUFFER
        // GL_ACTIVE_VARIABLES                     GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_TRANSFORM_FEEDBACK_BUFFER
        // GL_REFERENCED_BY_VERTEX_SHADER          GL_UNIFORM, GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_SHADER, GL_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_BUFFER_VARIABLE, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_REFERENCED_BY_TESS_CONTROL_SHADER    GL_UNIFORM, GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_SHADER, GL_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_BUFFER_VARIABLE, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_REFERENCED_BY_TESS_EVALUATION_SHADER GL_UNIFORM, GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_SHADER, GL_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_BUFFER_VARIABLE, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_REFERENCED_BY_GEOMETRY_SHADER        GL_UNIFORM, GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_SHADER, GL_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_BUFFER_VARIABLE, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_REFERENCED_BY_FRAGMENT_SHADER        GL_UNIFORM, GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_SHADER, GL_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_BUFFER_VARIABLE, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_REFERENCED_BY_COMPUTE_SHADER         GL_UNIFORM, GL_UNIFORM_BLOCK, GL_ATOMIC_COUNTER_SHADER, GL_BUFFER, GL_SHADER_STORAGE_BLOCK, GL_BUFFER_VARIABLE, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_NUM_COMPATIBLE_SUBROUTINES           GL_VERTEX_SUBROUTINE_UNIFORM, GL_TESS_CONTROL_SUBROUTINE_UNIFORM, GL_TESS_EVALUATION_SUBROUTINE_UNIFORM, GL_GEOMETRY_SUBROUTINE_UNIFORM, GL_FRAGMENT_SUBROUTINE_UNIFORM, GL_COMPUTE_SUBROUTINE_UNIFORM
        // GL_COMPATIBLE_SUBROUTINES               GL_VERTEX_SUBROUTINE_UNIFORM, GL_TESS_CONTROL_SUBROUTINE_UNIFORM, GL_TESS_EVALUATION_SUBROUTINE_UNIFORM, GL_GEOMETRY_SUBROUTINE_UNIFORM, GL_FRAGMENT_SUBROUTINE_UNIFORM, GL_COMPUTE_SUBROUTINE_UNIFORM
        // GL_TOP_LEVEL_ARRAY_SIZE                 GL_BUFFER_VARIABLE
        // GL_TOP_LEVEL_ARRAY_STRIDE               GL_BUFFER_VARIABLE
        // GL_LOCATION                             GL_UNIFORM, GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT, GL_VERTEX_SUBROUTINE_UNIFORM, GL_TESS_CONTROL_SUBROUTINE_UNIFORM, GL_TESS_EVALUATION_SUBROUTINE_UNIFORM, GL_GEOMETRY_SUBROUTINE_UNIFORM, GL_FRAGMENT_SUBROUTINE_UNIFORM, GL_COMPUTE_SUBROUTINE_UNIFORM
        // GL_LOCATION_INDEX                       GL_PROGRAM_OUTPUT
        // GL_IS_PER_PATCH                         GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_LOCATION_COMPONENT                   GL_PROGRAM_INPUT, GL_PROGRAM_OUTPUT
        // GL_TRANSFORM_FEEDBACK_BUFFER_INDEX      GL_TRANSFORM_FEEDBACK_VARYING
        // GL_TRANSFORM_FEEDBACK_BUFFER_STRIDE     GL_TRANSFORM_FEEDBACK_BUFFER

        // results[0] NAME_LENGTH
        // results[1] TYPE
        // results[2] LOCATION
    }

    // get active uniform variables
    GLint numUniforms;
    glGetProgramInterfaceiv(m_id, GL_UNIFORM, GL_ACTIVE_RESOURCES, &numUniforms);
    GLenum propertiesUniforms[] = {GL_NAME_LENGTH, GL_TYPE, GL_LOCATION, GL_BLOCK_INDEX, GL_OFFSET, GL_ARRAY_STRIDE};

    // std::cout << "NUM UNIFORMS " << numUniforms << "\n";
    for(GLuint ii = 0; ii < static_cast<GLuint>(numUniforms); ++ii){

        // get properties
        GLint results[6];
        glGetProgramResourceiv(m_id, GL_UNIFORM, ii, 4, propertiesUniforms, 4, nullptr, results);

        const auto nameLength = results[0];
        const auto type       = results[1];
        const auto location   = results[2];
        const auto blockIndex = results[3];
        // const auto offset       = results[4];
        // const auto arrayStride  = results[5];
        // std::cout << "offset " << offset << " arrayStride " << arrayStride << "\n";

        // get name
        std_v1<GLchar> nameArray(static_cast<size_t>(nameLength));
        glGetProgramResourceName(m_id, GL_UNIFORM, ii, static_cast<GLint>(nameArray.size()), nullptr, nameArray.data());

        const std::string uniformName(&nameArray[0], nameArray.size() - 1);

        if(blockIndex != -1){ // uniform blocks

            auto split = str::split(uniformName, '.');
//            if(split.size() < 2){
//                std::cerr << "[GL] Error, invalid uniform block name " << uniformName << "\n";
//                return;
//            }

            if(split.size() == 2){

                const auto fullBlockName = split[0];
                // const auto elementName   = split[1];

                if(uniformBlocks.count(fullBlockName) == 0){

                    GLint blockSize;
                    glGetActiveUniformBlockiv(m_id, static_cast<GLuint>(blockIndex), GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize);

                    UniformBlockInfo ubi;
                    ubi.size     = UniformBlockSize{blockSize};
                    ubi.location = UniformBlockLocation{static_cast<GLuint>(blockIndex)};
                    uniformBlocks[fullBlockName] = std::move(ubi);
                }

                GLuint indices[1];
                const GLchar *names[] = {uniformName.c_str()};
                glGetUniformIndices(m_id, 1, names, indices);

                GLint offset[1];
                glGetActiveUniformsiv(m_id, 1, indices, GL_UNIFORM_OFFSET, offset);
                uniformBlocks[fullBlockName].elements[uniformName] = UniformBlockElementInfo{indices[0], offset[0]};// ubei;
            }

        }else{ // uniforms
            uniforms[uniformName] = std::make_pair(UniformType{get_type(static_cast<GLenum>(type))},UniformLocation{location});
        }
    }

}

void ShaderProgram::clean(){

    if (m_id == 0){
        return;
    }

    detach_and_delete_shaders();

    // delete program
    glDeleteProgram(m_id);

    m_id = 0;
}

bool ShaderProgram::load_from_files(const std_v1<std::string> &shadersPaths){

    m_shadersFilePaths = shadersPaths;

    namespace fs = std::filesystem;

    std_v1<std::pair<Shader::Type, std::string>> shadersCode;

    m_loadedShadersFileNames.clear();

    // open files
    for(const auto &shaderPath : shadersPaths){

        const auto path = fs::path(shaderPath);

        // check if exists
        if(!fs::exists(path)){
            Logger::error(std::format("Shader path {} doesn't exists.\n", path.string()));
            return false;
        }

        // check extension
        auto ext = path.filename().string();
        if(auto id = ext.find("."); id != std::string::npos){
            ext.erase(ext.begin(), ext.begin() + id);
        }else{
            Logger::error("Invalid shader file name.\n");
            return false;
        }

        if(Shader::extensions.count(ext) == 0){
            Logger::error(std::format("Extension {} of shader file {} is not valid.\n", ext, path.string()));
            return false;
        }       

        // open file
        std::ifstream shaderFile(path, std::ios::in);
        if(!shaderFile.is_open()){
            Logger::error(std::format("Cannot open shader file {} \n", path.string()));
            return false;
        }

        m_loadedShadersFileNames.emplace_back(path.filename().string());

        // read file
        std::stringstream shaderStream;
        shaderStream << shaderFile.rdbuf();

        // close file
        shaderFile.close();

        // add to shader array
        shadersCode.emplace_back(std::make_pair(Shader::extensions[ext], shaderStream.str()));
        // Shader::get_name(Shader::extensions[ext])
    }


    return load_from_source_code(shadersCode);
}

bool ShaderProgram::load_from_source_code(const std_v1<std::pair<Shader::Type, std::string>> &shadersSourceCode){

    std_v1<GLuint> shaders;
    for(const auto &shaderSourceCode : shadersSourceCode){
        GLuint shader;
        if(!init_shader(shader, shaderSourceCode.first, shaderSourceCode.second.c_str())){
            Logger::error(std::format("Cannot load shader source code : {} of type: {}.\n", shaderSourceCode.second, Shader::get_name(shaderSourceCode.first)));
            return false;
        }
        shaders.emplace_back(shader);
    }

    return init_shader_program(shaders);
}

bool ShaderProgram::init_shader(GLuint &shader, Shader::Type shaderType, const char *sourceCode){

    // init buffers
    shader = glCreateShader(Shader::get_gl_type(shaderType));

    glShaderSource(shader, 1, &sourceCode, nullptr);
    glCompileShader(shader);

    int status = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

    if(status == GL_FALSE){
        glGetShaderInfoLog(shader, 512, nullptr, m_infoLog);
        Logger::error(std::format("Shader compilation failed: {} \n", m_infoLog));
        clean();
        return false;
    }

    return true;
}

bool ShaderProgram::init_shader_program(std_v1<GLuint> shaders){

    m_id = glCreateProgram();

    for(const auto shader : shaders){
        glAttachShader(m_id, shader);
    }    
    if(!link()){
        return false;
    }

    // clean shaders
    detach_and_delete_shaders();

    return true;
}

void ShaderProgram::detach_and_delete_shaders() {

    // Detach and delete the shader objects (if they are not already removed)
    GLint numShaders = 0;
    glGetProgramiv(m_id, GL_ATTACHED_SHADERS, &numShaders);
    std::vector<GLuint> shaderNames(static_cast<size_t>(numShaders));
    glGetAttachedShaders(m_id, numShaders, nullptr, shaderNames.data());
    for (const GLuint shader : shaderNames) {
        glDetachShader(m_id, shader);
        glDeleteShader(shader);
    }
}



//            std::cout << "fullBlockName " << fullBlockName << "\n";
//            std::cout << "elementName " << elementName << "\n";

//            if (fullBlockName.find('[') != std::string::npos){
//                auto fullBlockNameSplit = str::split(fullBlockName, '[');
//                fullBlockNameSplit[1].pop_back();

//                auto blockName      = fullBlockNameSplit[0];
//                auto blockElementId = std::stoi(fullBlockNameSplit[1]);

////                std::cout << "blockName " << blockName << "\n";
////                std::cout << "blockElementId " << blockElementId << "\n";

//                if(uniformBlocks.count(blockName) == 0){ // new block

//                    GLint blockSize;
//                    glGetActiveUniformBlockiv(m_id, static_cast<GLuint>(blockIndex), GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize);

//                    UniformBlockInfo blockInfo;
//                    blockInfo.size     = UniformBlockSize{blockSize};
//                    blockInfo.location = UniformBlockLocation{static_cast<GLuint>(blockIndex)};
//                    uniformBlocks[blockName] = std::move(blockInfo);
//                }

//                if(static_cast<size_t>(blockElementId) >= uniformBlocks[blockName].elements2.size()){
//                     uniformBlocks[blockName].elements2.emplace_back(std::unordered_map<std::string, UniformBlockElementInfo>());
//                }

//                GLuint indices[1];
//                const GLchar *names[] = {uniformName.c_str()};
//                glGetUniformIndices(m_id, 1, names, indices);

//                GLint offset[1];
//                glGetActiveUniformsiv(m_id, 1, indices, GL_UNIFORM_OFFSET, offset);

//                uniformBlocks[blockName].elements2[blockElementId][elementName] = UniformBlockElementInfo{indices[0], offset[0]};

//            }else{

//            }

//            GLint blockSize;
//            glGetActiveUniformBlockiv(m_id, static_cast<GLuint>(blockIndex), GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize);

//            GLuint indices[1];
//            const GLchar *names[] = {uniformName.c_str()};
//            glGetUniformIndices(m_id, 1, names, indices);

//            GLint offset[1];
//            glGetActiveUniformsiv(m_id, 1, indices, GL_UNIFORM_OFFSET, offset);

//            std::cout << "ELEMENT BLOCK " << fullBlockName<< " " << blockIndex << " " << blockSize << " " <<  indices[0] << " " <<offset[0] << "\n";



