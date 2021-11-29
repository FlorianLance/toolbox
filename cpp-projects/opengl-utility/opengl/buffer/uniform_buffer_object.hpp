

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
#include "opengl/utility/gl_utility.hpp"
#include "opengl/shader/shader.hpp"

namespace tool::gl {

struct UBO{

    UBO() = default;
    UBO(const UBO&) = delete;
    UBO& operator=(const UBO&) = delete;
    UBO(UBO&& other) = default;
    UBO& operator=(UBO&& other) = default;

    ~UBO(){
        clean();
    }

    constexpr GLuint id() const{ return m_id;}
    constexpr GLsizeiptr size() const{ return m_size;}

    virtual void generate(){

        if(m_id != 0){
            std::cerr << "[GL] UBO already generated: " << m_id << "\n";
            return;
        }
        glCreateBuffers(1, &m_id);
    }

    void set_data_storage(GLsizeiptr sizeData, void *data = nullptr, GLenum usage = GL_DYNAMIC_STORAGE_BIT){

        glNamedBufferStorage(
            m_id,              // GLuint buffer,
            m_size = sizeData, // GLsizeiptr size,
            data,              // const void *data,
            usage              // GLbitfield flags
        );
    }

    void update_data(void *data, GLsizeiptr sizeData, GLintptr offset = 0){

        if(m_size == 0){
            std::cerr << "[GL] UBO no storage initialized.\n";
            return;
        }

        if(sizeData > m_size){
            std::cerr << "[GL] UBO size is bigger than storage.\n";
            return;
        }

        glNamedBufferSubData(
            m_id,       // GLuint buffer: Specifies the name of the buffer object for glNamedBufferSubData.
            offset,     // GLintptr offset: Specifies the offset into the buffer object's data store where data replacement will begin, measured in bytes.
            sizeData,   // GLsizeiptr size: Specifies the size in bytes of the data store region being replaced.
            data        // const void *data: pecifies a pointer to the new data that will be copied into the data store.
        );
    }

    void bind(GLuint ubIndex){

        // bind a range within a buffer object to an indexed buffer target
        glBindBufferRange(
            GL_UNIFORM_BUFFER,  // GLenum target
            ubIndex,            // GLuint index
            m_id,               // GLuint buffer
            0,                  // GLintptr offset
            m_size              // GLsizeiptr size
        );
    }

    static void bind_range(const std::vector<UBO> &UBOs, GLuint first){

        // to test
        std_v1<GLuint> buffers;
        buffers.reserve(UBOs.size());

        std_v1<GLintptr> offsets;
        offsets.reserve(UBOs.size());

        std_v1<GLintptr> sizes;
        sizes.reserve(UBOs.size());

        for(const auto &ubo : UBOs){
            buffers.emplace_back(ubo.id());
            offsets.emplace_back(0);
            sizes.emplace_back(ubo.size());
        }

        glBindBuffersRange(
            GL_UNIFORM_BUFFER,                  // GLenum target,
            first,                              // GLuint first,
            static_cast<GLsizei>(UBOs.size()),  // GLsizei count,
            buffers.data(),                     // const GLuint *buffers,
            offsets.data(),                     // const GLintptr *offsets,
            sizes.data()                        // const GLintptr *sizes);
        );
    }

    void clean(){

        if(m_id == 0){
            return;
        }

        glDeleteBuffers(1, &m_id);
        m_id = 0;
    }


    void set_data_space_from_shader(ShaderProgram *shader, GLenum usage = GL_DYNAMIC_STORAGE_BIT){

        // shared, packed, std140, and std430
        // https://www.khronos.org/opengl/wiki/Interface_Block_(GLSL)
        if(shader == nullptr){
            std::cerr << "[GL] MaterialUBO shader is null.\n";
            return;
        }

        const auto nameStr = get_block_name();
        if(shader->uniformBlocks.count(nameStr) == 0){
            std::cerr << "[GL] MaterialUBO block uniform not found in shader.\n";
            return;
        }

        const auto &blockInfo = shader->uniformBlocks.at(nameStr);
        blockBuffer.resize((blockInfo.size.v));
        offsets = blockInfo.get_offsets(get_elements_name());

        set_data_storage(static_cast<GLint>(blockBuffer.size()), nullptr, usage);
    }

    virtual std::string get_block_name() const{
        return "";
    }

    virtual std::vector<std::string> get_elements_name() const{
        return {};
    }


protected:

    GLuint m_id = 0;
    GLsizeiptr m_size = 0;

    std_v1<GLubyte> blockBuffer;
    std::vector<GLint> offsets;
};


}

//struct GlobalData {
//    mat4 projection;
//    mat4 view;
//    mat4 projectionView;
//    vec3 camPos;
//    vec3 lightPos;
//} globalData;
//glBindBuffer(GL_UNIFORM_BUFFER, ubo);
//glBufferData(GL_UNIFORM_BUFFER, sizeof(GlobalData), &globalData, GL_DYNAMIC_DRAW);
//glBindBuffer(GL_UNIFORM_BUFFER, 0);

//glUniformBlockBinding(shader.GetProgram(), glGetUniformBlockIndex(shader.GetProgram(), "GlobalData"), 0);

//layout (std140) uniform GlobalData {
//    mat4 projection;
//    mat4 view;
//    mat4 projectionView;
//    vec3 camPos;
//    vec3 lightPos;
//};


//struct GlobalData {
//    mat4 projection;
//    mat4 view;
//    mat4 projectionView;
//    alignas(sizeof(float)*4) vec3 camPos;
//    alignas(sizeof(float)*4) vec3 lightPos;
//} globalData;


//#define MAX_NUM_TOTAL_LIGHTS 100
//struct Light {
//    vec3 position;
//    float padding;
//}
//layout (std140) uniform Lights {
//    Light light[MAX_NUM_TOTAL_LIGHTS];
//    int numLights;
//}



//Sample Fragment Shader (UBOs require OpenGL 3.1):

//#version 140 // GL 3.1

//      // Arrays in a UBO must use a constant expression for their size.
//      const int MY_ARRAY_SIZE = 512;

//// The name of the block is used for finding the index location only
//layout (std140) uniform myArrayBlock {
//    int myArray [MY_ARRAY_SIZE]; // This is the important name (in the shader).
//};

//void main (void) {
//    gl_FragColor = vec4 ((float)myArray [0] * 0.1, vec3 (1.0));
//}


//OpenGL Code

//    const int MY_ARRAY_SIZE = 512;

//GLuint myArrayUBO;
//glGenBuffers (1, &myArrayUBO);

//// Allocate storage for the UBO
//glBindBuffer (GL_UNIFORM_BUFFER, myArrayUBO);
//glBufferData (GL_UNIFORM_BUFFER, sizeof (GLint) * MY_ARRAY_SIZE,
//             NULL, GL_DYNAMIC_DRAW);

//[...]

//    // When you want to update the data in your UBO, you do it like you would any
//    //   other buffer object.
//    glBufferSubData (GL_UNIFORM_BUFFER, ...);

//[...]

//    GLuint myArrayBlockIdx = glGetUniformBlockIndex (GLSLProgramID, "myArrayBlock");

//glUniformBlockBinding (GLSLProgramID,     myArrayBlockIdx, 0);
//glBindBufferBase      (GL_UNIFORM_BUFFER, 0,               myArrayUBO);

// https://computergraphics.stackexchange.com/questions/4454/opengl-es-3-uniform-buffer-object-with-float-array
// https://www.khronos.org/opengl/wiki/Interface_Block_(GLSL)
