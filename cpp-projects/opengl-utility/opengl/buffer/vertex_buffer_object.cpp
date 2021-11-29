
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

#include "vertex_buffer_object.hpp"


using namespace tool::gl;


VBO::~VBO(){
    clean();
}

void VBO::generate(){

    if(m_id != 0){
        std::cerr << "VBO already generated: " << m_id << "\n";
        return;
    }
    glCreateBuffers(1, &m_id);
}

//void VBO::load_dynamic_data(FloatData data, SizeData size){

//    // GL_STREAM_DRAW, GL_STREAM_READ, GL_STREAM_COPY, GL_STATIC_DRAW, GL_STATIC_READ, GL_STATIC_COPY, GL_DYNAMIC_DRAW, GL_DYNAMIC_READ, or GL_DYNAMIC_COPY.
//    GLenum usage = GL_DYNAMIC_DRAW;
//    glNamedBufferData(
//        m_id,   // GLuint buffer
//        size.v, // GLsizeiptr size
//        data.v, // const void *data
//        usage   // GLenum usage
//    );
//}

void VBO::load_data(IntData data, SizeData size){
    GLenum usage =  GL_DYNAMIC_STORAGE_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT;
    glNamedBufferStorage(
        m_id,   // GLuint buffer
        size.v, // GLsizeiptr size
        data.v, // const void *data
        usage   // GLenum usage
    );
}

void VBO::load_data(FloatData data, SizeData size){

    // bit combination of: GL_DYNAMIC_STORAGE_BIT, GL_MAP_READ_BIT GL_MAP_WRITE_BIT, GL_MAP_PERSISTENT_BIT, GL_MAP_COHERENT_BIT, and GL_CLIENT_STORAGE_BIT.

    // GL_DYNAMIC_STORAGE_BIT
    // The contents of the data store may be updated after creation through calls to glBufferSubData.
    // If this bit is not set, the buffer content may not be directly updated by the client.
    // The data argument may be used to specify the initial content of the buffer's data store regardless of the presence of the GL_DYNAMIC_STORAGE_BIT.
    // Regardless of the presence of this bit, buffers may always be updated with server-side calls such as glCopyBufferSubData and glClearBufferSubData.

    // GL_MAP_READ_BIT
    // The data store may be mapped by the client for read access and a pointer in the client's address space obtained that may be read from.

    // GL_MAP_WRITE_BIT
    // The data store may be mapped by the client for write access and a pointer in the client's address space obtained that may be written through.

    // GL_MAP_PERSISTENT_BIT
    // The client may request that the server read from or write to the buffer while it is mapped.
    // The client's pointer to the data store remains valid so long as the data store is mapped, even during execution of drawing or dispatch commands.

    // GL_MAP_COHERENT_BIT
    // Shared access to buffers that are simultaneously mapped for client access and are used by the server will be coherent,
    // so long as that mapping is performed using glMapBufferRange.
    // That is, data written to the store by either the client or server will be immediately visible
    // to the other with no further action taken by the application. In particular,
    //  * If GL_MAP_COHERENT_BIT is not set and the client performs a write followed by a call to the glMemoryBarrier command with the GL_CLIENT_MAPPED_BUFFER_BARRIER_BIT set,
    //    then in subsequent commands the server will see the writes.
    //  * If GL_MAP_COHERENT_BIT is set and the client performs a write, then in subsequent commands the server will see the writes.
    //  * If GL_MAP_COHERENT_BIT is not set and the server performs a write, the application must call glMemoryBarrier with the GL_CLIENT_MAPPED_BUFFER_BARRIER_BIT set
    //    and then call glFenceSync with GL_SYNC_GPU_COMMANDS_COMPLETE (or glFinish). Then the CPU will see the writes after the sync is complete.
    //  * If GL_MAP_COHERENT_BIT is set and the server does a write, the app must call glFenceSync with GL_SYNC_GPU_COMMANDS_COMPLETE (or glFinish).
    //    Then the CPU will see the writes after the sync is complete.

    // GL_CLIENT_STORAGE_BIT
    // When all other criteria for the buffer storage allocation are met, this bit may be used by an implementation
    // to determine whether to use storage that is local to the server or to the client to serve as the backing store for the buffer.

    GLenum usage = 0;// GL_DYNAMIC_STORAGE_BIT;
    glNamedBufferStorage(
        m_id,   // GLuint buffer
        size.v, // GLsizeiptr size
        data.v, // const void *data
        usage   // GLenum usage
    );
}


bool VBO::attrib(AttriIndex index, AttriSize size, AttriType type, Stride stride, AttribOffset offset){

    glBindBuffer(GL_ARRAY_BUFFER, m_id);
    glEnableVertexAttribArray(index.v);

    if(type.v == GL_INT){
        glVertexAttribIPointer(index.v, size.v, type.v, stride.v,  offset.v);
    }else{
        glVertexAttribPointer(index.v, size.v, type.v, GL_FALSE, stride.v,  offset.v);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return true;
}

//    void dsa_attrib(GLuint vao, AttriIndex index, AttriSize size, AttriType type, Stride stride, RelativeOffset offset){
//        static_cast<void>(stride);

////        void glVertexArrayVertexBuffer(
////            GLuint vaobj,
////            GLuint bindingindex,
////            GLuint buffer,
////            GLintptr offset,
////            GLsizei stride);

//        glVertexArrayVertexBuffer(vao, 0, m_id, 0, size.v*sizeof(float));

//        glEnableVertexArrayAttrib(vao, index.v);
//        glVertexArrayAttribFormat(vao, index.v, size.v, type.v, GL_FALSE, offset.v);
//        glVertexArrayAttribBinding(vao, index.v, 0);
//    }


void VBO::clean(){

    if(m_id == 0){
        return;
    }

    glDeleteBuffers(1, &m_id);
    m_id = 0;
}
