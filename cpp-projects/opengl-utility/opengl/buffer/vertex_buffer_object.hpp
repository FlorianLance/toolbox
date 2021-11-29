

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

namespace tool::gl {


struct VBO{

    VBO() = default;
    VBO(const VBO&) = delete;
    VBO& operator=(const VBO&) = delete;
    VBO(VBO&& other) = delete;
    VBO& operator=(VBO&& other) = delete;

    ~VBO();

    constexpr GLuint id()const{return m_id;};

    void generate();
    void load_data(FloatData data, SizeData size);
    void load_data(IntData data, SizeData size);
//    void load_dynamic_data(FloatData data, SizeData size);
    bool attrib(AttriIndex index, AttriSize size, AttriType type, Stride stride, AttribOffset offset = {nullptr});
    void clean();

private:

    GLuint m_id = 0;
};


}
