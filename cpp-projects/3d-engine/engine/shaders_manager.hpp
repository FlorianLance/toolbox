
/*******************************************************************************
** Toolbox-3d-engine                                                          **
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

// opengl-utility
#include "opengl/shader/shader.hpp"


namespace tool::graphics {

class ShadersManager{

public:

    ~ShadersManager(){
        for(auto &shader : shaders){
            shader.second->clean();
        }
    }

    bool add_shader(const std::string &alias, const std::vector<std::string> &paths);
    bool add_shader(const std::string &alias, gl::ShaderProgram&& shader);
    gl::ShaderProgram* reload_shader(gl::ShaderProgram *shader);


    std::weak_ptr<gl::ShaderProgram> get(std::string alias);
    gl::ShaderProgram* get_ptr(std::string alias);

    void unbind();

private:

    std::unordered_map<std::string, std::shared_ptr<gl::ShaderProgram>> shaders;
};

}
