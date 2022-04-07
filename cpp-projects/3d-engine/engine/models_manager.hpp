
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


// base
#include "graphics/model.hpp"
// opengl-utility
#include "opengl/draw/drawer.hpp"

namespace tool::graphics {

class ModelsManager{

    using Alias = std::string;
    using Path  = std::string;

public:

    bool add_models(std::vector<std::pair<Alias,Path>> aliasPaths, bool forceReload = false);
    bool add_model(std::string alias, const std::string &path);

    std::weak_ptr<graphics::Model<float>> get_model(std::string alias);
    graphics::Model<float>* get_model_ptr(std::string alias);

private:

    std::unordered_map<Path, Alias> aliasPerPath;
    std::unordered_map<Alias, std::shared_ptr<graphics::Model<float>>> models;
};
}


