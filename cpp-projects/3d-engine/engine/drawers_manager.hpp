
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

class DrawersManager{

public:

    using Alias = std::string;

    bool add_drawer(const Alias &alias, std::shared_ptr<gl::Drawer> drawer);;

    std::weak_ptr<gl::Drawer> get_drawer(const Alias &alias) const;
    gl::Drawer *get_drawer_ptr(const Alias &alias) const;
    gl::Drawer *get_drawer_ptr(size_t id) const;
    Alias get_alias(size_t id) const noexcept;
    size_t get_id(const Alias &alias) const;

    constexpr size_t count() const noexcept {return aliases.size();}

private:
    std::vector<Alias> aliases;
    std::unordered_map<Alias, std::tuple<size_t, std::shared_ptr<gl::Drawer>>> drawers;
};
}


