

/*******************************************************************************
** Toolbox-base                                                               **
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
#include "geometry/point2.hpp"
#include "geometry/matrix4.hpp"

namespace tool::graphics{

class Screen{

public:

    Screen(unsigned int width = 800, unsigned int height = 800) : m_size(width,height){
    }    

    void resize(unsigned int width, unsigned int height){
        m_size = {width, height};
    }

    constexpr unsigned int width() const noexcept{
        return m_size.x();
    }

    constexpr unsigned int height() const noexcept{
        return m_size.y();
    }

    constexpr unsigned int size_pixels() const noexcept{
        return m_size.x() * m_size.y();
    }

    constexpr geo::Pt2<unsigned int> size() const noexcept{
        return m_size;
    }

private:
    geo::Pt2<unsigned int> m_size = {800,800};
};
}

