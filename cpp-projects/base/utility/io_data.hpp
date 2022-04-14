
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

// std
#include <algorithm>

namespace tool{

    template<typename T>
    static void read(T &v, std::int8_t *data, size_t &offset){
        std::copy(
            data + offset,
            data + offset + sizeof(T),
            reinterpret_cast<std::int8_t*>(&v));
        offset += sizeof(T);
    }

    template<typename T>
    static void read_array(T *a, std::int8_t *data, size_t sizeArray, size_t &offset){
        auto nbBytes = sizeArray * sizeof(T);
        std::copy(
            data + offset,
            data + offset + nbBytes,
            reinterpret_cast<std::int8_t*>(a));
        offset += nbBytes;
    }

    template<typename T>
    static void write(const T &v, std::int8_t *data, size_t &offset){
        std::copy(
            reinterpret_cast<const std::int8_t*>(&v),
            reinterpret_cast<const std::int8_t*>(&v) + sizeof(T),
            data + offset);
        offset += sizeof(T);
    }

    template<typename T>
    static void write_array(T *a, std::int8_t *data, size_t sizeArray, size_t &offset){
        auto nbBytes = sizeArray * sizeof(T);
        std::copy(
            reinterpret_cast<std::int8_t*>(a),
            reinterpret_cast<std::int8_t*>(a) + nbBytes,
            data+ offset);
        offset += nbBytes;
    }
}
