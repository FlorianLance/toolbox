
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
#include <fstream>
#include <ostream>
#include <vector>

namespace tool{

// write
template<typename T>
void write(std::ofstream &file, T value){
    file.write(reinterpret_cast<char*>(&value), sizeof(T));
}
template<typename T>
void write(std::ofstream &file, T *value){
    file.write(reinterpret_cast<char*>(value), sizeof(T));
}
template<typename T>
void write_array(std::ofstream &file, T *value, size_t size){
    if(size > 0 ){
        file.write(reinterpret_cast<char*>(value), sizeof(T)*size);
    }
}

// read
template<typename T>
T read(std::ifstream &file){
    T value;
    file.read(reinterpret_cast<char*>(&value), sizeof(T));
    return value;
}

template<typename T>
void read(std::ifstream &file, T *value){
    file.read(reinterpret_cast<char*>(value), sizeof(T));
}
template<typename T>
void read_array(std::ifstream &file, T *value, size_t size){
    if(size > 0){
        file.read(reinterpret_cast<char*>(value), sizeof(T)*size);
    }
}

template<typename T>
std::vector<T> read(std::ifstream &file, size_t size){
    std::vector<T> values(size);
    file.read(reinterpret_cast<char*>(values.data()), sizeof(T)*size);
    return values;
}


}
