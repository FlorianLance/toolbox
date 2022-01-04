

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

#include "cloud_io.hpp"

// local
#include "utility/logger.hpp"

using namespace tool::files;

bool CloudIO::open_file(const std::string &path, std::ofstream &file){
    file.open(path);
    if(!file.is_open()){
        Logger::error(std::format("[CloudIO::save_cloud] Cannot open file from path {}.\n", path));
        return false;
    }
    return true;
}

bool CloudIO::check_bufers_sizes(size_t sizeVertices, size_t sizeColors){
    if(sizeVertices == 0){
        Logger::error("[CloudIO::save_cloud] No vertices.\n");
        return false;
    }
    if(sizeColors != 0 && (sizeVertices != sizeColors)){
        Logger::error("[CloudIO::save_cloud] Not the same number of vertices and colors\n");
        return false;
    }
    return true;
}

bool CloudIO::check_input_values(size_t size, bool hasVertices, bool hasColors, int dimension, int nbChannels){

    bool isValid = true;
    if(size == 0 || dimension < 2 || dimension > 3 || !hasVertices){
        isValid = false;
    }
    if(hasColors && (nbChannels < 3 || nbChannels > 4)){
        isValid = false;
    }

    if(!isValid){
        Logger::error("[CloudIO::save_cloud] Invalid inputs values.\n");
    }
    return isValid;
}

//void CloudIO::write_line(std::ostream &file, int16_t v1, int16_t v2, int16_t v3){
//    file << std::format(line3, v1, v2, v3);
//}

//void CloudIO::write_line(std::ostream &file, int16_t v1, int16_t v2, int16_t v3){
//    file << std::format(line3, v1, v2, v3);
//}

void CloudIO::write(std::ostream &file, float v1, float v2){
    file << std::format(line2, v1, v2);
}

void CloudIO::write(std::ostream &file, float v1, float v2, float v3){
    file << std::format(line3, v1, v2, v3);
}

void CloudIO::write(std::ostream &file, float v1, float v2, float v3, float v4, float v5){
    file << std::format(line5, v1, v2, v3, v4, v5);
}

void CloudIO::write(std::ostream &file, float v1, float v2, float v3, float v4, float v5, float v6){
    file << std::format(line6, v1, v2, v3, v4, v5, v6);
}

void CloudIO::write(std::ostream &file, float v1, float v2, float v3, float v4, float v5, float v6, float v7){
    file << std::format(line7, v1, v2, v3, v4, v5, v6, v7);
}


