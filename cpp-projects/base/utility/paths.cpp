
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


#include "paths.hpp"

// std
#include <filesystem>


using namespace tool;
using namespace std::filesystem;


// https://stackoverflow.com/questions/1528298/get-path-of-executable

void Paths::initialize(char *argv[]){

    auto exePath = weakly_canonical(path(argv[0]));
    executable = exePath.string();

    auto applicationDirPath = exePath.parent_path();
    applicationDir = applicationDirPath.string();

    // resources
    auto resourcesDirPath = (applicationDirPath / "resources");
    if(exists(resourcesDirPath)){
        resourcesDir = resourcesDirPath.string();
    }
    auto shadersDirPath  = (resourcesDirPath / "shaders");
    if(exists(shadersDirPath)){
        shadersDir = shadersDirPath.string();
    }   

    auto dataDirPath = (applicationDirPath / "data");
    if(exists(dataDirPath)){
        dataDir = dataDirPath.string();
    }

    auto logsDirPath = (applicationDirPath / "logs");
    if(exists(logsDirPath)){
        logsDir = logsDirPath.string();
    }

    auto configDirPath = (applicationDirPath / "config");
    if(exists(configDirPath)){
        configDir = configDirPath.string();
    }
}

std::string Paths::get_shader(std::string_view name, std::string_view ext){
    std::string shaderPath;
    if(shadersDir.has_value()){
        shaderPath.reserve(shadersDir.value().length() + name.length() + ext.length());
        shaderPath.append(shadersDir.value());
        shaderPath.append(sep);
        shaderPath.append(name);        
        shaderPath.append(ext);
    }
    return shaderPath;
}


