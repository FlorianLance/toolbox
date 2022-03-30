
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

#include "k2_config_files.hpp"

// std
#include <filesystem>
#include <fstream>

// local
#include "utility/format.hpp"
#include "utility/logger.hpp"

using namespace tool;
using namespace tool::camera;

bool K2ConfigFiles::save_grabber_settings_config_file(const K2Settings &settings, std::string path){

    if(path.length() == 0){
        path = (std::filesystem::current_path() / "config/settings/settings_saved.config").string();
    }

    std::ofstream file;
    file.open(path);
    if(!file.is_open()){
        Logger::error(fmt("K2ConfigFiles: Cannot write on grabber settings config file with path: {}\n", path));
        return false;
    }

    file << settings.to_string();

    Logger::message(fmt("K2ConfigFiles: Grabber settings config file written: {}", path));
    return true;
}

std::optional<K2Settings> K2ConfigFiles::read_grabber_settings_config_file(std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/settings/settings_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            Logger::error(fmt("K2ConfigFiles: Invalid extension for settings config file {}\n", fsPath.string()));
            return std::nullopt;
        }
    }

    std::ifstream inConfigFile(fsPath.string());
    if(!inConfigFile.is_open()){
        Logger::error(fmt("K2ConfigFiles: No settings config file found at {}\n", fsPath.string()));
        return std::nullopt;
    }

    std::string str;
    inConfigFile.seekg(0, std::ios::end);
    str.reserve(static_cast<size_t>(inConfigFile.tellg()));
    inConfigFile.seekg(0, std::ios::beg);
    str.assign((std::istreambuf_iterator<char>(inConfigFile)),std::istreambuf_iterator<char>());

    Logger::message(fmt("Grabber settings config file read: {}", fsPath.string()));
    return {K2Settings::from_string(str)};
}

bool K2ConfigFiles::read_grabber_network_config_file(int *readingPort, std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/network/network_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            Logger::error(fmt("Invalid extension for grabber network config file {}\n",fsPath.string()));
            return false;
        }
    }

    std::ifstream inConfigFile(fsPath.string());
    if(!inConfigFile.is_open()){
        Logger::error(fmt("Cannot open grabber network config file with path: {}\n", fsPath.string()));
        return false;
    }

    // retrieve reading port
    std::string line;
    while (std::getline(inConfigFile, line)){
        *readingPort = std::stoi(line);
    }

    Logger::message(fmt("Grabber network config file read: {}", fsPath.string()));
    return true;
}

std::vector<K2GrabberTargetInfo> K2ConfigFiles::read_manager_network_config_file(std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/network/network_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            Logger::error(fmt("Invalid extension for manager network config file {}\n", fsPath.string()));
            return {};
        }
    }

    std::ifstream inConfigFile(fsPath.string());
    if(!inConfigFile.is_open()){
        Logger::error(fmt("Cannot open manager network config file with path: {}\n", fsPath.string()));
        return {};
    }

    // retrieve infos
    std_v1<K2GrabberTargetInfo> infos;
    std::string line;
    while (std::getline(inConfigFile, line)){
        auto split = tool::str::split(line, ' ');
        if(split.size() != 4){
            Logger::error(fmt("Invalid manager network config file with path:: {}\n", fsPath.string()));
            return {};
        }
        infos.emplace_back(K2GrabberTargetInfo{split[0],std::stoi(split[1]),std::stoi(split[2]),std::stoi(split[3])});        
    }

    Logger::message(fmt("Manager network config file read: {}", fsPath.string()));
    return infos;
}


std::vector<tool::geo::Mat4d> K2ConfigFiles::read_manager_calibration_file(std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/calibration/calibration_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            Logger::error(fmt("Invalid extension for manager calibration config file {}\n", fsPath.string()));;
            return {};
        }
    }

    std::ifstream inConfigfile(fsPath.string());
    if(!inConfigfile.is_open()){
        Logger::error(fmt("Cannot open manager calibration config file with path: {}\n", fsPath.string()));
        return {};
    }

    std::stringstream buffer;
    buffer << inConfigfile.rdbuf();
    inConfigfile.close();


    auto read_matrix = [&]()-> std::optional<geo::Mat4d>{

        std::string line;
        std_v1<double> values;
        for(size_t ii = 0; ii < 4; ++ii){

            std::getline(buffer, line);

            auto split = str::split(line, ' ');
            if(split.size() == 4){
                values.emplace_back(std::stof(split[0]));
                values.emplace_back(std::stof(split[1]));
                values.emplace_back(std::stof(split[2]));
                values.emplace_back(std::stof(split[3]));
            }else{
                return {};
            }
        }

        return std::make_optional<geo::Mat4d>(values.data());
    };

    bool read = true;
    std_v1<geo::Mat4d> mCalibs;
    while(read){
        std::string line;
        std::getline(buffer, line);
        if(line.size() > 0){

            auto matrix = read_matrix();
            if(matrix.has_value()){
                mCalibs.emplace_back(std::move(matrix.value()));
            }else{
                Logger::error(fmt("Invalid manager calibration config file with path:: {}\n", fsPath.string()));
                return {};
            }

        }else{
            read = false;
        }
    }

    Logger::message(fmt("Manager calibfration config file read: {}", fsPath.string()));
    return mCalibs;
}
