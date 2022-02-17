
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
#include <iostream>
#include <fstream>

using namespace tool::camera;

std::pair<bool, std::string> K2ConfigFiles::save_grabber_settings_config_file(const K2Settings &settings, std::string path){

    if(path.length() == 0){
        path = (std::filesystem::current_path() / "config/settings/settings_saved.config").string();
    }

    std::ofstream file;
    file.open(path);
    if(!file.is_open()){
        return {false, std::format("Cannot write on grabber settings config file with path: {}\n", path)};
    }

    file << settings.to_string();

    return {true, std::format("Grabber settings config file written: {}", path)};
}

std::pair<std::optional<K2Settings>, std::string> K2ConfigFiles::read_grabber_settings_config_file(std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/settings/settings_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            return {std::nullopt, std::format("Invalid extension for settings config file {}\n", fsPath.string())};
        }
    }

    std::ifstream inConfigFile(fsPath.string());
    if(!inConfigFile.is_open()){
        return {std::nullopt, std::format("No settings config file found at {}\n", fsPath.string())};
    }

    std::string str;
    inConfigFile.seekg(0, std::ios::end);
    str.reserve(static_cast<size_t>(inConfigFile.tellg()));
    inConfigFile.seekg(0, std::ios::beg);
    str.assign((std::istreambuf_iterator<char>(inConfigFile)),std::istreambuf_iterator<char>());

    return {K2Settings::from_string(str), std::format("Grabber settings config file read: {}", fsPath.string())};
}

std::pair<bool, std::string> K2ConfigFiles::read_grabber_network_config_file(int *readingPort, std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/network/network_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            return {false, std::format("Invalid extension for grabber network config file {}\n",fsPath.string())};
        }
    }

    std::ifstream inConfigFile(fsPath.string());
    if(!inConfigFile.is_open()){
        return {false, std::format("Cannot open grabber network config file with path: {}\n", fsPath.string())};
    }

    // retrieve reading port
    std::string line;
    while (std::getline(inConfigFile, line)){
        *readingPort = std::stoi(line);
    }

    return {true, std::format("Grabber network config file read: {}", fsPath.string())};
}

std::pair<std::vector<K2GrabberTargetInfo>, std::string> K2ConfigFiles::read_manager_network_config_file(std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/network/network_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            return {{}, std::format("Invalid extension for manager network config file {}\n", fsPath.string())};
        }
    }

    std::ifstream inConfigFile(fsPath.string());
    if(!inConfigFile.is_open()){
        return {{}, std::format("Cannot open manager network config file with path: {}\n", fsPath.string())};
    }

    // retrieve infos
    std_v1<K2GrabberTargetInfo> infos;
    std::string line;
    while (std::getline(inConfigFile, line)){
        auto split = tool::str::split(line, ' ');
        if(split.size() != 4){
            return {{}, std::format("Invalid manager network config file with path:: {}\n", fsPath.string())};
        }
        infos.emplace_back(K2GrabberTargetInfo{split[0],std::stoi(split[1]),std::stoi(split[2]),std::stoi(split[3])});
    }
    return {infos, std::format("Manager network config file read: {}", fsPath.string())};
}


std::pair<std::vector<tool::geo::Mat4d>, std::string> K2ConfigFiles::read_manager_calibration_file(std::string path){

    std::filesystem::path fsPath;
    if(path.length() == 0){
        fsPath = std::filesystem::current_path() / "config/calibration/calibration_default.config";
    }else{
        fsPath = path;
        if(fsPath.extension() != ".config"){
            return {{}, std::format("Invalid extension for manager calibration config file {}\n", fsPath.string())};
        }
    }

    std::ifstream inConfigfile(fsPath.string());
    if(!inConfigfile.is_open()){
        return {{}, std::format("Cannot open manager calibration config file with path: {}\n", fsPath.string())};
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
                return {{}, std::format("Invalid manager calibration config file with path:: {}\n", fsPath.string())};
            }

        }else{
            read = false;
        }
    }

    return {mCalibs, std::format("Manager calibfration config file read: {}", fsPath.string())};
}
