
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
#include <string>
#include <sstream>
#include <algorithm>

// local
#include "utility/vector.hpp"

namespace tool::str {

[[maybe_unused]] static inline bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos){
        return false;
    }
    str.replace(start_pos, from.length(), to);
    return true;
}

[[maybe_unused]] static inline void replace_all(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty()){
        return;
    }
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

static void replace_all2(std::string& source, const std::string& from, const std::string& to){

    std::string newString;
    newString.reserve(source.length());  // avoids a few memory allocations

    std::string::size_type lastPos = 0;
    std::string::size_type findPos;

    while(std::string::npos != (findPos = source.find(from, lastPos))){
        newString.append(source, lastPos, findPos - lastPos);
        newString += to;
        lastPos = findPos + from.length();
    }

    // Care for the rest after last occurrence
    //newString += source.substr(lastPos);
    newString.append(source, lastPos, source.length() - lastPos);

    source.swap(newString);
}


[[maybe_unused]] static inline void remove_from_left(std::string &s, char delim){
    auto pos = s.find_last_of(delim);
    if(pos ==  std::string::npos){
        return;
    }
    s.erase(0, pos+1);
}

[[maybe_unused]] static inline void remove_from_right(std::string &s, char delim){
    auto pos = s.find_last_of(delim);
    if(pos ==  std::string::npos){
        return;
    }
    s.erase(pos);
}

[[maybe_unused]] static inline void remove_after_right(std::string &s, char delim){
    auto pos = s.find_first_of(delim);
    if(pos ==  std::string::npos){
        return;
    }
    s.erase(pos);
}



[[maybe_unused]] static inline std::pair<std::string, std::string> split_path_and_filename(const std::string &s){
    auto pos1 = s.find_last_of('/');
    auto pos2 = s.find_last_of('\\');

    if(pos1 ==  std::string::npos){
        if(pos2 == std::string::npos){
            return {".", s};
        }else{
            return std::make_pair(s.substr(0, pos2), s.substr(pos2+1));
        }
    }

    return std::make_pair(s.substr(0, pos1), s.substr(pos1+1));
}

[[maybe_unused]] static inline std_v1<std::string> split(const std::string &s, char delim){
    std_v1<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)){
        elems.push_back(item);
    }
    return elems;
}


//static std::vector<std::string> split3(const std::string& str, const std::string& delims = " "){

//    std::vector<std::string> output;
//    auto first = std::cbegin(str);

//    while (first != std::cend(str)){

//        const auto second = std::find_first_of(first, std::cend(str),
//                  std::cbegin(delims), std::cend(delims));

//        if (first != second){
//            output.emplace_back(first, second);
//        }

//        if (second == std::cend(str)){
//            break;
//        }

//        first = std::next(second);
//    }

//    return output;
//}


//template <typename Out>
//static void split2(const std::string &s, char delim, Out result) {
//    std::istringstream iss(s);
//    std::string item;
//    while (std::getline(iss, item, delim)) {
//        *result++ = item;
//    }
//}

//static std::vector<std::string> split2(const std::string &s, char delim) {
//    std::vector<std::string> elems;
//    split2(s, delim, std::back_inserter(elems));
//    return elems;
//}




}
