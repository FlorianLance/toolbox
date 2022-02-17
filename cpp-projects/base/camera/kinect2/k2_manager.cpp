
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

#include "k2_manager.hpp"

// std
#include <chrono>
#include <iostream>

// base
#include "utility/benchmark.hpp"

using namespace std::chrono;
using namespace tool::camera;

K2Manager::K2Manager(){
}

bool K2Manager::open_kinect(K2FrameRequest mode){
    return initialized = kinect.open(mode);
}

void K2Manager::close_kinect(){
    initialized = false;
    kinect.close();
}

std::int64_t K2Manager::get_data(){

    if(!initialized){
        return -1;
    }

    auto timeStart = high_resolution_clock::now();
    timeStampFrame = timeStart.time_since_epoch().count();


    Bench::reset();
    if(auto newFrame = kinect.get_kinect_data(); newFrame.has_value()){

        frame = std::make_shared<K2Frame>(std::move(newFrame.value()));
//        if(rand()%100 == 0){
//            Bench::display(BenchUnit::microseconds,1);
//        }
//        std::cout << "d" << duration_cast<microseconds>(high_resolution_clock::now() - timeStart).count() << " ";
        return duration_cast<microseconds>(high_resolution_clock::now() - timeStart).count();
    }

    return -1;
}

void K2Manager::update_parameters(K2Settings parameters){
    kinect.parameters = std::move(parameters);
}

