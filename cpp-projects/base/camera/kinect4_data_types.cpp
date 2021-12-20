


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

#include "kinect4_data_types.hpp"

using namespace tool::geo;
using namespace tool::camera::K4;

#include <iostream>


PackedVoxel::PackedVoxel(const Pt3<int16_t> &pos, const Pt3<uint8_t> &col){

    std::bitset<8> r,g,b;
    r = col.x();
    g = col.y();
    b = col.z();

    std::bitset<16> x,y,z;
    x = static_cast<int>(pos.x())+4096;
    y = static_cast<int>(pos.y())+4096;
    z = static_cast<int>(pos.z())+4096;

    std::cout << "x " << x << "\n";
    std::cout << "y " << y << "\n";
    std::cout << "z " << z << "\n";

    std::bitset<32> p;

    size_t id = 0;
    for(int ii = 0; ii < 10; ++ii){
        p[id++] = x[ii];
        p[id++] = y[ii];
        p[id++] = z[ii];
    }
    p[30] = x[10];
    p[31] = y[10];

    std::bitset<32> rgb;
    id = 0;
    for(int ii = 0; ii < 8; ++ii){
        rgb[id++] = r[ii];
        rgb[id++] = g[ii];
        rgb[id++] = b[ii];
    }

    rgb[24] = z[10];
    rgb[25] = x[11];
    rgb[26] = y[11];

    rgb[27] = z[11];
    rgb[28] = x[12];
    rgb[29] = y[12];

    rgb[30] = z[12];
    rgb[31] = z[13];

    p1 = p.to_ulong();
    p2 = rgb.to_ulong();

    std::cout << "rgb " << rgb << " i " << rgb.to_ulong() << " " << p2 << "\n";
}

std::tuple<Pt3<int16_t>, Pt3<uint8_t>> PackedVoxel::unpack() const noexcept{

    std::bitset<32> p   = p1;
    std::bitset<32> rgb = p2;

    std::cout << "rgb " << rgb << " i " << rgb.to_ulong() << " " << p2 << "\n";

    size_t id = 0;
    std::bitset<16> x,y,z;
    for(int ii = 0; ii < 10; ++ii){
        x[ii] = p[id++];
        y[ii] = p[id++];
        z[ii] = p[id++];
    }
    x[10] = p[30];
    y[10] = p[31];

    id = 0;
    std::bitset<8> r,g,b;
    for(int ii = 0; ii < 8; ++ii){
        r[ii] = rgb[id++];
        g[ii] = rgb[id++];
        b[ii] = rgb[id++];
    }

    z[10] = rgb[24];
    x[11] = rgb[25];
    y[11] = rgb[26];

    z[11] = rgb[27];
    x[12] = rgb[28];
    y[12] = rgb[29];

    z[12] = rgb[30];
    z[13] = rgb[31];

    std::cout << "x " << x << "\n";
    std::cout << "y " << y << "\n";
    std::cout << "z " << z << "\n";

    return std::make_tuple(
        Pt3<std::int16_t>{
            static_cast<int16_t>(static_cast<int>(x.to_ulong())-4096),
            static_cast<int16_t>(static_cast<int>(y.to_ulong())-4096),
            static_cast<int16_t>(static_cast<int>(z.to_ulong())-4096)
        },
        Pt3<uint8_t>{
            static_cast<uint8_t>(r.to_ulong()),
            static_cast<uint8_t>(g.to_ulong()),
            static_cast<uint8_t>(b.to_ulong())
        }
    );
}
