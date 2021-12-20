


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

ImageFormat image_format(Mode m) {
    return modes.at<0,1>(m);
}

std::tuple<std::uint32_t,std::uint32_t> PackedVoxel::pack(const geo::Pt3<int16_t> &pos, const geo::Pt4<uint8_t> &col) noexcept{

    const int x = static_cast<int>(pos.x())+4096;
    const int y = static_cast<int>(pos.y())+4096;
    const int z = static_cast<int>(pos.z());

    std::uint32_t p1 =
        /**  0-0  */((x & 0b1))                     |
        /**  1-0  */((y & 0b1)               << 1)  |
        /**  2-0  */((z & 0b1)               << 2)  |
        /**  3-1  */((x & 0b10)              << 2)  |
        /**  4-1  */((y & 0b10)              << 3)  |
        /**  5-1  */((z & 0b10)              << 4)  |
        /**  6-2  */((x & 0b100)             << 4)  |
        /**  7-2  */((y & 0b100)             << 5)  |
        /**  8-2  */((z & 0b100)             << 6)  |
        /**  9-3  */((x & 0b1000)            << 6)  |
        /** 10-3  */((y & 0b1000)            << 7)  |
        /** 11-3  */((z & 0b1000)            << 8)  |
        /** 12-4  */((x & 0b10000)           << 8)  |
        /** 13-4  */((y & 0b10000)           << 9)  |
        /** 14-4  */((z & 0b10000)           << 10) |
        /** 15-5  */((x & 0b100000)          << 10) |
        /** 16-5  */((y & 0b100000)          << 11) |
        /** 17-5  */((z & 0b100000)          << 12) |
        /** 18-6  */((x & 0b1000000)         << 12) |
        /** 19-6  */((y & 0b1000000)         << 13) |
        /** 20-6  */((z & 0b1000000)         << 14) |
        /** 21-7  */((x & 0b10000000)        << 14) |
        /** 22-7  */((y & 0b10000000)        << 15) |
        /** 23-7  */((z & 0b10000000)        << 16) |
        /** 24-8  */((x & 0b100000000)       << 16) |
        /** 25-8  */((y & 0b100000000)       << 17) |
        /** 26-8  */((z & 0b100000000)       << 18) |
        /** 27-9  */((x & 0b1000000000)      << 18) |
        /** 28-9  */((y & 0b1000000000)      << 19) |
        /** 29-9  */((z & 0b1000000000)      << 20) |
        /** 30-10 */((x & 0b10000000000)     << 20) |
        /** 31-10 */((y & 0b10000000000)     << 21);

    const std::uint8_t r = col.x();
    const std::uint8_t g = col.y();
    const std::uint8_t b = col.z();
    std::uint32_t p2 =
        /** 0-0   */ ((r & 0b1))                    |
        /** 1-0   */ ((g & 0b1)              << 1)  |
        /** 2-0   */ ((b & 0b1)              << 2)  |
        /** 3-1   */ ((r & 0b10)             << 2)  |
        /** 4-1   */ ((g & 0b10)             << 3)  |
        /** 5-1   */ ((b & 0b10)             << 4)  |
        /** 6-2   */ ((r & 0b100)            << 4)  |
        /** 7-2   */ ((g & 0b100)            << 5)  |
        /** 8-2   */ ((b & 0b100)            << 6)  |
        /** 9-3   */ ((r & 0b1000)           << 6)  |
        /** 10-3  */ ((g & 0b1000)           << 7)  |
        /** 11-3  */ ((b & 0b1000)           << 8)  |
        /** 12-4  */ ((r & 0b10000)          << 8)  |
        /** 13-4  */ ((g & 0b10000)          << 9)  |
        /** 14-4  */ ((b & 0b10000)          << 10) |
        /** 15-5  */ ((r & 0b100000)         << 10) |
        /** 16-5  */ ((g & 0b100000)         << 11) |
        /** 17-5  */ ((b & 0b100000)         << 12) |
        /** 18-6  */ ((r & 0b1000000)        << 12) |
        /** 19-6  */ ((g & 0b1000000)        << 13) |
        /** 20-6  */ ((b & 0b1000000)        << 14) |
        /** 21-7  */ ((r & 0b10000000)       << 14) |
        /** 22-7  */ ((g & 0b10000000)       << 15) |
        /** 23-7  */ ((b & 0b10000000)       << 16) |

        /** 24-10 */ ((z & 0b10000000000)    << 14) |
        /** 25-11 */ ((x & 0b100000000000)   << 14) |
        /** 26-11 */ ((y & 0b100000000000)   << 15) |
        /** 27-11 */ ((z & 0b100000000000)   << 16) |
        /** 28-12 */ ((x & 0b1000000000000)  << 16) |
        /** 29-12 */ ((y & 0b1000000000000)  << 17) |
        /** 30-12 */ ((z & 0b1000000000000)  << 18) |
        /** 31-13 */ ((z & 0b10000000000000) << 18);

    return {p1,p2};
}

void PackedVoxel::unpack(std::uint32_t p1, std::uint32_t p2, geo::Pt3<int16_t> &pos, geo::Pt3<uint8_t> &col)  noexcept{

    pos.x() = static_cast<int16_t>(
        /* 0  */ ((p1 & 0b1))                                      |
        /* 1  */ ((p1 & 0b1000)                             >> 2)  |
        /* 2  */ ((p1 & 0b1000000)                          >> 4)  |
        /* 3  */ ((p1 & 0b1000000000)                       >> 6)  |
        /* 4  */ ((p1 & 0b1000000000000)                    >> 8)  |
        /* 5  */ ((p1 & 0b1000000000000000)                 >> 10) |
        /* 6  */ ((p1 & 0b1000000000000000000)              >> 12) |
        /* 7  */ ((p1 & 0b1000000000000000000000)           >> 14) |
        /* 8  */ ((p1 & 0b1000000000000000000000000)        >> 16) |
        /* 9  */ ((p1 & 0b1000000000000000000000000000)     >> 18) |
        /* 10 */ ((p1 & 0b1000000000000000000000000000000)  >> 20) |
        /* 11 */ ((p2 & 0b10000000000000000000000000)       >> 14) |
        /* 12 */ ((p2 & 0b10000000000000000000000000000)    >> 16) - 4096);

    pos.y() = static_cast<int16_t>(
        /* 0  */ ((p1 & 0b10)                                >> 1)  |
        /* 1  */ ((p1 & 0b10000)                             >> 3)  |
        /* 2  */ ((p1 & 0b10000000)                          >> 5)  |
        /* 3  */ ((p1 & 0b10000000000)                       >> 7)  |
        /* 4  */ ((p1 & 0b10000000000000)                    >> 9)  |
        /* 5  */ ((p1 & 0b10000000000000000)                 >> 11) |
        /* 6  */ ((p1 & 0b10000000000000000000)              >> 13) |
        /* 7  */ ((p1 & 0b10000000000000000000000)           >> 15) |
        /* 8  */ ((p1 & 0b10000000000000000000000000)        >> 17) |
        /* 9  */ ((p1 & 0b10000000000000000000000000000)     >> 19) |
        /* 10 */ ((p1 & 0b10000000000000000000000000000000)  >> 21) |
        /* 11 */ ((p2 & 0b100000000000000000000000000)       >> 15) |
        /* 12 */ ((p2 & 0b100000000000000000000000000000)    >> 17) - 4096);

    pos.z() = static_cast<int16_t>(
        /* 0  */ ((p1 & 0b100)                               >> 2)  |
        /* 1  */ ((p1 & 0b100000)                            >> 4)  |
        /* 2  */ ((p1 & 0b100000000)                         >> 6)  |
        /* 3  */ ((p1 & 0b100000000000)                      >> 8)  |
        /* 4  */ ((p1 & 0b100000000000000)                   >> 10) |
        /* 5  */ ((p1 & 0b100000000000000000)                >> 12) |
        /* 6  */ ((p1 & 0b100000000000000000000)             >> 14) |
        /* 7  */ ((p1 & 0b100000000000000000000000)          >> 16) |
        /* 8  */ ((p1 & 0b100000000000000000000000000)       >> 18) |
        /* 9  */ ((p1 & 0b100000000000000000000000000000)    >> 20) |
        /* 10 */ ((p2 & 0b1000000000000000000000000)         >> 14) |
        /* 11 */ ((p2 & 0b1000000000000000000000000000)      >> 16) |
        /* 12 */ ((p2 & 0b1000000000000000000000000000000)   >> 18) |
        /* 13 */ ((p2 & 0b10000000000000000000000000000000)  >> 19));

    col.x() = static_cast<uint8_t>(
        /* 0  */ ((p2 & 0b1))                                     |
        /* 1  */ ((p2 & 0b1000)                            >> 2)  |
        /* 2  */ ((p2 & 0b1000000)                         >> 4)  |
        /* 3  */ ((p2 & 0b1000000000)                      >> 6)  |
        /* 4  */ ((p2 & 0b1000000000000)                   >> 8)  |
        /* 5  */ ((p2 & 0b1000000000000000)                >> 10) |
        /* 6  */ ((p2 & 0b1000000000000000000)             >> 12) |
        /* 7  */ ((p2 & 0b1000000000000000000000)          >> 14));

    col.y() = static_cast<uint8_t>(
        /* 0  */ ((p2 & 0b10)                              >> 1)  |
        /* 1  */ ((p2 & 0b10000)                           >> 3)  |
        /* 2  */ ((p2 & 0b10000000)                        >> 5)  |
        /* 3  */ ((p2 & 0b10000000000)                     >> 7)  |
        /* 4  */ ((p2 & 0b10000000000000)                  >> 9)  |
        /* 5  */ ((p2 & 0b10000000000000000)               >> 11) |
        /* 6  */ ((p2 & 0b10000000000000000000)            >> 13) |
        /* 7  */ ((p2 & 0b10000000000000000000000)         >> 15));

    col.z() = static_cast<uint8_t>(
        /* 0  */ ((p2 & 0b100)                             >> 2)  |
        /* 1  */ ((p2 & 0b100000)                          >> 4)  |
        /* 2  */ ((p2 & 0b100000000)                       >> 6)  |
        /* 3  */ ((p2 & 0b100000000000)                    >> 8)  |
        /* 4  */ ((p2 & 0b100000000000000)                 >> 10) |
        /* 5  */ ((p2 & 0b100000000000000000)              >> 12) |
        /* 6  */ ((p2 & 0b100000000000000000000)           >> 14) |
        /* 7  */ ((p2 & 0b100000000000000000000000)        >> 16));
}

//PackedVoxel::PackedVoxel(const Pt3<int16_t> &pos, const Pt3<uint8_t> &col){

//    std::bitset<8> r,g,b;
//    r = col.x();
//    g = col.y();
//    b = col.z();

//    std::bitset<16> x,y,z;
//    x = static_cast<int>(pos.x())+4096;
//    y = static_cast<int>(pos.y())+4096;
//    z = pos.z();

//    std::bitset<32> p;

//    size_t id = 0;
//    for(int ii = 0; ii < 10; ++ii){
//        p[id++] = x[ii];
//        p[id++] = y[ii];
//        p[id++] = z[ii];
//    }
//    p[30] = x[10];
//    p[31] = y[10];

//    std::bitset<32> rgb;
//    id = 0;
//    for(int ii = 0; ii < 8; ++ii){
//        rgb[id++] = r[ii];
//        rgb[id++] = g[ii];
//        rgb[id++] = b[ii];
//    }

//    rgb[24] = z[10];
//    rgb[25] = x[11];
//    rgb[26] = y[11];

//    rgb[27] = z[11];
//    rgb[28] = x[12];
//    rgb[29] = y[12];

//    rgb[30] = z[12];
//    rgb[31] = z[13];

//    p1 = p.to_ulong();
//    p2 = rgb.to_ulong();

//}

//std::tuple<Pt3<int16_t>, Pt3<uint8_t>> PackedVoxel::unpack() const noexcept{

//    std::bitset<32> p   = p1;
//    std::bitset<32> rgb = p2;

//    size_t id = 0;
//    std::bitset<16> x,y,z;
//    for(int ii = 0; ii < 10; ++ii){
//        x[ii] = p[id++];
//        y[ii] = p[id++];
//        z[ii] = p[id++];
//    }
//    x[10] = p[30];
//    y[10] = p[31];

//    id = 0;
//    std::bitset<8> r,g,b;
//    for(int ii = 0; ii < 8; ++ii){
//        r[ii] = rgb[id++];
//        g[ii] = rgb[id++];
//        b[ii] = rgb[id++];
//    }

//    z[10] = rgb[24];
//    x[11] = rgb[25];
//    y[11] = rgb[26];

//    z[11] = rgb[27];
//    x[12] = rgb[28];
//    y[12] = rgb[29];

//    z[12] = rgb[30];
//    z[13] = rgb[31];

//    return std::make_tuple(
//        Pt3<std::int16_t>{
//            static_cast<int16_t>(static_cast<int>(x.to_ulong())-4096),
//            static_cast<int16_t>(static_cast<int>(y.to_ulong())-4096),
//            static_cast<int16_t>(z.to_ulong())
//        },
//        Pt3<uint8_t>{
//            static_cast<uint8_t>(r.to_ulong()),
//            static_cast<uint8_t>(g.to_ulong()),
//            static_cast<uint8_t>(b.to_ulong())
//        }
//    );
//}

