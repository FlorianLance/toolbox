
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

// local
#include "kinect4_data.hpp"

namespace tool::camera::K4{


struct CloudFrameUncompressor{

    CloudFrameUncompressor();
    ~CloudFrameUncompressor();

    bool uncompress_color(CompressedCloudFrame *cFrame, std::vector<uint8_t> &uncompressedColor);
    bool uncompress(CompressedCloudFrame *cFrame, CloudFrame &frame);

private:
    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};

struct FullFrameUncompressor{

    FullFrameUncompressor();
    ~FullFrameUncompressor();

    bool uncompress_color(CompressedFullFrame *cFrame, std::vector<uint8_t> &uncompressedColor);
    bool uncompress_depth(CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedDepth);
    bool uncompress_infra(CompressedFullFrame *cFrame, std::vector<uint16_t> &uncompressedInfra);
    void generate_cloud(CompressedFullFrame *cFrame, const std::vector<uint16_t> &uncompressedDepth);

    geo::Pt3<int16_t>* cloud_data();

private:
    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};
}









