
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

// kinect
#include "k4a/k4a.hpp"

// local
#include "k4_frame.hpp"

namespace tool::camera{

struct K4CloudFrameCompressor{

    K4CloudFrameCompressor();
    ~K4CloudFrameCompressor();

    std::shared_ptr<K4CompressedCloudFrame> compress(
        size_t validDepthValues,
        int jpegQuality,
        k4a::image colorImage,
        k4a::image depthImage,
        k4a::image cloud,
        float *audioData, size_t audioSize);


private:
    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};

struct K4FullFrameCompressor{

    K4FullFrameCompressor();
    ~K4FullFrameCompressor();

    std::shared_ptr<K4CompressedFullFrame> compress(
        size_t validDepthValues,
        int jpegQuality,
        std::optional<k4a::image> colorImage,
        std::optional<k4a::image> depthImage,
        std::optional<k4a::image> infraredImage);

private:
    struct Impl;
    std::unique_ptr<Impl> i = nullptr;
};


}









