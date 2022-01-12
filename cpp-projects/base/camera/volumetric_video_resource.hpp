

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
#include "geometry/matrix4.hpp"

namespace tool::camera::K4{

struct FrameData{
    size_t idFrame;
    std::int64_t timeStamp;
    std::shared_ptr<CompressedFrame> data = nullptr;
};

struct CameraData{
    geo::Mat4d transform = geo::Mat4d::identity();
    std::vector<FrameData> frames;
};

class VolumetricVideoResource{

public:
    size_t nb_cameras() const noexcept;
    size_t nb_frames(size_t idCamera = 0) const noexcept;
    size_t frame_id(size_t idCamera, float timeMs) const;
    size_t valid_vertices_count(size_t idFrame, size_t idCamera = 0) const;

    std::int64_t start_time(size_t idCamera) const;
    std::int64_t end_time(size_t idCamera) const;

    void set_transform(size_t idCamera, geo::Mat4d tr);
    geo::Mat4d get_transform(size_t idCamera) const;

    CompressedFrame *get_frame(size_t idFrame, size_t idCamera = 0);
    std::vector<FrameData> *get_frames(size_t idCamera = 0);

    void add_frame(size_t idCamera, std::int64_t timestamp, std::shared_ptr<CompressedFrame> frame);
    void remove_frames_until(size_t idFrame);
    void remove_frames_after(size_t idFrame);
    void clean_frames();

    bool save_to_file(const std::string &path);
    bool load_from_file(const std::string &path);

    std::vector<CameraData> camData;

private:
    virtual bool read_file(std::ifstream &file) = 0;
    virtual void write_file(std::ofstream &file) = 0;

};



}
