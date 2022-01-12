
#pragma once

// local
#include "volumetric_video_resource.hpp"

namespace tool::camera::K4{

class VolumetricCloudVideoResource : public VolumetricVideoResource{
public:

    CompressedCloudFrame *get_cloud_frame(size_t idFrame, size_t idCamera);

private:

    bool read_file(std::ifstream &file) override;
    void write_file(std::ofstream &file) override;
    void read_frame(std::ifstream &file, CompressedCloudFrame *fData);
    void write_frame(std::ofstream &file, CompressedCloudFrame *fData);
};

}
