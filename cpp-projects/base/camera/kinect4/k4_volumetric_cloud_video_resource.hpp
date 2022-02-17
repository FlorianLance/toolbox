
#pragma once

// local
#include "k4_volumetric_video_resource.hpp"

namespace tool::camera{

class K4VolumetricCloudVideoResource : public K4VolumetricVideoResource{
public:

    K4CompressedCloudFrame *get_cloud_frame(size_t idFrame, size_t idCamera);

private:

    bool read_file(std::ifstream &file) override;
    void write_file(std::ofstream &file) override;
    void read_frame(std::ifstream &file, K4CompressedCloudFrame *fData);
    void write_frame(std::ofstream &file, K4CompressedCloudFrame *fData);
};

}
