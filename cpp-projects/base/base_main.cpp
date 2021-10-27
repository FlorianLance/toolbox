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

// std
#include <iostream>
#include <filesystem>
#include <ostream>

// kinect4
#include <k4a/k4a.hpp>

// turbojpg
#include <turbojpeg.h>

// local
#include "utility/logger.hpp"
#include "camera/kinect2.hpp"
#include "camera/kinect4.hpp"
#include "data/integers_encoder.hpp"
#include "graphics/texture.hpp"
#include "utility/benchmark.hpp"
#include "utility/files.hpp"

using namespace tool;

void kinect2_test(){

    using namespace std::chrono_literals;
    camera::Kinect2 kinect;
    if(!kinect.open(camera::K2::FrameRequest::compressed_color_mesh)){
        std::cerr << "Cannot init kinect 2\n";
        return;
    }

    std::cout << "init\n";
    std::this_thread::sleep_for(1000ms);
    std::cout << "try to grab\n";

    for(int ii = 0; ii < 10000; ++ii){
        if(auto newFrame = kinect.get_kinect_data(); newFrame.has_value()){
            std::cout << "-";
        }else{
            std::cout << "E";
        }
        std::this_thread::sleep_for((1000/33)*1ms);
    }

    std::cout << "close\n";
    kinect.close();
}

void kinect4_test(){

    using namespace camera;
    using namespace camera::K4;


    Logger::message("Create kinect4\n");
    Kinect4 kinect;

    Logger::message("Open kinect4\n");
    if(!kinect.open(0)){
        return;
    }

    std::cout << "Main thread id id " << std::this_thread::get_id() << "\n";

    K4::Config config;
//    config.mode = K4::Mode::Only_color_1280x720; // works
//    config.mode = K4::Mode::Only_color_1920x1080; // works
//    config.mode = K4::Mode::Only_color_2048x1536; // works
    config.mode = K4::Mode::Cloud_1024x1024;
    kinect.start_cameras(config);

    // stored frames
    std::vector<std::shared_ptr<DisplayDataFrame>> displayFrames;
    std::vector<std::shared_ptr<CompressedDataFrame>> compressedFrames;

    // connections
    kinect.new_compressed_data_frame_signal.connect([&](std::shared_ptr<CompressedDataFrame> frame){
        std::cout << "receive compressed frame from thread id " << std::this_thread::get_id() << "\n";
//        Logger::message(std::format("receive compressed frame from thread id {}\n", std::this_thread::get_id()));
        compressedFrames.emplace_back(frame);
    });   
    kinect.new_display_frame_signal.connect([&](std::shared_ptr<DisplayDataFrame> frame){
        std::cout << "receive display frame from thread id " << std::this_thread::get_id() << "\n";
//        Logger::message(std::format("receive display frame from thread id {}\n", std::this_thread::get_id()));
        displayFrames.emplace_back(frame);
    });

    // parameters
    Parameters p;
    p.sendCompressedDataFrame   = true;
    p.sendDisplayCloud          = true;
    p.sendDisplayColorFrame     = true;
    p.sendDisplayDepthFrame     = true;
    p.sendDisplayInfraredFrame  = false;
    p.filterDepthWithColor      = false;
    p.jpegCompressionRate       = 80;
    kinect.set_parameters(p);

    Logger::message("Start reading.\n");
    kinect.start_reading();

    Logger::message("Sleep...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    Logger::message("Stop reading.\n");
    kinect.stop_reading();

    Logger::message("Close.\n");
    kinect.close();

    Logger::message("Save display frames.\n");
    size_t idFrame = 0;
    for(const auto &frame : displayFrames){

        BenchGuard g("save frame");

        std::string pathColor = "./display_color_" + std::to_string(idFrame) + ".png";
        std::string pathDepth = "./display_depth_" + std::to_string(idFrame) + ".png";
        std::string pathInfra = "./display_infra_" + std::to_string(idFrame) + ".png";
        std::string pathCloud = "./display_cloud_" + std::to_string(idFrame) + ".obj";

        auto &cf = frame->colorFrame;
        tool::graphics::Texture texColor;
        texColor.copy_2d_data(
            cf.width,
            cf.height,
            cf.pixels
        );
        if(!texColor.write_2d_image_file_data(pathColor)){
            Logger::error("Faild color.\n");
        }

        auto &df = frame->depthFrame;
        tool::graphics::Texture texDepth;
        texDepth.copy_2d_data(
            df.width,
            df.height,
            df.pixels
        );
        if(!texDepth.write_2d_image_file_data(pathDepth)){
            Logger::error("Faild depth.\n");
        }

        auto &irf = frame->infraredFrame;
        tool::graphics::Texture texInfra;
        texInfra.copy_2d_data(
            irf.width,
            irf.height,
            irf.pixels
        );
        if(!texInfra.write_2d_image_file_data(pathInfra)){
            Logger::error("Faild infra.\n");
        }

        auto &cloudF = frame->cloud;
        if(!tool::files::save_cloud(pathCloud, cloudF.vertices.data(), cloudF.colors.data(), cloudF.validVerticesCount)){
            Logger::error("Faild cloud.\n");
        }


        ++idFrame;
    }

    Logger::message("Save uncompressed frames.\n");
    idFrame = 0;
    tjhandle jpegUncompressor = tjInitDecompress();
    tool::camera::IntegersEncoder depthCompressor;
    for(const auto &frame : compressedFrames){

        BenchGuard g("save uncompressed frame");

        std::string pathColor = "./uncompressed_color_" + std::to_string(idFrame) + ".png";
        std::string pathDepth = "./uncompressed_depth_" + std::to_string(idFrame) + ".png";

        std::vector<std::uint8_t> uncompressedColorData;
        uncompressedColorData.resize(frame->colorWidth * frame->colorHeight*4);

        const int decompressStatus = tjDecompress2(
            jpegUncompressor,
            frame->compressedColorBuffer.data(),
            static_cast<unsigned long>(frame->compressedColorBuffer.size()),
            uncompressedColorData.data(),
            frame->colorWidth,
            0, // pitch
            frame->colorHeight,
            TJPF_RGBA,
            TJFLAG_FASTDCT// | TJFLAG_FASTUPSAMPLE
        );
        if(decompressStatus == -1){
            std::cerr << "Error uncompress jpeg\n";
            break;
        }

        tool::graphics::Texture texColor;
        texColor.copy_2d_data(
            frame->colorWidth,
            frame->colorHeight,
            4,
            uncompressedColorData.data()
        );
        texColor.write_2d_image_file_data(pathColor);

        // depth
        std::vector<std::uint16_t> depthData;
        depthData.resize(frame->depthWidth*frame->depthHeight);

        size_t originalSize;

        try{
            originalSize= depthCompressor.decode(
                frame->compressedDepthBuffer.data(),
                frame->compressedDepthBuffer.size(),
                reinterpret_cast<std::uint32_t*>(depthData.data()),
                (frame->depthWidth*frame->depthHeight)/2
            );
        }catch(std::exception e){
            std::cout << "Error uncompress depth " << e.what() << "\n";
        }

        k4a::image depthI = k4a::image::create(
            K4A_IMAGE_FORMAT_DEPTH16,
            frame->depthWidth,
            frame->depthHeight,
            frame->depthWidth * 2
        );
        std::copy(
            depthData.begin(),
            depthData.end(),
            reinterpret_cast<std::uint16_t*>(depthI.get_buffer())
        );

//         camera::Kinect4::write_depth_image(pathDepth, depthI);

        idFrame++;
    }
    tjDestroy(jpegUncompressor);




    //    size_t idCloud = 0;
    //    for(const auto &cloud : clouds){
    //        std::string pathCloud = "./cloud_" + std::to_string(idCloud++) + ".obj";
    //        std::cout << "cloud size: " << cloud->vertices.size() << "\n";
    //        tool::io::save_cloud<float>(pathCloud, cloud->vertices.data(), cloud->colors.data(), cloud->vertices.size());
    //    }

}

void bench_test(){

    {
        const auto t1 = "t1"sv;
        tool::Bench::start(t1, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tool::Bench::stop();

        {
            tool::BenchGuard g("g1", true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        tool::Bench::start("t2", true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tool::Bench::stop();

        tool::BenchGuard g0("g0", true);
        {
            tool::BenchGuard g1("g1", true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            {
                tool::BenchGuard g2("g2", true);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

            }

            tool::BenchGuard g3("g3", true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        tool::Bench::start("t3"sv, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tool::Bench::stop();

        tool::Bench::start("t4", true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tool::Bench::stop();
    }

    tool::BenchGuard g3("g4", true);

    tool::Bench::start("base-lib start", true);
    Logger::init((std::filesystem::current_path() / "base-lib-logs").string(), "logs.html");

    auto logger = Logger::get();
    logger->message_signal.connect([&](std::string messsage){
        std::cout << "Message from logger: " << messsage << "\n";
    });

    tool::Bench::stop();
}


int main(){

    Logger::message("base-lib start\n");

    tool::Bench::reset();
    tool::Bench::start("kinect4_test");
    Logger::message("kinect4_test\n");
    kinect4_test();
    tool::Bench::stop();
    tool::Bench::display();

//    tool::Bench::reset();
//    tool::Bench::start("bench_test");
//    Logger::message("bench_test\n");
//    bench_test();
//    tool::Bench::stop();
//    tool::Bench::display();

    Logger::message("base-lib end\n");

    return 0;
}



