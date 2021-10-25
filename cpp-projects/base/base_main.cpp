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

    std::cout << "open kinect\n";
    Kinect4 kinect;
    if(!kinect.open(0)){
        return;
    }

    std::cout << "main thread id : " << std::this_thread::get_id() << "\n";
    std::cout << "kinect opened : " << kinect.is_opened() << "\n";

    K4::Config config;
//    config.mode = K4::Mode::Only_color_1280x720; // works
//    config.mode = K4::Mode::Only_color_1920x1080; // works
//    config.mode = K4::Mode::Only_color_2048x1536; // works
    config.mode = K4::Mode::Only_color_2560x1440;
    config.synchronizeColorAndDepth = true;
    config.delayBetweenColorAndDepthUsec = 0;


//    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//    config.color_format    = static_cast<k4a_image_format_t>(ImageFormat::BGRA32);
//    config.color_resolution= static_cast<k4a_color_resolution_t>(ColorResolution::R720P);
//    config.depth_mode      = static_cast<k4a_depth_mode_t>(DepthMode::OFF);
////    config.camera_fps      = static_cast<k4a_fps_t>(Framerate::F30);


//    k4a_device_configuration_t config = kinect.generate_config(
//        ImageFormat::BGRA32,
//        ColorResolution::R720P,
//        DepthMode::OFF,
//        Framerate::F30
//    );
    kinect.start_cameras(config);


    // store frames
    std::vector<std::shared_ptr<CompressedDataFrame>> compressedFrames;
    kinect.new_compressed_data_frame_signal.connect([&](std::shared_ptr<CompressedDataFrame> frame){
        std::cout << "receive compressed frame\n";
        compressedFrames.emplace_back(frame);
    });

    std::vector<std::shared_ptr<DisplayDataFrame>> displayFrames;
    kinect.new_display_frame_signal.connect([&](std::shared_ptr<DisplayDataFrame> frame){
        std::cout << "receive display frame\n";
        displayFrames.emplace_back(frame);
    });

    Parameters p;
    p.sendCompressedDataFrame   = false;
    p.sendDisplayCloud          = false;
    p.sendDisplayColorFrame     = true;
    p.sendDisplayDepthFrame     = false;
    p.sendDisplayInfraredFrame  = false;
    p.filterDepthWithColor      = false;
    kinect.set_parameters(p);

    std::cout << "start reading\n";
    kinect.start_reading();

    std::cout << "sleep\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(7000));

    std::cout << "stop reading\n";
    kinect.stop_reading();

    std::cout << "close\n";
    kinect.close();

    std::cout << "save frames\n";
    size_t idFrame = 0;
    for(const auto &frame : displayFrames){

        std::string pathColor = "./display_color_" + std::to_string(idFrame) + ".png";
        auto &cf = frame->colorFrame;

        tool::graphics::Texture texColor;
        std::cout << cf.width << " " << cf.height << " " << cf.pixels.size() << "\n";
        texColor.copy_2d_data(
            cf.width,
            cf.height,
            cf.pixels
        );

        texColor.write_2d_image_file_data(pathColor);

        ++idFrame;
    }

    std::cout << "end\n";
    return;

    idFrame = 0;
    tjhandle m_jpegUncompressor = tjInitDecompress();
    tool::camera::IntegersEncoder depthCompressor;
    for(const auto &frame : compressedFrames){

        std::cout <<"start processing: " <<  idFrame << "\n";

        std::vector<std::uint8_t> uncompressedColorData;
        uncompressedColorData.resize(frame->colorWidth * frame->colorHeight*4);

        const int decompressStatus = tjDecompress2(
            m_jpegUncompressor,
            frame->compressedColorBuffer.data(),
            static_cast<unsigned long>(frame->compressedColorBuffer.size()),
            uncompressedColorData.data(),
            frame->colorWidth,
            0, // pitch
            frame->colorHeight,
            TJPF_BGRA,
            TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE
            );
        std::cout << "uncompress: " << decompressStatus << "\n";
        std::string pathColor = "./uncompressed_color_" + std::to_string(idFrame) + ".png";
        std::string pathDepth = "./uncompressed_depth_" + std::to_string(idFrame) + ".png";

        std_v1<unsigned char> data;
        data.resize(uncompressedColorData.size());

        auto buffer = uncompressedColorData.data();
        for(size_t ii = 0; ii < uncompressedColorData.size()/4; ++ii){
            const size_t id = ii*4;
            data[id+0] = buffer[id+2];
            data[id+1] = buffer[id+1];
            data[id+2] = buffer[id+0];
            data[id+3] = buffer[id+3];
        }

        tool::graphics::Texture texColor;
        texColor.copy_2d_data(
            frame->colorWidth,
            frame->colorHeight,
            4,
            data
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
                //            reinterpret_cast<std::uint32_t*>(depthI.get_buffer()),
                reinterpret_cast<std::uint32_t*>(depthData.data()),
                (frame->depthWidth*frame->depthHeight)/2
                );
        }catch(std::exception e){
            std::cout << "error " << e.what() << "\n";
        }

        std::cout << "o size: " << originalSize << " " << frame->depthWidth << " " << frame->depthHeight << " " << frame->depthWidth*frame->depthHeight << "\n";

        k4a::image depthI = k4a::image::create(
            K4A_IMAGE_FORMAT_DEPTH16,
            frame->depthWidth,
            frame->depthHeight,
            frame->depthWidth * 2
            );
        std::copy(
            depthData.begin(),
            depthData.end(),
            reinterpret_cast<std::uint16_t*>(depthI.get_buffer()));

        // camera::Kinect4::write_depth_image(pathDepth, depthI);


        ++idFrame;
    }

    //    size_t idCloud = 0;
    //    for(const auto &cloud : clouds){
    //        std::string pathCloud = "./cloud_" + std::to_string(idCloud++) + ".obj";
    //        std::cout << "cloud size: " << cloud->vertices.size() << "\n";
    //        tool::io::save_cloud<float>(pathCloud, cloud->vertices.data(), cloud->colors.data(), cloud->vertices.size());
    //    }

    tool::Bench::display();
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

    //    Logger::message("Test formated log message", true);
    //    Logger::message("Test log message", false);


    //    Logger::warning("Test log warning");

    //    kinect4_test();

    tool::Bench::display();
    tool::Bench::reset();

    tool::Bench::stop();
}


int main(){

    kinect4_test();


    std::cout << "base-lib end\n";
    return 0;
}



