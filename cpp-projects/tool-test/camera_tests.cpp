
/*******************************************************************************
** tool-test                                                                  **
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
#include <format>
#include <iostream>
#include <execution>
#include <map>

// catch
#include <catch.hpp>

// qoi
#define QOI_IMPLEMENTATION
#include <qoi.h>

// turbojpg
#include <turbojpeg.h>

// oroch
//#include "oroch/bitpfr.h"
//#include "oroch/varint.h"

// simdcomp
//#include "data/simdcomp/simdcomp.h"
#include "data/FastDifferentialCoding/fastdelta.h"
//#include "fastpfor/codecfactory.h"

#include "TurboPFor/vp4.h"

// base
#include "graphics/texture.hpp"
#include "camera/kinect4/k4_data.hpp"
#include "utility/logger.hpp"
#include "utility/benchmark.hpp"
#include "data/integers_encoder.hpp"
#include "files/cloud_io.hpp"

#include "camera/kinect4/k4_volumetric_full_video_manager.hpp"

#include "camera/kinect4/k4_frame_compressor.hpp"
#include "camera/kinect4/k4_frame_uncompressor.hpp"


using namespace tool;
using namespace tool::camera;


TEST_CASE("Kinect4 camera"){

//    CloudFrameCompressor cc;
//    FullFrameCompressor fc;
//    CloudFrameUncompressor cu;
//    FullFrameUncompressor fu;

//    CloudFrame dd;
//    CompressedCloudFrame f;
//    cu.uncompress(&f, dd);

    K4VolumetricFullVideoResource video;

//    const std::string filePath = "E:/ttt.kvid";
    const std::string filePath = "D:/compress_test.kvid";
    REQUIRE(video.load_from_file(filePath));

    K4VolumetricFullVideoManager manager(&video);

    auto jpegCompressor = tjInitCompress();
    tjhandle jpegUncompressor = tjInitDecompress();    
    unsigned char *tjCompressedImage = nullptr;
    data::IntegersEncoder integerCompressor;
    std::vector<std::uint8_t> uncompressedColorFrame;
    std::vector<std::uint16_t> uncompressedDepthFrame;

    std::map<std::string, std::vector<std::tuple<float, size_t>>> scores;

    auto check16 = [](std::uint16_t *input, std::uint16_t *decoded, size_t size, size_t encodedBytesNb, size_t decodedBytesNb){
        REQUIRE(encodedBytesNb == decodedBytesNb);
        bool isIdentical = true;
        for(size_t ii= 0; ii < size; ++ii){
            if(input[ii] != decoded[ii]){
                isIdentical = false;
                break;
            }
        }
        REQUIRE(isIdentical);
    };
    auto check32 = [](std::uint32_t *input, std::uint32_t *decoded, size_t size, size_t encodedBytesNb, size_t decodedBytesNb){
        REQUIRE(encodedBytesNb == decodedBytesNb);
        bool isIdentical = true;
        for(size_t ii= 0; ii < size; ++ii){
            if(input[ii] != decoded[ii]){
                isIdentical = false;
                break;
            }
        }
        REQUIRE(isIdentical);
    };
    auto check64 = [](std::uint64_t *input, std::uint64_t *decoded, size_t size, size_t encodedBytesNb, size_t decodedBytesNb){
        REQUIRE(encodedBytesNb == decodedBytesNb);
        bool isIdentical = true;
        for(size_t ii= 0; ii < size; ++ii){
            if(input[ii] != decoded[ii]){
                isIdentical = false;
                break;
            }
        }
        REQUIRE(isIdentical);
    };

    auto algos_16 = [&](std::string name, std::uint16_t *inputData16, size_t insize, unsigned char *encoded, std::uint16_t *decoded16){

//        std::cout << name << " -> algos_16\n";

        size_t encodedBytesNb,decodedBytesNb;

        Bench::start("p4nenc16");
        encodedBytesNb = p4nenc16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4nenc16");
        decodedBytesNb = p4ndec16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nenc16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4ndenc16");
        encodedBytesNb = p4ndenc16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4nddec16");
        decodedBytesNb = p4nddec16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4ndenc16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nd1enc16");
        encodedBytesNb = p4nd1enc16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4nd1dec16");
        decodedBytesNb = p4nd1dec16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nd1enc16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nzenc16");
        encodedBytesNb = p4nzenc16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4nzdec16");
        decodedBytesNb = p4nzdec16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nzenc16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);
    };

    auto algos_32 = [&](std::string name, std::uint32_t *inputData32, size_t insize, unsigned char *encoded, std::uint32_t *decoded32){

//        std::cout << name << " -> algos_32\n";

        size_t encodedBytesNb,decodedBytesNb;

        Bench::start("p4nenc32");
        encodedBytesNb = p4nenc32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4ndec32");
        decodedBytesNb = p4ndec32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nenc32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4ndenc32");
        encodedBytesNb = p4ndenc32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4nddec32");
        decodedBytesNb = p4nddec32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4ndenc32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nd1enc32");
        encodedBytesNb = p4nd1enc32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4nd1dec32");
        decodedBytesNb = p4nd1dec32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nd1enc32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nzenc32");
        encodedBytesNb = p4nzenc32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4nzdec32");
        decodedBytesNb = p4nzdec32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nzenc32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

    };

    auto algos_64 = [&](std::string name, std::uint64_t *inputData64, size_t insize, unsigned char *encoded, std::uint64_t *decoded64){

//        std::cout << name << " -> algos_64\n";

        size_t encodedBytesNb,decodedBytesNb;

        Bench::start("p4nenc64");
        encodedBytesNb = p4nenc64(inputData64, insize, encoded); Bench::stop();
        Bench::start("p4ndec64");
        decodedBytesNb = p4ndec64(encoded, insize, decoded64); Bench::stop();
        check64(inputData64, decoded64, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nenc64"].emplace_back(0.125f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4ndenc64");
        encodedBytesNb = p4ndenc64(inputData64, insize, encoded); Bench::stop();
        Bench::start("p4nddec64");
        decodedBytesNb = p4nddec64(encoded, insize, decoded64); Bench::stop();
        check64(inputData64, decoded64, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4ndenc64"].emplace_back(0.125f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nd1enc64");
        encodedBytesNb = p4nd1enc64(inputData64, insize, encoded); Bench::stop();
        Bench::start("p4nd1dec64");
        decodedBytesNb = p4nd1dec64(encoded, insize, decoded64); Bench::stop();
        check64(inputData64, decoded64, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nd1enc64"].emplace_back(0.125f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nzenc64");
        encodedBytesNb = p4nzenc64(inputData64, insize, encoded); Bench::stop();
        Bench::start("p4nzdec64");
        decodedBytesNb = p4nzdec64(encoded, insize, decoded64); Bench::stop();
        check64(inputData64, decoded64, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nzenc64"].emplace_back(0.125f*encodedBytesNb/insize, encodedBytesNb);
    };

    auto simd128_algos_16 = [&](std::string name, std::uint16_t *inputData16, size_t insize, unsigned char *encoded, std::uint16_t *decoded16){

//        std::cout << name << " -> simd128_algos_16\n";
        size_t encodedBytesNb,decodedBytesNb;

        Bench::start("p4nenc128v16");
        encodedBytesNb = p4nenc128v16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4ndec128v16");
        decodedBytesNb = p4ndec128v16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nenc128v16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4ndenc128v16");
        encodedBytesNb = p4ndenc128v16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4nddec128v16");
        decodedBytesNb = p4nddec128v16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4ndenc128v16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nd1enc128v16");
        encodedBytesNb = p4nd1enc128v16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4nd1dec128v16");
        decodedBytesNb = p4nd1dec128v16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nd1enc128v16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nzenc128v16");
        encodedBytesNb = p4nzenc128v16(inputData16, insize, encoded); Bench::stop();
        Bench::start("p4nzdec128v16");
        decodedBytesNb = p4nzdec128v16(encoded, insize, decoded16); Bench::stop();
        check16(inputData16, decoded16, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nzenc128v16"].emplace_back(0.5f*encodedBytesNb/insize, encodedBytesNb);
    };

    auto simd128_algos_32 = [&](std::string name, std::uint32_t *inputData32, size_t insize, unsigned char *encoded, std::uint32_t *decoded32){

//        std::cout << name << " -> simd128_algos_32\n";
return;
        size_t encodedBytesNb,decodedBytesNb;

        Bench::start("p4nenc128v32");
        encodedBytesNb = p4nenc128v32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4ndec128v32");
        decodedBytesNb = p4ndec128v32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nenc128v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4ndenc128v32");
        encodedBytesNb = p4ndenc128v32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4nddec128v32");
        decodedBytesNb = p4nddec128v32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4ndenc128v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nd1enc128v32");
        encodedBytesNb = p4nd1enc128v32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4nd1dec128v32");
        decodedBytesNb = p4nd1dec128v32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nd1enc128v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

        Bench::start("p4nzenc128v32");
        encodedBytesNb = p4nzenc128v32(inputData32, insize, encoded); Bench::stop();
        Bench::start("p4nzdec128v32");
        decodedBytesNb = p4nzdec128v32(encoded, insize, decoded32); Bench::stop();
        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nzenc128v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);
    };

    auto simd128_algos_64 = [&](std::string name, std::uint64_t *inputData64, size_t insize, unsigned char *encoded, std::uint64_t *decoded64){

//        std::cout << name << " -> simd128_algos_64\n";

        size_t encodedBytesNb,decodedBytesNb;

        Bench::start("p4nenc128v64");
        encodedBytesNb = p4nenc128v64(inputData64, insize, encoded); Bench::stop();
        Bench::start("p4ndec128v64");
        decodedBytesNb = p4ndec128v64(encoded, insize, decoded64); Bench::stop();
        check64(inputData64, decoded64, insize, encodedBytesNb, decodedBytesNb);
        scores[name + "-p4nenc128v64"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);
    };

//    auto simd256_algos_32 = [&](std::string name, std::uint32_t *inputData32, size_t insize, unsigned char *encoded, std::uint32_t *decoded32){

//        std::cout << name << " -> simd256_algos_32\n";

//        size_t encodedBytesNb,decodedBytesNb;

//        Bench::start("p4nenc256v32");
//        encodedBytesNb = p4nenc256v32(inputData32, insize, encoded); Bench::stop();
//        Bench::start("p4ndec256v32");
//        decodedBytesNb = p4ndec256v32(encoded, insize, decoded32); Bench::stop();
//        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
//        scores[name + "-p4nenc256v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

//        Bench::start("p4ndenc256v32");
//        encodedBytesNb = p4ndenc256v32(inputData32, insize, encoded); Bench::stop();
//        Bench::start("p4nddec256v32");
//        decodedBytesNb = p4nddec256v32(encoded, insize, decoded32); Bench::stop();
//        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
//        scores[name + "-p4ndenc256v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

//        Bench::start("p4nd1enc256v32");
//        encodedBytesNb = p4nd1enc256v32(inputData32, insize, encoded); Bench::stop();
//        Bench::start("p4nd1dec256v32");
//        decodedBytesNb = p4nd1dec256v32(encoded, insize, decoded32); Bench::stop();
//        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
//        scores[name + "-p4nd1enc256v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);

//        Bench::start("p4nzenc256v32");
//        encodedBytesNb = p4nzenc256v32(inputData32, insize, encoded); Bench::stop();
//        Bench::start("p4nzdec256v32");
//        decodedBytesNb = p4nzdec256v32(encoded, insize, decoded32); Bench::stop();
//        check32(inputData32, decoded32, insize, encodedBytesNb, decodedBytesNb);
//        scores[name + "-p4nzenc256v32"].emplace_back(0.25f*encodedBytesNb/insize, encodedBytesNb);
//    };


    auto process16 = [&](std::string name, std::vector<std::uint16_t> &inputData16, std::vector<unsigned char> &encoded, std::vector<std::uint16_t> &decoded16){

        size_t beforePaddingSize = inputData16.size();

        size_t insize = inputData16.size();
        encoded.resize(insize*2);
        decoded16.resize(insize);
        algos_16(name, inputData16.data(), insize, encoded.data(), decoded16.data());

        if(inputData16.size()%128 != 0){
            inputData16.resize(inputData16.size() + (128-(inputData16.size()%128)));
            REQUIRE(inputData16.size()%128==0);
            std::fill(inputData16.begin() + beforePaddingSize, inputData16.end(), 0);
            insize = inputData16.size();
            encoded.resize(insize*2);
            decoded16.resize(insize);
        }
        simd128_algos_16(name, inputData16.data(), insize, encoded.data(), decoded16.data());

        if(beforePaddingSize != inputData16.size()){
            inputData16.resize(beforePaddingSize);
        }
    };

    auto process32 = [&](std::string name, std::vector<std::uint32_t> &inputData32, std::vector<unsigned char> &encoded, std::vector<std::uint32_t> &decoded32){

        size_t beforePaddingSize = inputData32.size();
        size_t insize = inputData32.size();
        encoded.resize(insize*4);
        decoded32.resize(insize);
        algos_32(name, inputData32.data(), insize, encoded.data(), decoded32.data());

        if(inputData32.size()%128 != 0){
            inputData32.resize(inputData32.size() + (128-(inputData32.size()%128)));
            REQUIRE(inputData32.size()%128==0);
            std::fill(inputData32.begin() + beforePaddingSize, inputData32.end(), 0);
            insize = inputData32.size();
            encoded.resize(insize*4);
            decoded32.resize(insize);
        }

        simd128_algos_32(name, inputData32.data(), insize, encoded.data(), decoded32.data());

//        if(inputData32.size()%256 != 0){
//            inputData32.resize(inputData32.size() + (256-(inputData32.size()%256)));
//            REQUIRE(inputData32.size()%256==0);
//            std::fill(inputData32.begin() + beforePaddingSize, inputData32.end(), 0);
//            insize = inputData32.size();
//            encoded.resize(insize*4);
//            decoded32.resize(insize);
//        }
//        simd256_algos_32(name, inputData32.data(), insize, encoded.data(), decoded32.data());


//        if(beforePaddingSize != inputData32.size()){
//            inputData32.resize(beforePaddingSize);
//            encoded.resize(inputData32.size()*4+1024);
//        }

//        Bench::start("integerCompressor");
//        size_t sizeCompressed = integerCompressor.encode(
//            inputData32.data(), inputData32.size(),
//            reinterpret_cast<std::uint32_t*>(encoded.data()),  beforePaddingSize + 1024);
//        Bench::stop();
//        REQUIRE(sizeCompressed != 0);
//        REQUIRE(sizeCompressed < inputData32.size());

//        scores[name + "-integerCompressor"].emplace_back(1.f *sizeCompressed/insize, sizeCompressed*4);
    };

    auto process64 = [&](std::string name, std::vector<std::uint64_t> &inputData64, std::vector<unsigned char> &encoded, std::vector<std::uint64_t> &decoded64){

        size_t beforePaddingSize = inputData64.size();
        size_t insize = inputData64.size();
        encoded.resize(insize*8);
        decoded64.resize(insize);
        algos_64(name, inputData64.data(), insize, encoded.data(), decoded64.data());

        if(inputData64.size()%128 != 0){
            inputData64.resize(inputData64.size() + (128-(inputData64.size()%128)));
            REQUIRE(inputData64.size()%128==0);
            std::fill(inputData64.begin() + beforePaddingSize, inputData64.end(), 0);
        }
        simd128_algos_64(name, inputData64.data(), insize, encoded.data(), decoded64.data());

        if(beforePaddingSize != inputData64.size()){
            inputData64.resize(beforePaddingSize);
        }
    };

    std::cout << "start " <<  video.nb_frames(0) << "\n";
    K4FullFrame fFrame;
    for(size_t ii = 0; ii < 79; ++ii){//video.nb_frames(0); ++ii){

        // use only lossless for depth/cloud values
        auto frame = video.get_full_frame(ii,0);

        std::cout << "frame " << ii << " " << frame->colorWidth << " " << frame->colorHeight << "\n";

        Bench::start("uncompress_frame");
        REQUIRE (manager.uncompress_frame(frame, fFrame));
        Bench::stop();


        geo::Pt3<std::int16_t> *cloud = manager.cloud_data();
        auto cloudSize = (frame->depthWidth*frame->depthHeight);

        std::cout << "cloudSize " << cloudSize << " " << frame->depthWidth << " " << frame->depthHeight << "\n";


        std::vector<size_t> indices(cloudSize);
        std::iota(indices.begin(), indices.end(), 0);

        std::vector<size_t> vIndices;
        vIndices.resize(cloudSize);

        size_t idV = 0;
        for(size_t jj = 0; jj < cloudSize; ++jj){
            if(uncompressedDepthFrame[jj] != k4_invalid_depth_value){
                vIndices[idV++] = jj;
            }
        }

        std::vector<std::uint64_t> packedBuffer;
        packedBuffer.resize(cloudSize);

        auto color = reinterpret_cast<geo::Pt4<std::uint8_t>*>(uncompressedColorFrame.data());
        Bench::start("pack");
        for_each(std::execution::par_unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){
            packedBuffer[id] = K4PackedVoxel::pack64(cloud[vIndices[id]], color[vIndices[id]]);
        });
        std::sort(std::execution::par_unseq, std::begin(packedBuffer), std::end(packedBuffer));
        Bench::stop();




        std::vector<std::uint16_t> depth16(cloudSize);
        std::vector<std::uint32_t> depth32(cloudSize);
        std::vector<std::uint32_t> ddepth32(cloudSize);
        std::vector<std::uint64_t> depth64(cloudSize);

        std::vector<std::uint16_t> xyz16, uxyz16;
        std::vector<std::uint32_t> xyz32, uxyz32, dxyz32, duxyz32;
        std::vector<std::uint64_t> xyz64, uxyz64;

        std::vector<std::uint8_t> inputColorData(cloudSize*3);
        std::fill(inputColorData.begin(), inputColorData.end(), 0);

        xyz16.resize(idV*3);
        xyz32.resize(idV*3);
        xyz64.resize(idV*3);

        uxyz16.resize(idV*3);        
        uxyz32.resize(idV*3);
        uxyz64.resize(idV*3);

        dxyz32.resize(idV*3);
        duxyz32.resize(idV*3);

        Bench::start("fill arrays");

        // check cloud values
        bool validValues = true;
        for(size_t jj= 0; jj < cloudSize; ++jj){

            if(uncompressedDepthFrame[jj] < 0) [[unlikely]] {
                validValues = false;
                break;
            }else if(uncompressedDepthFrame[jj] > 10000) [[unlikely]] {
                validValues = false;
                break;
            }
            depth16[jj] = uncompressedDepthFrame[jj];
            depth32[jj] = uncompressedDepthFrame[jj];
            depth64[jj] = uncompressedDepthFrame[jj];
        }
        REQUIRE(validValues);

        validValues = true;
        for_each(std::execution::unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){

            if(cloud[vIndices[id]].x() < -4096) [[unlikely]] {
                validValues = false;
                return;
            }else if(cloud[vIndices[id]].y() < -4096) [[unlikely]] {
                validValues = false;
                return;
            }else if(cloud[vIndices[id]].z() < 0) [[unlikely]] {
                validValues = false;
                return;
            }

            xyz16[id]         = static_cast<std::uint16_t>(static_cast<std::int32_t>(cloud[vIndices[id]].x())+4096);
            xyz16[idV   + id] = static_cast<std::uint16_t>(static_cast<std::int32_t>(cloud[vIndices[id]].y())+4096);
            xyz16[2*idV + id] = static_cast<std::uint16_t>(cloud[vIndices[id]].z());

            xyz32[id]         = static_cast<std::uint32_t>(static_cast<std::int32_t>(cloud[vIndices[id]].x())+4096);
            xyz32[idV   + id] = static_cast<std::uint32_t>(static_cast<std::int32_t>(cloud[vIndices[id]].y())+4096);
            xyz32[2*idV + id] = static_cast<std::uint32_t>(cloud[vIndices[id]].z());

            if(static_cast<std::int32_t>(cloud[vIndices[id]].x()) != static_cast<std::int32_t>(xyz32[id]) - 4096){
                validValues = false;
                return;
            }
            if(static_cast<std::int32_t>(cloud[vIndices[id]].y()) != static_cast<std::int32_t>(xyz32[idV   + id]) - 4096){
                validValues = false;
                return;
            }
            if(static_cast<std::int32_t>(cloud[vIndices[id]].z()) != static_cast<std::int32_t>(xyz32[2*idV + id])){
                validValues = false;
                return;
            }

            xyz64[id]         = static_cast<std::uint64_t>(static_cast<std::int32_t>(cloud[vIndices[id]].x())+4096);
            xyz64[idV   + id] = static_cast<std::uint64_t>(static_cast<std::int32_t>(cloud[vIndices[id]].y())+4096);
            xyz64[2*idV + id] = static_cast<std::uint64_t>(cloud[vIndices[id]].z());

        });
        REQUIRE(validValues);

        for_each(std::execution::par_unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){

            uxyz16[id*3+0] = static_cast<std::uint16_t>(static_cast<std::int32_t>(cloud[vIndices[id]].x())+4096);
            uxyz16[id*3+1] = static_cast<std::uint16_t>(static_cast<std::int32_t>(cloud[vIndices[id]].y())+4096);
            uxyz16[id*3+2] = static_cast<std::uint16_t>(cloud[vIndices[id]].z());

            uxyz32[id*3+0] = static_cast<std::uint32_t>(cloud[vIndices[id]].x()+4096);
            uxyz32[id*3+1] = static_cast<std::uint32_t>(cloud[vIndices[id]].y()+4096);
            uxyz32[id*3+2] = static_cast<std::uint32_t>(cloud[vIndices[id]].z());

            uxyz64[id*3+0] = static_cast<std::uint64_t>(cloud[vIndices[id]].x()+4096);
            uxyz64[id*3+1] = static_cast<std::uint64_t>(cloud[vIndices[id]].y()+4096);
            uxyz64[id*3+2] = static_cast<std::uint64_t>(cloud[vIndices[id]].z());

            auto c = reinterpret_cast<geo::Pt4<std::uint8_t>*>(uncompressedColorFrame.data());
            inputColorData[id*3+0] = c[vIndices[id]].x();
            inputColorData[id*3+1] = c[vIndices[id]].y();
            inputColorData[id*3+2] = c[vIndices[id]].z();
        });

        compute_deltas(xyz32.data(), idV*3, dxyz32.data(), 0);
        compute_deltas(uxyz32.data(), idV*3, duxyz32.data(), 0);
        compute_deltas(depth32.data(), cloudSize, ddepth32.data(), 0);

        Bench::stop();

        Bench::start("check arrays");
        validValues = true;
        for_each(std::execution::par_unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){
            auto x = static_cast<std::int16_t>(static_cast<std::int32_t>(xyz32[id]) - 4096);
            auto y = static_cast<std::int16_t>(static_cast<std::int32_t>(xyz32[idV   + id]) - 4096);
            auto z = static_cast<std::int16_t>(xyz32[2*idV + id]);

            if(x != cloud[vIndices[id]].x()){
                validValues = false;
                return;
            }
            if(y != cloud[vIndices[id]].y()){
                validValues = false;
                return;
            }
            if(z != cloud[vIndices[id]].z()){
                validValues = false;
                return;
            }

        });
        REQUIRE(validValues);

        validValues = true;
        for_each(std::execution::par_unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){
            auto x = static_cast<std::int16_t>(static_cast<std::int32_t>(uxyz32[id*3+0]) - 4096);
            auto y = static_cast<std::int16_t>(static_cast<std::int32_t>(uxyz32[id*3+1]) - 4096);
            auto z = static_cast<std::int16_t>(uxyz32[id*3+2]);
            if(x != cloud[vIndices[id]].x()){
                validValues = false;
                return;
            }
            if(y != cloud[vIndices[id]].y()){
                validValues = false;
                return;
            }
            if(z != cloud[vIndices[id]].z()){
                validValues = false;
                return;
            }
        });
        REQUIRE(validValues);
        Bench::stop();

        // data
        std::vector<unsigned char> encoded;
        std::vector<uint16_t> decoded16;
        std::vector<uint32_t> decoded32;
        std::vector<uint64_t> decoded64;

        // process
        process16("depth16", depth16, encoded, decoded16);
        process16("xyz16", xyz16, encoded, decoded16);
        process16("uxyz16", uxyz16, encoded, decoded16);
        process32("depth32", depth32, encoded, decoded32);
        process32("xyz32", xyz32, encoded, decoded32);

        process32("uxyz32", uxyz32, encoded, decoded32);
        process32("dxyz32", dxyz32, encoded, decoded32);
        process32("duxyz32", duxyz32, encoded, decoded32);

        process64("xyz64", xyz64, encoded, decoded64);
        process64("uxyz64", uxyz64, encoded, decoded64);
        process64("packed", packedBuffer, encoded, decoded64);

        // compress color
        graphics::Texture uncompressedTexture;


        std::vector<std::uint8_t> uncompressColorData;
        uncompressColorData.resize(inputColorData.size());

        int jpegQuality = 80;
        long unsigned int jpegColorSize = 0;
        if(tjCompressedImage == nullptr){
            tjCompressedImage = tjAlloc(int(frame->colorWidth*frame->colorHeight*3));
        }

        int ret = tjCompress2(jpegCompressor,
            reinterpret_cast<const unsigned char*>(inputColorData.data()),
            int(frame->colorWidth), 0, int(frame->colorHeight),
            TJPF_BGR,
            &tjCompressedImage, &jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
        scores["TJSAMP_444-jpeg80"].emplace_back(0.25f*jpegColorSize/(frame->colorWidth*frame->colorHeight*3), jpegColorSize);

        ret = tjDecompress2(jpegUncompressor, tjCompressedImage, jpegColorSize, uncompressColorData.data(),
                            int(frame->colorWidth), 0, int(frame->colorHeight),TJPF_BGR, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
//        uncompressedTexture.copy_2d_data(frame->colorWidth, frame->colorHeight, 3, uncompressColorData.data());
//        uncompressedTexture.write_2d_image_file_data(std::format("E:/TJSAMP_444-jpeg80-{}.jpg", ii));

        jpegQuality = 60;
        ret = tjCompress2(jpegCompressor,
            reinterpret_cast<const unsigned char*>(inputColorData.data()),
            int(frame->colorWidth), 0, int(frame->colorHeight),
            TJPF_BGR,
            &tjCompressedImage, &jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
        scores["TJSAMP_444-jpeg60"].emplace_back(0.25f*jpegColorSize/(frame->colorWidth*frame->colorHeight*3), jpegColorSize);

        ret = tjDecompress2(jpegUncompressor, tjCompressedImage, jpegColorSize, uncompressColorData.data(),
                            int(frame->colorWidth), 0, int(frame->colorHeight),TJPF_BGR, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
//        uncompressedTexture.copy_2d_data(frame->colorWidth, frame->colorHeight, 3, uncompressColorData.data());
//        uncompressedTexture.write_2d_image_file_data(std::format("E:/TJSAMP_444-jpeg60-{}.jpg", ii));


        jpegQuality = 50;
        ret = tjCompress2(jpegCompressor,
            reinterpret_cast<const unsigned char*>(inputColorData.data()),
            int(frame->colorWidth), 0, int(frame->colorHeight),
            TJPF_BGR,
            &tjCompressedImage, &jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
        scores["TJSAMP_444-jpeg50"].emplace_back(0.25f*jpegColorSize/(frame->colorWidth*frame->colorHeight*3), jpegColorSize);
        ret = tjDecompress2(jpegUncompressor, tjCompressedImage, jpegColorSize, uncompressColorData.data(),
                            int(frame->colorWidth), 0, int(frame->colorHeight),TJPF_BGR, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
//        uncompressedTexture.copy_2d_data(frame->colorWidth, frame->colorHeight, 3, uncompressColorData.data());
//        uncompressedTexture.write_2d_image_file_data(std::format("E:/TJSAMP_444-jpeg50-{}.jpg", ii));

        ret = tjCompress2(jpegCompressor,
            reinterpret_cast<const unsigned char*>(inputColorData.data()),
            int(frame->colorWidth), 0, int(frame->colorHeight),
            TJPF_BGR,
            &tjCompressedImage, &jpegColorSize, TJSAMP_422, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
        scores["TJSAMP_422-jpeg80"].emplace_back(0.25f*jpegColorSize/(frame->colorWidth*frame->colorHeight*3), jpegColorSize);
        ret = tjDecompress2(jpegUncompressor, tjCompressedImage, jpegColorSize, uncompressColorData.data(),
                            int(frame->colorWidth), 0, int(frame->colorHeight),TJPF_BGR, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
//        uncompressedTexture.copy_2d_data(frame->colorWidth, frame->colorHeight, 3, uncompressColorData.data());
//        uncompressedTexture.write_2d_image_file_data(std::format("E:/TJSAMP_422-jpeg80-{}.jpg", ii));

        jpegQuality = 60;
        ret = tjCompress2(jpegCompressor,
            reinterpret_cast<const unsigned char*>(inputColorData.data()),
            int(frame->colorWidth), 0, int(frame->colorHeight),
            TJPF_BGR,
            &tjCompressedImage, &jpegColorSize, TJSAMP_422, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
        scores["TJSAMP_422-jpeg60"].emplace_back(0.25f*jpegColorSize/(frame->colorWidth*frame->colorHeight*3), jpegColorSize);
        ret = tjDecompress2(jpegUncompressor, tjCompressedImage, jpegColorSize, uncompressColorData.data(),
                            int(frame->colorWidth), 0, int(frame->colorHeight),TJPF_BGR, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
//        uncompressedTexture.copy_2d_data(frame->colorWidth, frame->colorHeight, 3, uncompressColorData.data());
//        uncompressedTexture.write_2d_image_file_data(std::format("E:/TJSAMP_422-jpeg60-{}.jpg", ii));

        jpegQuality = 50;
        ret = tjCompress2(jpegCompressor,
            reinterpret_cast<const unsigned char*>(inputColorData.data()),
            int(frame->colorWidth), 0, int(frame->colorHeight),
            TJPF_BGR,
            &tjCompressedImage, &jpegColorSize, TJSAMP_422, jpegQuality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
        scores["TJSAMP_422-jpeg50"].emplace_back(0.25f*jpegColorSize/(frame->colorWidth*frame->colorHeight*3), jpegColorSize);
        ret = tjDecompress2(jpegUncompressor, tjCompressedImage, jpegColorSize, uncompressColorData.data(),
                            int(frame->colorWidth), 0, int(frame->colorHeight),TJPF_BGR, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
        REQUIRE(ret != -1);
//        uncompressedTexture.copy_2d_data(frame->colorWidth, frame->colorHeight, 3, uncompressColorData.data());
//        uncompressedTexture.write_2d_image_file_data(std::format("E:/TJSAMP_422-jpeg50-{}.jpg", ii));


        for_each(std::execution::unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){

        });

//        REQUIRE(files::CloudIO::save_cloud(std::format("E:/boring_cloud_{}_1.obj",ii), pts.data(), pts.size()));

//        continue;



//        REQUIRE(files::CloudIO::save_cloud<std::int16_t>(std::format("E:/boring_cloud_{}_2.obj", ii), cloud, pts.size()));
    }



    Bench::display(BenchUnit::microseconds);

    std::vector<std::pair<std::string, double>> results;
    for(auto &m : scores){
        double total = 0.;
        size_t totalB = 0;
        for(auto &f : m.second){
            total += std::get<0>(f);
            totalB +=  std::get<1>(f);
        }
        double res = ((1.*totalB/m.second.size())/1024)/1024;
        results.emplace_back(m.first, res);
//        std::cout << "mode : " << m.first << " " << total/m.second.size()<< " " << ((1.*totalB/m.second.size())/1024)/1024 << "\n";
    }

   std::sort(std::begin(results), std::end(results), [](const auto &p1, const auto &p2) {
        return p1.second < p2.second;
    });

    for(const auto &res : results){
       std::cout << res.first << " " << res.second << "\n";
   }

    if(tjCompressedImage != nullptr){
        tjFree(tjCompressedImage);
    }


    tjDestroy(jpegUncompressor);
    tjDestroy(jpegCompressor);


}
