
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
#include "camera/kinect4_utility.hpp"
#include "utility/logger.hpp"
#include "utility/benchmark.hpp"
#include "data/integers_encoder.hpp"
#include "files/cloud_io.hpp"

using namespace tool;
using namespace tool::camera::K4;




int delta() {

    uint32_t datain[] = {1,14,15,25,100};
    int N = sizeof(datain) / sizeof(uint32_t);
    printf("starting from: ");
    for(int k = 0; k < N; ++k) printf(" %d ",datain[k]);
    printf("\n");

    compute_deltas_inplace(datain,N,0);

    printf("deltas: ");
    for(int k = 0; k < N; ++k) printf(" %d ",datain[k]);
    printf("\n");

    compute_prefix_sum_inplace(datain,N,0);


    printf("recovered: ");
    for(int k = 0; k < N; ++k) printf(" %d ",datain[k]);
    printf("\n");

    // you can also, similarly, write values to another buffer with compute_deltas, and compute_prefix_sum
    return 0;
}


int delta(std::uint32_t *inData, size_t inSize, std::vector<std::uint32_t> &deltas) {


    deltas.resize(inSize);

//    compute_deltas_inplace(inData,inSize,0);
    compute_deltas(inData, inSize, deltas.data(), 0);

//    compute_prefix_sum_inplace(inData,inSize,0);

    // you can also, similarly, write values to another buffer with compute_deltas, and compute_prefix_sum
    return 0;
}


//int compress_decompress_demo() {
//    size_t N = 9999;
//    __m128i * endofbuf;
//    int howmanybytes;
//    float compratio;
//    std::vector<uint32_t> datain(N);
//    std::vector<uint32_t> backbuffer(N);

//    for (size_t k = 0; k < N; ++k) {       /* start with k=0, not k=1! */
//        datain[k] = k;
//    }

//    uint32_t b;
//    b = maxbits_length(datain.data(), N);



//    std::vector<uint8_t> buffer(simdpack_compressedbytes(N,b));

//    endofbuf     = simdpack_length(datain.data(), N, reinterpret_cast<__m128i *>(buffer.data()), b);
//    howmanybytes = (endofbuf-(__m128i *)buffer.data())*sizeof(__m128i); /* number of compressed bytes */
//    compratio    = N*sizeof(uint32_t) * 1.0 / howmanybytes;
//    /* endofbuf points to the end of the compressed data */

//    buffer.resize((endofbuf-reinterpret_cast<__m128i *>(buffer.data()))*sizeof(__m128i));
//    //buffer = realloc(buffer,(endofbuf-(__m128i *)buffer.data())*sizeof(__m128i)); /* optional but safe. */

//    printf("Compressed %d integers down to %d bytes (comp. ratio = %f).\n",(int)N,howmanybytes,compratio);
//    /* in actual applications b must be stored and retrieved: caller is responsible for that. */
//    simdunpack_length(reinterpret_cast<const __m128i *>(buffer.data()), N, backbuffer.data(), b); /* will return a pointer to endofbuf */

//    for (size_t k = 0; k < N; ++k) {
//        if(datain[k] != backbuffer[k]) {
//            printf("bug at %lu \n",(unsigned long)k);
//            return -1;
//        }
//    }
//    printf("Code works!\n");
//    //    free(datain);
//    //    free(buffer);
//    //    free(backbuffer);
//    return 0;
//}

//int compress_decompress_demo(std::uint32_t *inData, size_t inSize, std::vector<uint8_t> &compressedBuffer) {

//    // count nb bits
//    uint32_t maxNbBits = maxbits_length(inData, inSize);

//    // resize buffer
//    compressedBuffer.resize(simdpack_compressedbytes(inSize,maxNbBits));

//    // compress
//    __m128i * endofbuf = simdpack_length(inData, inSize, reinterpret_cast<__m128i *>(compressedBuffer.data()), maxNbBits);

//    // nb of compressed bytes
//    int howmanybytes   = (endofbuf-(__m128i *)compressedBuffer.data())*sizeof(__m128i);

//    // compress ratio
//    float compratio    = inSize*sizeof(uint32_t) * 1.0 / howmanybytes;

//    // resize buffer to end of buf size
//    compressedBuffer.resize((endofbuf-reinterpret_cast<__m128i *>(compressedBuffer.data()))*sizeof(__m128i));

//    printf("Compressed %d integers down to %d bytes (comp. ratio = %f).\n",(int)inSize,howmanybytes,compratio);

//    /* in actual applications b must be stored and retrieved: caller is responsible for that. */
//    std::vector<uint32_t> backbuffer(inSize);
//    simdunpack_length(reinterpret_cast<const __m128i *>(compressedBuffer.data()), inSize, backbuffer.data(), maxNbBits); /* will return a pointer to endofbuf */

//    for (size_t k = 0; k < inSize; ++k) {
//        if(inData[k] != backbuffer[k]) {
//            printf("bug at %lu \n",(unsigned long)k);
//            return -1;
//        }
//    }
//    printf("Code works!\n");

//    return 0;
//}

TEST_CASE("Kinect4 camera"){

    REQUIRE(delta()==0);

//    for(auto &code : FastPForLib::CODECFactory::allNames()){
//        std::cout << code << "\n";
//    }


//    REQUIRE(compress_decompress_demo()==0);
//    return;

    VolumetricVideoResource video;

    const std::string filePath = "E:/ttt.kvid";
    REQUIRE(video.load_from_file(filePath));

    VolumetricVideoManager manager(&video);

    auto jpegCompressor = tjInitCompress();
    tjhandle jpegUncompressor = tjInitDecompress();

    unsigned char *tjCompressedImage = nullptr;
    unsigned char *tjCompressedImage2 = nullptr;

    data::IntegersEncoder integerCompressor;

    std::vector<std::uint8_t> uncompressedColorFrame;
    std::vector<std::uint16_t> uncompressedDepthFrame;

    for(size_t ii = 0; ii < video.nb_frames(0); ++ii){ // 1; ++ii){//


        // use only lossless for depth/cloud values

        auto frame = video.get_frame(ii,0);

        Bench::start("uncompress_color1");
        REQUIRE (manager.uncompress_color(frame,  uncompressedColorFrame));
        Bench::stop();

        Bench::start("uncompress_depth1");
        REQUIRE(manager.uncompress_depth(frame,  uncompressedDepthFrame));
        Bench::stop();

        Bench::start("generate_cloud");
        manager.generate_cloud(frame, uncompressedDepthFrame);
        Bench::stop();


        geo::Pt3<std::int16_t> *cloud = manager.cloud_data();
        auto cloudSize = (frame->depthWidth*frame->depthHeight);

        std::vector<size_t> indices(cloudSize);
        std::iota(indices.begin(), indices.end(), 0);

        std::vector<size_t> vIndices;
        vIndices.resize(cloudSize);

        size_t idV = 0;
        for(size_t jj = 0; jj < cloudSize; ++jj){
            if(uncompressedDepthFrame[jj] != invalid_depth_value){
                vIndices[idV++] = jj;
            }
        }

        std::vector<std::uint64_t> packedBuffer;
        packedBuffer.resize(cloudSize);

        auto color = reinterpret_cast<geo::Pt4<std::uint8_t>*>(uncompressedColorFrame.data());
        Bench::start("pack");
            for_each(std::execution::par_unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){
                packedBuffer[id] = PackedVoxel::pack64(cloud[vIndices[id]], color[vIndices[id]]);
            });
        Bench::stop();

        std::vector<std::uint16_t> depth16(cloudSize);
        std::vector<std::uint32_t> depth(cloudSize);
        for(size_t jj= 0; jj < cloudSize; ++jj){
            depth[jj] = uncompressedDepthFrame[jj];
            depth16[jj] = uncompressedDepthFrame[jj];
        }

        std::vector<std::uint32_t> xyz, uxyz;
        std::vector<std::uint16_t> xyz16, uxyz16;
        xyz.resize(idV*3);
        xyz16.resize(idV*3);

        uxyz.resize(idV*3);
        uxyz16.resize(idV*3);

        Bench::start("fill");
        for_each(std::execution::par_unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){
            xyz[id]         = static_cast<std::uint32_t>(static_cast<std::int32_t>(cloud[vIndices[id]].x())+4096);
            xyz[idV   + id] = static_cast<std::uint32_t>(static_cast<std::int32_t>(cloud[vIndices[id]].y())+4096);
            xyz[2*idV + id] = static_cast<std::uint32_t>(static_cast<std::int32_t>(cloud[vIndices[id]].z()));

            xyz16[id]         = cloud[vIndices[id]].x();
            xyz16[idV   + id] = cloud[vIndices[id]].y();
            xyz16[2*idV + id] = cloud[vIndices[id]].z();
        });
        for_each(std::execution::par_unseq, std::begin(indices), std::begin(indices) + idV, [&](size_t id){
            uxyz[id*3+0] = static_cast<std::uint32_t>(cloud[vIndices[id]].x()+4096);
            uxyz[id*3+1] = static_cast<std::uint32_t>(cloud[vIndices[id]].y()+4096);
            uxyz[id*3+2] = static_cast<std::uint32_t>(cloud[vIndices[id]].z());

            uxyz16[id*3+0] = cloud[vIndices[id]].x();
            uxyz16[id*3+1] = cloud[vIndices[id]].y();
            uxyz16[id*3+2] = cloud[vIndices[id]].z();
        });
        Bench::stop();

//        continue;
        std::vector<unsigned char> out;



        auto insize = cloudSize;
        out.resize(insize*2);
        auto outsize = p4nenc128v16(depth16.data(), insize, out.data());
        std::cout << "p4nenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nenc128v16(depth16.data(), insize, out.data());
        std::cout << "p4nenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nd1enc128v16(depth16.data(), insize, out.data());
        std::cout << "p4nd1enc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nzenc128v16(depth16.data(), insize, out.data());
        std::cout << "p4nzenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n\n";

        // NOT THE GOOD INPUT SIZE
        // NEED PADDING

        xyz16.resize(xyz16.size() + (128-(xyz16.size()%128)));
        std::cout << "MOD " << xyz16.size()%128 << "\n";

        insize = xyz16.size();
        out.resize(insize*2);
        outsize = p4nenc128v16(xyz16.data(), insize, out.data());
        std::cout << "xyz16 p4nenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nenc128v16(xyz16.data(), insize, out.data());
        std::cout << "xyz16 p4nenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nd1enc128v16(xyz16.data(), insize, out.data());
        std::cout << "xyz16 p4nd1enc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nzenc128v16(xyz16.data(), insize, out.data());
        std::cout << "xyz16 p4nzenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n\n";

        std::vector<std::uint16_t> bxyz16(xyz16.size());
        auto size2 = p4nzdec128v16(out.data(), outsize, bxyz16.data());

        std::cout << "size2 " << size2 << " " << insize << "\n";
        for(size_t jj = 0; jj < bxyz16.size(); ++jj){
            if(bxyz16[jj] != xyz16[jj]){
                std::cout << "JJ " << jj << "\n";
            }
            REQUIRE(bxyz16[jj] == xyz16[jj]);
        }

        uxyz16.resize(uxyz16.size() + (128-(uxyz16.size()%128)));
        std::cout << "MOD " << uxyz16.size()%128 << "\n";

        insize = uxyz16.size();
        out.resize(insize*2);
        outsize = p4nenc128v16(uxyz16.data(), insize, out.data());
        std::cout << "uxyz16 p4nenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nenc128v16(uxyz16.data(), insize, out.data());
        std::cout << "uxyz16 p4nenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nd1enc128v16(uxyz16.data(), insize, out.data());
        std::cout << "uxyz16 p4nd1enc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n";
        outsize = p4nzenc128v16(uxyz16.data(), insize, out.data());
        std::cout << "uxyz16 p4nzenc128v16 " << insize << " " << outsize << " " << (0.5f*outsize/insize)  << "\n\n";


        // DECODE / TEST


//        out.resize(xyz16.size()*2);
//        outsize = p4nenc128v16(xyz16.data(), xyz16.size(), out.data());
//        std::cout << "xyz16 p4nenc128v16 " << xyz16.size() << " " << outsize << " " << (1.f*outsize/xyz16.size())  << "\n";
//        outsize = p4nenc128v16(xyz16.data(), xyz16.size(), out.data());
//        std::cout << "xyz16 p4nenc128v16 " << xyz16.size() << " " << outsize << " " << (1.f*outsize/xyz16.size())  << "\n";
//        outsize = p4nd1enc128v16(xyz16.data(), xyz16.size(), out.data());
//        std::cout << "xyz16 p4nd1enc128v16 " << xyz16.size() << " " << outsize << " " << (1.f*outsize/xyz16.size())  << "\n";
//        outsize = p4nzenc128v16(xyz16.data(), xyz16.size(), out.data());
//        std::cout << "xyz16 p4nzenc128v16 " << xyz16.size() << " " << outsize << " " << (1.f*outsize/xyz16.size())  << "\n\n";






        // Get the memory space required to encode the arrays.
//        size_t ints_space = oroch::varint_codec<int>::space(depth16.begin(), depth16.end());

//        std::cout << "ints_space: " << ints_space << "\n";
        continue;

        Bench::start("integerCompressor_depth");
        {
            std::vector<std::uint32_t> cBuffer(depth.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(depth.data()), depth.size(),
                cBuffer.data(), cBuffer.size()
            );
            std::cout << "integerCompressor_depth " << depth.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(depth.size()) << "\n";
        }
        Bench::stop();

        Bench::start("integerCompressor_depth16");
        {
            std::vector<std::uint32_t> cBuffer(depth16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(depth16.data()), depth16.size()/2,
                cBuffer.data(), cBuffer.size()
            );
            std::cout << "integerCompressor_depth16 " << (depth16.size()/2) << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(depth16.size()/2) << "\n";
        }
        Bench::stop();

        return;

        Bench::start("packedBuffer");
        {
            std::vector<std::uint32_t> cBuffer(packedBuffer.size()*2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                packedBuffer.data(), packedBuffer.size(),
                cBuffer.data(), cBuffer.size()
            );
            std::cout << "packedBuffer " << packedBuffer.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(packedBuffer.size()) << "\n";
        }
        Bench::stop();


        Bench::start("integerCompressor_xyz");
        {
            std::vector<std::uint32_t> cBuffer(xyz.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(xyz.data()), xyz.size(),
                cBuffer.data(), cBuffer.size()
            );
            std::cout << "integerCompressor_xyz " << xyz.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(xyz.size()) << "\n";
        }
        Bench::stop();

        Bench::start("integerCompressor_xyz16");
        {
            std::vector<std::uint32_t> cBuffer(xyz16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(xyz16.data()), xyz16.size()/2,
                cBuffer.data(), cBuffer.size()
            );
            std::cout << "integerCompressor_xyz16 " << xyz16.size()/2 << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(xyz16.size()/2) << "\n";
        }
        Bench::stop();

        Bench::start("integerCompressor_uxyz");
        {
            std::vector<std::uint32_t> cBuffer(uxyz.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(uxyz.data()), uxyz.size(),
                cBuffer.data(), cBuffer.size()
            );
            std::cout << "integerCompressor_uxyz " << uxyz.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(uxyz.size()) << "\n";
        }
        Bench::stop();

        Bench::start("integerCompressor_uxyz16");
        {
            std::vector<std::uint32_t> cBuffer(uxyz16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(uxyz16.data()), uxyz16.size()/2,
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "integerCompressor_uxyz16 " << uxyz16.size()/2 << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(uxyz16.size()/2) << "\n";
        }
        Bench::stop();


        compute_deltas_inplace(xyz.data(),idV*3,0);
        compute_deltas_inplace(uxyz.data(),idV*3,0);

        compute_deltas_inplace(depth.data(),cloudSize,0);
        compute_deltas_inplace(reinterpret_cast<std::uint32_t*>(depth16.data()),cloudSize/2,0);


        for (size_t jj = 1; jj < idV*3; ++jj) {
            xyz16[jj] += xyz16[jj-1];
        }
        for (size_t jj = 1; jj < idV*3; ++jj) {
            uxyz16[jj] += uxyz16[jj-1];
        }
        for (size_t jj = 1; jj < idV; ++jj) {
            packedBuffer[jj] += packedBuffer[jj-1];
        }

        Bench::start("delta_integerCompressor_depth");
        {
            std::vector<std::uint32_t> cBuffer(depth.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(depth.data()), depth.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "delta_integerCompressor_depth " << depth.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(depth.size()) << "\n";
        }
        Bench::stop();

        Bench::start("delta_integerCompressor_depth16");
        {
            std::vector<std::uint32_t> cBuffer(depth16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(depth16.data()), depth16.size()/2,
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "delta_integerCompressor_depth16 " << (depth16.size()/2) << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(depth16.size()/2) << "\n";
        }
        Bench::stop();



        Bench::start("delta_integerCompressor_xyz");
        {
            std::vector<std::uint32_t> cBuffer(xyz.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(xyz.data()), xyz.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "delta_integerCompressor_xyz " << xyz.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(xyz.size()) << "\n";
        }
        Bench::stop();

        Bench::start("delta_integerCompressor_xyz16");
        {
            std::vector<std::uint32_t> cBuffer(xyz16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(xyz16.data()), xyz16.size()/2,
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "delta_integerCompressor_xyz16 " << xyz16.size()/2 << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(xyz16.size()/2) << "\n";
        }
        Bench::stop();

        Bench::start("delta_integerCompressor_uxyz");
        {
            std::vector<std::uint32_t> cBuffer(uxyz.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(uxyz.data()), uxyz.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "delta_integerCompressor_uxyz " << uxyz.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(uxyz.size()) << "\n";
        }
        Bench::stop();

        Bench::start("delta_integerCompressor_uxyz16");
        {
            std::vector<std::uint32_t> cBuffer(uxyz16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(uxyz16.data()), uxyz16.size()/2,
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "delta_integerCompressor_uxyz16 " << uxyz16.size()/2 << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(uxyz16.size()/2) << "\n";
        }
        Bench::stop();

        Bench::start("delta_packedBuffer");
        {
            std::vector<std::uint32_t> cBuffer(packedBuffer.size()*2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                packedBuffer.data(), packedBuffer.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "delta_packedBuffer " << packedBuffer.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(packedBuffer.size()) << "\n";
        }
        Bench::stop();

        std::sort(std::execution::par_unseq, std::begin(depth), std::end(depth));
        std::sort(std::execution::par_unseq, std::begin(depth16), std::end(depth16));

        std::sort(std::execution::par_unseq, std::begin(packedBuffer), std::end(packedBuffer));
        std::sort(std::execution::par_unseq, std::begin(xyz), std::end(xyz));
        std::sort(std::execution::par_unseq, std::begin(uxyz), std::end(uxyz));
        std::sort(std::execution::par_unseq, std::begin(xyz16), std::end(xyz16));
        std::sort(std::execution::par_unseq, std::begin(uxyz16), std::end(uxyz16));

        Bench::start("sorted_delta_integerCompressor_depth");
        {
            std::vector<std::uint32_t> cBuffer(depth.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(depth.data()), depth.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "sorted_delta_integerCompressor_depth " << depth.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(depth.size()) << "\n";
        }
        Bench::stop();

        Bench::start("sorted_delta_integerCompressor_depth16");
        {
            std::vector<std::uint32_t> cBuffer(depth16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(depth16.data()), depth16.size()/2,
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "sorted_delta_integerCompressor_depth16 " << (depth16.size()/2) << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(depth16.size()/2) << "\n";
        }
        Bench::stop();


        Bench::start("sorted_delta_integerCompressor_xyz");
        {
            std::vector<std::uint32_t> cBuffer(xyz.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(xyz.data()), xyz.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "sorted_delta_integerCompressor_xyz " << xyz.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(xyz.size()) << "\n";
        }
        Bench::stop();

        Bench::start("sorted_delta_integerCompressor_xyz16");
        {
            std::vector<std::uint32_t> cBuffer(xyz16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(xyz16.data()), xyz16.size()/2,
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "sorted_delta_integerCompressor_xyz16 " << xyz16.size()/2 << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(xyz16.size()/2) << "\n";
        }
        Bench::stop();

        Bench::start("sorted_delta_integerCompressor_uxyz");
        {
            std::vector<std::uint32_t> cBuffer(uxyz.size()+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(uxyz.data()), uxyz.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "sorted_delta_integerCompressor_uxyz " << uxyz.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(uxyz.size()) << "\n";
        }
        Bench::stop();

        Bench::start("sorted_delta_integerCompressor_uxyz16");
        {
            std::vector<std::uint32_t> cBuffer(uxyz16.size()/2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                reinterpret_cast<uint32_t*>(uxyz16.data()), uxyz16.size()/2,
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "sorted_delta_integerCompressor_uxyz16 " << uxyz16.size()/2 << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(uxyz16.size()/2) << "\n";
        }
        Bench::stop();

        Bench::start("sorted_delta_packedBuffer");
        {
            std::vector<std::uint32_t> cBuffer(packedBuffer.size()*2+1024);
            size_t sizeCompressed = integerCompressor.encode(
                packedBuffer.data(), packedBuffer.size(),
                cBuffer.data(), cBuffer.size()
                );
            std::cout << "sorted_delta_packedBuffer " << packedBuffer.size() << " " << (sizeCompressed) << " " <<
                (1.f*sizeCompressed)/(packedBuffer.size()) << "\n";
        }
        Bench::stop();


//        compute_deltas_inplace(reinterpret_cast<std::uint32_t*>(xyz16.data()),(idV*3)/2,0);





        continue;

////        //        REQUIRE(encoded != nullptr);


//        std::cout << "alloc\n";

//        if(tjCompressedImage2 == nullptr){
//            tjCompressedImage2 = tjAlloc(frame->depthWidth*frame->depthHeight*4);
//        }


//        std::cout << "compress\n";
//        REQUIRE(tjCompress2(jpegCompressor,
//            reinterpret_cast<const unsigned char*>(ppos.data()),
//            frame->depthWidth, 0, frame->depthHeight,
//            TJPF_RGBA,
//            &tjCompressedImage2, &jpegColorSize, TJSAMP_444, 95, TJFLAG_NOREALLOC | TJFLAG_FASTDCT) == 0);

//        std::cout << "jpegColorSize " << (1.f*jpegColorSize)/(ppos.size()*4) << "\n";

//        std::vector<geo::Pt4<std::uint8_t>> r;
//        r.resize(ppos.size());

//        // uncompress
//        int res = (tjDecompress2(
//            jpegUncompressor,
//            tjCompressedImage2,
//            jpegColorSize,
//            reinterpret_cast<unsigned char*>(r.data()),
//            frame->depthWidth,
//            0, // pitch
//            frame->depthHeight,
//            TJPF_RGBA,
//            TJFLAG_NOREALLOC | TJFLAG_FASTDCT// | TJFLAG_FASTUPSAMPLE
//        )==0);

////        REQUIRE(res == 0);

//        if(res != 0){
//            std::cout << tjGetErrorStr2(jpegUncompressor) << "\n";
//        }

//        size_t count = 0;
//        size_t diff = 0;
//        for(size_t jj = 0; jj < ppos.size(); ++jj){
//            if(ppos[jj].x() != r[jj].x()){
//                count++;
//                diff += std::abs(ppos[jj].x()-r[jj].x());
////                std::cout << (int)ppos[jj].x() <<  " " << (int)r[jj].x() << " | ";
//            }else{
////                std::cout << (int)ppos[jj].x() <<  " " << (int)r[jj].x() << " | ";
//            }
//        }
//        std::cout << "count " <<ppos.size() << " "<<  count << " diff " << diff << "\n";
////        return;
//        std::cout << "save\n";
//        graphics::Texture t2;
//        t2.copy_2d_data(frame->depthWidth, frame->depthHeight, 4, reinterpret_cast<std::uint8_t*>(r.data()));
//        REQUIRE(t2.write_2d_image_file_data(std::format("E:/ppos_{}_2.png", ii)));




////        std::vector<uint8_t> r2;
////        r2.resize(ppos.size()*4);

//        // uncompress
//        int res = (tjDecompress2(
//            jpegCompressor,
//            tjCompressedImage2,
//            jpegColorSize,
//            reinterpret_cast<unsigned char*>(r.data()),
//            frame->depthWidth,
//            0, // pitch
//            frame->depthHeight,
//            TJPF_RGBA,
//            TJFLAG_NOREALLOC | TJFLAG_FASTDCT// | TJFLAG_FASTUPSAMPLE
//        )==0);
//        if(res != 0){
//            std::cout << tjGetErrorStr2(jpegCompressor) << "\n";
//        }

//        for(size_t jj = 0; jj < r.size(); ++jj){
//            REQUIRE((int)r[jj].x() == (int)ppos[jj].x() );
//        }

//        REQUIRE(res == 0);

//        t1.copy_2d_data(frame->depthWidth, frame->depthHeight, 4, reinterpret_cast<std::uint8_t*>(r.data()));
//        REQUIRE(t1.write_2d_image_file_data(std::format("E:/ppos_{}_2.png", ii)));



        //graphics::Texture t1;
        //t1.copy_2d_data(frame->colorWidth, frame->colorHeight, 4, uncompressedColorFrames[ii].data());
        //REQUIRE(t1.write_2d_image_file_data(std::format("E:/boring_img_{}_1.png", ii)));

//        qoi_desc qoid;
//        qoid.channels   = 4;
//        qoid.width      = frame->colorWidth;
//        qoid.height     = frame->colorHeight;
//        qoid.colorspace = QOI_SRGB;

//        int size;
//        Bench::start("compress_color2");
//        auto encoded = reinterpret_cast<std::uint8_t*>(qoi_encode(uncompressedColorFrame.data(), &qoid, &size));
//        Bench::stop();
//        REQUIRE(encoded != nullptr);

//        Bench::start("uncompress_color2");
//        auto decoded = reinterpret_cast<std::uint8_t*>(qoi_decode(encoded, size, &qoid, 4));
//        Bench::stop();

//        REQUIRE(decoded != nullptr);


//        Logger::message(std::format("color w_{} h_{} us_{} tjpgs_{} tjpgr_{} qois_{} qoir_{}\n",
//            frame->colorWidth,
//            frame->colorHeight,
//            uncompressedColorFrame.size(),
//            jpegColorSize,
//            1.f*jpegColorSize/uncompressedColorFrame.size(),
//            size,
//            1.f*size/uncompressedColorFrame.size()));

//        bool equal = true;
//        for(size_t jj = 0; jj < frame->colorWidth*frame->colorHeight; ++jj){
//            if(decoded[jj] != uncompressedColorFrame[jj]){
//                equal = false;
//                break;
//            }
//        }
//        REQUIRE(equal);

//        Bench::start("clean2");
//        delete encoded;
//        delete decoded;
//        Bench::stop();




//        Bench::start("simdcomp");
//        std::vector<std::uint8_t> cBuffer;
//        REQUIRE(compress_decompress_demo(
//            reinterpret_cast<std::uint32_t*>(uncompressedDepthFrame.data()),
//            uncompressedDepthFrame.size()/2, cBuffer) == 0);
//        Bench::stop();

        PackedVoxel p;
        // create 2x size buffer
        // copy packed integer

//        std::vector<std::uint32_t> inData;
//        inData.resize(uncompressedDepthFrame.size()/2);
//        std::copy(
//            reinterpret_cast<std::uint32_t*>(uncompressedDepthFrame.data()),
//            reinterpret_cast<std::uint32_t*>(uncompressedDepthFrame.data()) + uncompressedDepthFrame.size()/2, inData.begin());
//        std::sort(inData.begin(), inData.end());

//        std::vector<std::uint32_t> deltas;
//        delta(inData.data(), inData.size(), deltas);


//        for(size_t jj = 0; jj < inData.size(); ++jj){
////            if(inData[jj] != 0){

//                std::cout << deltas[jj] << " ";
////            }
//        }

//        break;

//        // init buffer
//        auto depthSize = frame->depthWidth*frame->depthHeight/2;
//        std::vector<std::uint32_t> depthBuffer;
//        depthBuffer.resize(depthSize + 1024);

//        // fill buffer
//        std::fill(std::begin(depthBuffer), std::end(depthBuffer), 0);


//        // encode
//        Bench::start("encode_depth1");
//        size_t sizeDepthCompressed = integerCompressor.encode(
//            reinterpret_cast<uint32_t*>(uncompressedDepthFrame.data()), depthSize,
//            depthBuffer.data(), depthBuffer.size()
//        );
//        Bench::stop();
//        REQUIRE(sizeDepthCompressed != 0);





//        std::vector<std::uint64_t> packedBuffer;
//        packedBuffer.resize(cloudSize);


//        size_t idV = 0;
//        size_t countInvalid1  = 0;
//        size_t countInvalid2  = 0;
//        size_t countInvalid3  = 0;
//        size_t countInvalid4  = 0;

//        std::vector<geo::Pt3<std::int16_t>> npos0;
//        std::vector<geo::Pt4<std::uint8_t>> ncol0;

//        std::vector<geo::Pt3<std::int16_t>> npos1;
//        std::vector<geo::Pt4<std::uint8_t>> ncol1;

//        for(size_t jj = 0; jj < cloudSize; ++jj){

//            bool inv1 = uncompressedDepthFrame[jj] == invalid_depth_value;
//            bool inv2 = cloud[jj].x() < -4096 || cloud[jj].x() >= 4096;
//            bool inv3 = cloud[jj].y() < -4096 || cloud[jj].y() >= 4096;
//            bool inv4 = cloud[jj].z() < 0     || cloud[jj].z() >= 8192;

//            if(inv1){
//                ++countInvalid1;
//            }

//            if(inv2){
//                ++countInvalid2;
//            }

//            if(inv3){
//                ++countInvalid3;
//            }

//            if(inv4){
//                ++countInvalid4;
//            }

//            if(inv1 || inv2 || inv3 || inv4){
//                continue;
//            }

//            npos0.push_back(cloud[jj]);
//            ncol0.push_back(color[jj]);

//            packedBuffer[idV] = PackedVoxel::pack64(cloud[jj], color[jj]);
//            geo::Pt3<std::int16_t> pos;
//            geo::Pt4<std::uint8_t> col;
//            PackedVoxel::unpack64(packedBuffer[idV], pos, col);

//            npos1.push_back(pos);
//            ncol1.push_back(col);

//            REQUIRE(cloud[jj].x() == pos.x());
//            REQUIRE(cloud[jj].y() == pos.y());
//            REQUIRE(cloud[jj].z() == pos.z());
//            REQUIRE(color[jj].xyz().conv<int>() == col.xyz().conv<int>());
//            ++idV;
//        }
//        std::cout << "invalid: " << (1.f*countInvalid1/cloudSize) << " "<<
//            (1.f*countInvalid2/cloudSize) << " " <<
//            (1.f*countInvalid3/cloudSize) << " " <<
//            (1.f*countInvalid4/cloudSize) << "\n";




        Bench::start("delta");

        compute_deltas_inplace(xyz.data(),idV*3,0);
        compute_deltas_inplace(reinterpret_cast<std::uint32_t*>(xyz16.data()),(idV*3)/2,0);
//            for (size_t jj = 1; jj < idV*3; ++jj) {
//                xyz[jj] += xyz[jj-1];
//            }
        Bench::stop();


        std::vector<std::uint32_t> cb;
        cb.resize(idV*3 + 1024);
        Bench::start("compress");
        size_t sizeCloudCompressed1 = integerCompressor.encode(
            xyz.data(), idV*3,
            cb.data(), cb.size()
        );

        std::vector<std::uint32_t> cb16;
        cb16.resize((idV*3)/2 + 1024);
        size_t sizeCloudCompressed2 = integerCompressor.encode(
            reinterpret_cast<std::uint32_t*>(xyz16.data()), (idV*3)/2,
            cb16.data(), cb16.size()
        );
        Bench::stop();
//        std::cout << (1.f*sizeCloudCompressed1)/(cb.size()) << "\n";
        std::cout << (1.f*sizeCloudCompressed2)/(cb16.size()) << "\n";



//        Bench::start("sort");
//            std::sort(std::execution::par_unseq, packedBuffer.begin(), packedBuffer.begin() + idV);
//        Bench::stop();

//        Bench::start("delta");
//            for (size_t jj = 1; jj < idV; ++jj) {
//                packedBuffer[jj] += packedBuffer[jj-1];
//            }
//        Bench::stop();
//        for (size_t jj = 0; jj < 3000; ++jj) {
//            std::cout << packedBuffer[jj] << " ";
//        }

//        std::vector<std::uint64_t> cb;
//        cb.resize(idV + 1024);
//        size_t sizeCloudCompressed = integerCompressor.encode(
//            packedBuffer.data(), idV,
//            reinterpret_cast<std::uint32_t*>(cb.data()), cb.size()*2
//        );

//        std::cout << (1.f*sizeCloudCompressed/2)/idV << "\n";


//        REQUIRE(files::CloudIO::save_cloud(std::format("E:/boring_cloud_{}_0.obj",ii), npos0.data(), npos0.size()));
//        REQUIRE(files::CloudIO::save_cloud(std::format("E:/boring_cloud_{}_1.obj",ii), npos1.data(), npos1.size()));

        std::vector<std::uint64_t> compressedPackedBuffer;
        compressedPackedBuffer.resize(cloudSize+1024);
//        Logger::message(std::to_string(cloudSize) + "\n");





//        Bench::start("compress_cloud1");
//        size_t sizeCloudCompressed = integerCompressor.encode(
//            reinterpret_cast<uint32_t*>(cloud), cloudSize,
//            cloudBuffer.data(), cloudBuffer.size()
//        );
//        Bench::stop();

//        std::vector<geo::Pt3f> pts;
//        pts.resize(frame->depthWidth*frame->depthHeight);
//        for(size_t jj = 0; jj < pts.size(); ++jj){
//            auto c = cloud[jj];
//            pts[jj] = {c.x()*0.01f,c.y()*0.01f,c.z()*0.01f};
//        }




//        REQUIRE(files::CloudIO::save_cloud(std::format("E:/boring_cloud_{}_1.obj",ii), pts.data(), pts.size()));
//        REQUIRE(files::CloudIO::save_cloud<std::int16_t>(std::format("E:/boring_cloud_{}_2.obj", ii), cloud, pts.size()));

//        Bench::start("decompress_cloud1");

//        std::vector<geo::Pt3<std::int16_t>> decodedCloud;
//        decodedCloud.resize(frame->depthWidth*frame->depthHeight);
//        size_t sizeCloudUncompressed = integerCompressor.decode(
//            cloudBuffer.data(), sizeCloudCompressed,
//            reinterpret_cast<std::uint32_t*>(decodedCloud.data()), 3*decodedCloud.size()/2
//        );


//        REQUIRE(sizeCloudUncompressed == ((frame->depthWidth*frame->depthHeight*3)/2));

//        Bench::stop();


//        Logger::message(std::format("depth w_{} h_{} us_{} ies_{} ier_{} {} {} {}\n",
//            frame->depthWidth,
//            frame->depthHeight,
//            uncompressedDepthFrame.size(),
//            sizeDepthCompressed,
//            1.f*sizeDepthCompressed/uncompressedDepthFrame.size(),
//            cloudBuffer.size(),
//            sizeCloudCompressed,
//            1.f*sizeCloudCompressed/cloudBuffer.size()
//            ));


//        qoi_desc qoid2;
//        qoid2.channels   = 3;
//        qoid2.width      = frame->depthWidth;
//        qoid2.height     = frame->depthHeight;
//        qoid2.colorspace = QOI_LINEAR;

//        std::vector<geo::Pt3f> cloudToEncode;
//        cloudToEncode.resize(frame->depthWidth*frame->depthHeight);
//        auto max = static_cast<float>(std::numeric_limits<std::int16_t>::max() - std::numeric_limits<std::int16_t>::min());

//        float minX = 10000.f;
//        float maxX = -10000.f;
//        for(size_t jj = 0; jj < cloudToEncode.size(); ++jj){
////            cloudToEncode[jj]
//            auto x = (static_cast<float>(cloud[jj].x()) - std::numeric_limits<std::int16_t>::min())/max;
//            auto y = (static_cast<float>(cloud[jj].y()) - std::numeric_limits<std::int16_t>::min())/max;
//            auto z = (static_cast<float>(cloud[jj].z()) - std::numeric_limits<std::int16_t>::min())/max;
//            if(x < minX){
//                minX = x;
//            }
//            if(x > maxX){
//                maxX = x;
//            }

//            cloudToEncode[jj] = {x,y,z};
//            // 0 - (max-min)
//        }



//        Bench::start("compress_cloud2");
//        encoded = reinterpret_cast<std::uint8_t*>(qoi_encode(cloudToEncode.data(), &qoid2, &size));
//        Bench::stop();
//        REQUIRE(encoded != nullptr);

//        Logger::message(std::format("max {} {} {} size {} {}\n", max, minX, maxX, size, frame->depthWidth*frame->depthHeight));

//        delete encoded;




//        frame->depthWidth;
//        frame->depthHeight;

        //graphics::Texture t2;
        //t2.copy_2d_data(frame->colorWidth, frame->colorHeight, 4, decoded);
        //REQUIRE(t2.write_2d_image_file_data(std::format("E:/boring_img_{}_2.png", ii)));

//        qoi_write(std::format("E:/boring_img_{}.png", ii).c_str(), decoded, &qoid);
    }

    if(tjCompressedImage != nullptr){
        tjFree(tjCompressedImage);
    }
    if(tjCompressedImage2 != nullptr){
        tjFree(tjCompressedImage2);
    }

    tjDestroy(jpegUncompressor);
    tjDestroy(jpegCompressor);

    Bench::display(BenchUnit::milliseconds);



//    void *encoded_qoi = qoi_encode(pixels, &(qoi_desc){
//       .width = w,
//       .height = h,
//       .channels = channels,
//       .colorspace = QOI_SRGB
//   }, &encoded_qoi_size);



    //    using namespace tool::geo;
    //    using namespace tool::camera::K4;

    //    std::mt19937 gen;
    //    std::uniform_int_distribution<> distr1(-4000, 4000);
    //    std::uniform_int_distribution<> distr2(0000, 8000);
    //    std::uniform_int_distribution<> distr3(0, 255);

    //    std::vector<std::tuple<Pt3<std::int16_t>,Pt3<std::uint8_t>>> pts;
    //    size_t count = 1000000;
    //    std::vector<size_t> indices(count);
    //    std::iota(indices.begin(), indices.end(), 0);

    //    pts.reserve(count);
    //    tool::Bench::start("generate");
    //    for(size_t ii = 0; ii < count; ++ii){
    //        pts.emplace_back(
    //            std::make_tuple(
    //                Pt3<std::int16_t>{
    //                    static_cast<std::int16_t>(distr1(gen)),
    //                    static_cast<std::int16_t>(distr1(gen)),
    //                    static_cast<std::int16_t>(distr2(gen)),
    //                },
    //                Pt3<std::uint8_t>{
    //                    static_cast<std::uint8_t>(distr3(gen)),
    //                    static_cast<std::uint8_t>(distr3(gen)),
    //                    static_cast<std::uint8_t>(distr3(gen)),
    //                }
    //            )
    //        );
    //    }
    //    tool::Bench::stop();

    //    std::vector<PackedVoxel> pVoxels(count);
    //    tool::Bench::start("pack");
    //    for_each(std::execution::par_unseq, std::begin(indices), std::end(indices), [&](size_t id){
    //        pVoxels[id].pack(std::get<0>(pts[id]), std::get<1>(pts[id]));
    //    });
    //    tool::Bench::stop();

    //    std::vector<std::tuple<Pt3<std::int16_t>,Pt3<std::uint8_t>>> uPts(count);
    //    tool::Bench::start("unpack");
    //    for_each(std::execution::par_unseq, std::begin(indices), std::end(indices), [&](size_t id){
    //        pVoxels[id].unpack(std::get<0>(uPts[id]), std::get<1>(uPts[id]));
    //    });
    //    tool::Bench::stop();

    //    tool::Bench::start("compare");
    //    size_t countOk1 = 0,countOk2 = 0;
    //    for(size_t ii = 0; ii < uPts.size(); ++ii){
    //        if(std::get<0>(uPts[ii]) == std::get<0>(pts[ii])){
    //            countOk1++;
    //        }
    //        if(std::get<1>(uPts[ii]) == std::get<1>(pts[ii])){
    //            countOk2++;
    //        }
    //    }
    //    tool::Bench::stop();

    //    tool::Bench::display();
    //    std::cout << "count " << countOk1 << " " << countOk2 << "\n";

    //    return 0;

}
