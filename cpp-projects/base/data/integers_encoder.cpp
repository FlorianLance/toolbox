

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

#include "integers_encoder.hpp"

#pragma warning( disable : 4100 )
#pragma warning( disable : 4267 )



// fastfor
#include "fastpfor/codecfactory.h"
#include "fastpfor/deltautil.h"
//#include "fastpfor/simdbinarypacking.h"


using namespace tool::data;

struct IntegersEncoder::Impl{
    std::shared_ptr<FastPForLib::IntegerCODEC> codec = nullptr;
    Impl(){}
};

//void _verify64() {
//    size_t inSize = in64.size();
//    std::vector<uint32_t> encoded(in64.size() * 4);
//    size_t encodeSize = encoded.size();

//    codec->encodeArray(
//        reinterpret_cast<uint64_t *>(in64.data()),
//        inSize,
//        encoded.data(),
//        encodeSize);

//    out64.resize(inSize);
//    codec->decodeArray(
//        encoded.data(),
//        encodeSize,
//        reinterpret_cast<uint64_t *>(out64.data()),
//        inSize);

//    bool passed = true;
//    for (size_t i = 0; i < inSize; ++i) {
//        if (in64[i] != out64[i]) {
//            passed = false;
//        }
//        EXPECT_EQ(in64[i], out64[i]);
//    }
//    if (!passed) {
//        std::cout << "Test failed with int64 input: ";
//        for (size_t i = 0; i < inSize; ++i) {
//            std::cout << in64[i] << " ";
//        }
//        std::cout << std::endl;
//    }
//}


IntegersEncoder::IntegersEncoder() : m_p(std::make_unique<IntegersEncoder::Impl>()){

    // std::shared_ptr<IntegerCODEC>(
    // new CompositeCodec<FastPFor<4>, VariableByte>());

//    m_p->codec = std::shared_ptr<FastPForLib::IntegerCODEC>(
//        new FastPForLib::CompositeCodec<FastPForLib::SIMDFastPFor<2>, FastPForLib::VariableByte>());

//    m_p->codec = std::shared_ptr<FastPForLib::IntegerCODEC>(
//        new FastPForLib::CompositeCodec<FastPForLib::FastPFor<8>, FastPForLib::VariableByte>());

    m_p->codec = std::shared_ptr<FastPForLib::IntegerCODEC>(
        new FastPForLib::CompositeCodec<FastPForLib::FastPFor<8>, FastPForLib::VariableByte>());
}

IntegersEncoder::IntegersEncoder(int nbBits){
    if(nbBits == 2){
        m_p->codec = std::shared_ptr<FastPForLib::IntegerCODEC>(
            new FastPForLib::SIMDBinaryPacking());
    }else if(nbBits == 4){
        m_p->codec = std::shared_ptr<FastPForLib::IntegerCODEC>(
            new FastPForLib::CompositeCodec<FastPForLib::SIMDFastPFor<4>, FastPForLib::VariableByte>());
    }else if(nbBits == 8){
        m_p->codec = std::shared_ptr<FastPForLib::IntegerCODEC>(
            new FastPForLib::CompositeCodec<FastPForLib::FastPFor<8>, FastPForLib::VariableByte>());
    }
}

IntegersEncoder::~IntegersEncoder(){}

size_t IntegersEncoder::encode(uint32_t *inputData, size_t sizeInput, uint32_t *compressedData, size_t outputFullSize){

    try{
        m_p->codec->encodeArray(
            inputData, sizeInput,
            compressedData, outputFullSize
        );
    }catch(std::exception e){
        std::cerr << "Error encode array: " << e.what() << "\n";
        return 0;
    }catch(...){
        std::cerr << "Error encode array: unknow error \n";
        return 0;
    }

    return outputFullSize;
}

size_t IntegersEncoder::encode(uint64_t *inputData, size_t sizeInput, uint32_t *compressedData, size_t outputFullSize){
    try{
        m_p->codec->encodeArray(
            inputData, sizeInput,
            compressedData, outputFullSize
            );
    }catch(std::exception e){
        std::cerr << "Error encode array: " << e.what() << "\n";
        return 0;
    }catch(...){
        std::cerr << "Error encode array: unknow error \n";
        return 0;
    }

    return outputFullSize;
}



size_t IntegersEncoder::decode(uint32_t *codedData, size_t sizeCoded, uint32_t *decodedData, size_t sizeOriginalData){

    try{
        m_p->codec->decodeArray(
            codedData, sizeCoded,
            decodedData, sizeOriginalData
        );
    }catch(std::exception e){
        std::cerr << "Error decode array: " << e.what() << "\n";
        return 0;
    }catch(...){
        std::cerr << "Error decode array: unknow error \n";
        return 0;
    }

    return sizeOriginalData;
}

size_t IntegersEncoder::decode(uint32_t *codedData, size_t sizeCoded, uint64_t *decodedData, size_t sizeOriginalData){

    try{
        m_p->codec->decodeArray(
            codedData, sizeCoded,
            decodedData, sizeOriginalData
            );
    }catch(std::exception e){
        std::cerr << "Error decode array: " << e.what() << "\n";
        return 0;
    }catch(...){
        std::cerr << "Error decode array: unknow error \n";
        return 0;
    }

    return sizeOriginalData;
}


#pragma warning( default : 4100 )
#pragma warning( default : 4267 )


