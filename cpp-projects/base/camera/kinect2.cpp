
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

#include "kinect2.hpp"

// std
#include <bitset>
#include <chrono>
#include <thread>
#include <limits>
#include <algorithm>
#include <execution>
#include <iostream>
#include <fstream>
#include <numeric>

// windows
#include <Ole2.h>

// kinect
#include <Kinect.h>

// opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// turbojpeg
#include <turbojpeg.h>

// local
#include "utility/view.hpp"
#include "utility/utility.hpp"
#include "utility/benchmark.hpp"
#include "utility/logger.hpp"
#include "data/integers_encoder.hpp"


using namespace tool;
using namespace tool::camera;
using namespace tool::camera::K2;
using namespace tool::geo;
using namespace std::chrono_literals;


struct Kinect2::Impl{

    bool cameraInitialized = false;

    bool colorFrameInfosInitialized = false;
    float colorFovDiago  = 0.f;
    float colorFovHori   = 0.f;
    float colorFovVerti  = 0.f;
    int colorWidth = 0;
    int colorHeight = 0;
    unsigned int colorLengthInPixels;
    unsigned int colorBytesPerPixel;

    bool depthFrameInfosInitialized = false;
    float depthFovDiago  = 0.f;
    float depthFovHori   = 0.f;
    float depthFovVerti  = 0.f;
    int depthWidth = 0;
    int depthHeight = 0;
    unsigned int depthLengthInPixels;
    unsigned int depthBytesPerPixel;

    // source
    DWORD frameSource = 0;
    // frames
    IColorFrame *colorframe = nullptr;
    IDepthFrame *depthFrame = nullptr;
    IInfraredFrame *infraFrame = nullptr;
    ILongExposureInfraredFrame *longExposureInfraFrame = nullptr;
    IBodyFrame *bodyFrame = nullptr;
    // frames references
    IMultiSourceFrame *multiSouceFrame = nullptr;
    IColorFrameReference *colorFrameRef = nullptr;
    IDepthFrameReference *depthFrameRef = nullptr;
    IInfraredFrameReference *infraFrameRef = nullptr;
    ILongExposureInfraredFrameReference *longExposureInfraFrameRef = nullptr;
    IBodyFrameReference *bodyFrameRef = nullptr;

    // kinect
    FrameRequest mode = FrameRequest::undefined;
    IKinectSensor* sensor = nullptr;            // Kinect sensor
    IMultiSourceFrameReader* reader = nullptr;  // Kinect data source
    ICoordinateMapper* mapper = nullptr;        // Converts between depth, color, and 3d coordinate
    CameraIntrinsics camIntrinsics;

    // mapping
    uint32_t cameraSpaceTableLength = 0;
    PointF* cameraSpaceTable = nullptr;
    std_v1<ColorSpacePoint> depth2rgb;   // Maps depth pixels to rgb pixels
    std_v1<bool> validityDepth2Rgb;

    // indices
    std_v1<size_t> indicesRawColors;
    std_v1<size_t> indicesDepths;
    std_v1<size_t> indicesDepthsWithoutBorders;
    std_v1<size_t> indicesDepthToColor;

    std_v1<std::pair<size_t, size_t>> indices2dDepths;
//    std_v1<int> indicesDepth2Rgb;

    // depth/cloud
    std_v1<bool> depthMask;
    std::unique_ptr<std_a1<Pt3f, depth_count>> depthDataTemporal = nullptr;    
    cv::Mat binaryDepth;
    cv::Mat erodedBinaryDepth;
    data::IntegersEncoder depthCompressor;
    CloudDataUP cloudData  = nullptr;
    unsigned short* depthBuffer = nullptr;
    unsigned int depthBufferSize = 0;
    int previousSumDepthValues = 0;

    // color
    tjhandle jpegCompressor;
    RgbDataUP colorData          = nullptr;
    RgbDataUP processedColorData = nullptr;
    unsigned char *tjCompressedImage = nullptr;

    // bodiess
    // std::unordered_map<std::uint64_t, std::uint64_t> idBodies;

    // frames
    std::uint32_t currentFrame = 0;
    FrameUP processedFrame = nullptr;

    Impl(){

        // init compressor
        jpegCompressor = tjInitCompress();

        depthDataTemporal = std::make_unique<std_a1<Pt3f, depth_count>>();


        depth2rgb.resize(depth_count);
        validityDepth2Rgb.resize(depth_count);

        indicesDepths.resize(depth_count);
        std::iota(std::begin(indicesDepths), std::end(indicesDepths), 0);

        // ?
        indicesDepthsWithoutBorders.resize((depth_width-2)*(depth_height-2));
        size_t count = 0;
        for(size_t ii = 0; ii < depth_height; ++ii){
            for(size_t jj = 0; jj < depth_width; ++jj){
                if(ii > 0 && ii < depth_height-1 && jj > 0 && jj < depth_width-1 ){
                    indicesDepthsWithoutBorders.emplace_back(count);
                }
                ++count;
            }
        }

        size_t id = 0;
        indices2dDepths.resize(depth_count);
        for(size_t ii = 0; ii < depth_height; ++ii){
            for(size_t jj = 0; jj < depth_width; ++jj){
                indices2dDepths[id++] = std::make_pair(jj,ii); // (0-width / 0-height)
            }
        }
        indicesDepthToColor.resize(depth_count);
//        indicesDepth2Rgb.resize(kinect2_depth_count);

        depthMask.resize(depth_count);

        indicesRawColors.resize(raw_color_data_size/4);
        std::iota(std::begin(indicesRawColors), std::end(indicesRawColors), 0);
        binaryDepth         =   cv::Mat(cv::Size(depth_width, depth_height), CV_8U);
        erodedBinaryDepth   =   cv::Mat(cv::Size(depth_width, depth_height), CV_8U);

        // final data
        colorData = std::make_unique<RgbData>();
        cloudData = std::make_unique<CloudData>();
        processedColorData      = std::make_unique<RgbData>();

        // frame
        processedFrame = std::make_unique<Frame>(true);
    }
};

bool check_func_sucess(HRESULT res){
    return res >= 0;
}

Kinect2::Kinect2() : m_p(std::make_unique<Impl>()){
    Logger::message("Init kinect2");
}

Kinect2::~Kinect2(){

    // camera
    close();


    if(m_p->tjCompressedImage != nullptr){
        tjFree(m_p->tjCompressedImage);
    }

    // compression
    tjDestroy(m_p->jpegCompressor);

    // get map table
    if(m_p->cameraSpaceTable != nullptr){
        delete m_p->cameraSpaceTable;
    }
}

void Kinect2::close(){

    // close sensor
    if(m_p->sensor != nullptr){
        m_p->sensor->Close();
    }

    // clean mapper
    if (m_p->mapper != nullptr){
        m_p->mapper->Release();
        m_p->mapper = nullptr;
    }

    // clean reader
    if (m_p->reader != nullptr){
        m_p->reader->Release();
        m_p->reader = nullptr;
    }

    // clean sensor
    if (m_p->sensor != nullptr){
        m_p->sensor->Release();
        m_p->sensor = nullptr;
    }
    m_p->cameraInitialized = false;
    m_p->mode = FrameRequest::undefined;
}



bool Kinect2::acquire_multi_sources_frame(){

    m_p->multiSouceFrame = nullptr;

    using namespace std::chrono;

    auto timeStart = high_resolution_clock::now();

    bool get = true;
    while(get){
        auto ret = m_p->reader->AcquireLatestFrame(&m_p->multiSouceFrame);
        if(ret == E_PENDING){
            auto currenTime = duration_cast<milliseconds>(high_resolution_clock::now() - timeStart).count();
            if(currenTime > 3000){
                Logger::error("Failed: AcquireLatestFrame still pending");
                return false;
            }
            continue;
        }else if(!check_func_sucess(ret)){
            Logger::error("Failed: AcquireLatestFrame");
            return false;
        }
        break;
    }


    return true;
}

bool Kinect2::acquire_color_frame(){ 

    if(!check_func_sucess(m_p->multiSouceFrame->get_ColorFrameReference(&m_p->colorFrameRef))){
        Logger::warning("Fail get_ColorFrameReference");
        return false;
    }


    auto ret = m_p->colorFrameRef->AcquireFrame(&m_p->colorframe);
    if(!check_func_sucess(ret)){
        return false;
    }


    if(!m_p->colorFrameInfosInitialized){

        IFrameDescription *f= nullptr;
        if(check_func_sucess(m_p->colorframe->get_FrameDescription(&f))){
            f->get_DiagonalFieldOfView(&m_p->colorFovDiago);
            f->get_HorizontalFieldOfView(&m_p->colorFovHori);
            f->get_VerticalFieldOfView(&m_p->colorFovVerti);
            f->get_Width(&m_p->colorWidth);
            f->get_Height(&m_p->colorHeight);
            f->get_LengthInPixels(&m_p->colorLengthInPixels);
            f->get_BytesPerPixel(&m_p->colorBytesPerPixel);

        }
        m_p->colorFrameInfosInitialized = true;
        // Logger::message(QSL("Color FOV: ") % QString::number(m_p->colorFovDiago) % QSL(" ") % QString::number(m_p->colorFovHori) % QSL(" ") % QString::number(m_p->colorFovVerti));
        // Logger::message(QSL("Color Lengths: ") % QString::number(m_p->colorWidth) % QSL(" ") % QString::number(m_p->colorHeight) % QSL(" ") % QString::number(m_p->colorLengthInPixels)% QSL(" ") % QString::number(m_p->colorBytesPerPixel));
    }

    return true;
}

bool Kinect2::acquire_depth_frame(){

    if(!check_func_sucess(m_p->multiSouceFrame->get_DepthFrameReference(&m_p->depthFrameRef))){
        Logger::warning("Fail get_DepthFrameReference");
        return false;
    }

    auto ret = m_p->depthFrameRef->AcquireFrame(&m_p->depthFrame);
    if(!check_func_sucess(ret)){
        Logger::warning("Fail to acquire depth frame");
        return false;
    }

    if(!m_p->depthFrameInfosInitialized){

        IFrameDescription *f= nullptr;
        if(check_func_sucess(m_p->depthFrame->get_FrameDescription(&f))){
            f->get_DiagonalFieldOfView(&m_p->depthFovDiago);
            f->get_HorizontalFieldOfView(&m_p->depthFovHori);
            f->get_VerticalFieldOfView(&m_p->depthFovVerti);
            f->get_Width(&m_p->depthWidth);
            f->get_Height(&m_p->depthHeight);
            f->get_LengthInPixels(&m_p->depthLengthInPixels);
            f->get_BytesPerPixel(&m_p->depthBytesPerPixel);
        }

        m_p->depthFrameInfosInitialized = true;
        // Logger::message(QSL("Depth FOV: ") % QString::number(m_p->depthFovDiago) % QSL(" ") % QString::number(m_p->depthFovHori) % QSL(" ") % QString::number(m_p->depthFovVerti));
        // Logger::message(QSL("Depth Lengths: ") % QString::number(m_p->depthWidth) % QSL(" ") % QString::number(m_p->depthHeight) % QSL(" ") % QString::number(m_p->depthLengthInPixels)% QSL(" ") % QString::number(m_p->colorBytesPerPixel));
    }


    return true;
}

bool Kinect2::acquire_infra_frame(){

    if(!check_func_sucess(m_p->multiSouceFrame->get_InfraredFrameReference(&m_p->infraFrameRef))){
         Logger::warning("Fail get_InfraredFrameReference");
        return false;
    }
    if(!check_func_sucess(m_p->infraFrameRef->AcquireFrame(&m_p->infraFrame))){
        return false;
    }

    return true;
}

bool Kinect2::acquire_long_exposure_infra_frame(){

    if(!check_func_sucess(m_p->multiSouceFrame->get_LongExposureInfraredFrameReference(&m_p->longExposureInfraFrameRef))){
         Logger::warning("Fail get_LongExposureInfraredFrameReference");
        return false;
    }
    if(!check_func_sucess(m_p->longExposureInfraFrameRef->AcquireFrame(&m_p->longExposureInfraFrame))){
        return false;
    }
    return true;
}

bool Kinect2::acquire_body_frame(){

    if(!check_func_sucess(m_p->multiSouceFrame->get_BodyFrameReference(&m_p->bodyFrameRef))){
         Logger::warning("Fail get_BodyFrameReference");
        return false;
    }
    if(!check_func_sucess(m_p->bodyFrameRef->AcquireFrame(&m_p->bodyFrame))){
        return false;
    }

    return true;
}


bool Kinect2::open(FrameRequest m){

    // initialize sensor
    if(!m_p->cameraInitialized){
        if(!check_func_sucess(GetDefaultKinectSensor(&m_p->sensor))){
            Logger::error("Failed get default kinect sensor");
            return false;
        }

        if (m_p->sensor == nullptr) {
            Logger::error("Failed opening sensor");
            return false;
        }

        if(!check_func_sucess(m_p->sensor->Open())){
            Logger::error("Cannot open kinect");
            return false;
        }

        if(!check_func_sucess(m_p->sensor->get_CoordinateMapper(&m_p->mapper))){
            Logger::error("Fail get coordinate mapper");
            return false;
        }
        m_p->cameraInitialized = true;
    }

    if(m_p->mode == m){
        return true; // nothing to do
    }else{
        // update current mode
        m_p->mode = m;
    }

    // init frame source
    m_p->frameSource = 0;
    if(color_channel_required(m)){
        m_p->frameSource |= FrameSourceTypes::FrameSourceTypes_Color;
    }
    if(depth_channel_required(m)){
        m_p->frameSource |= FrameSourceTypes::FrameSourceTypes_Depth;
    }
    if(infra_channel_required(m)){
        m_p->frameSource |= FrameSourceTypes::FrameSourceTypes_Infrared;
    }
    if(long_infra_channel_required(m)){
        m_p->frameSource |= FrameSourceTypes::FrameSourceTypes_LongExposureInfrared;
    }
    m_p->frameSource |= FrameSourceTypes::FrameSourceTypes_Body;

    // reset data
    std::fill(std::begin(*m_p->colorData), std::end(*m_p->colorData), Pt3<std::uint8_t>{0,0,0});
    std::fill(std::begin(*m_p->cloudData), std::end(*m_p->cloudData), Pt3f{});
    std::fill(std::begin(m_p->depthMask), std::end(m_p->depthMask), false);

    // open frame source
    if(!check_func_sucess(m_p->sensor->OpenMultiSourceFrameReader(m_p->frameSource, &m_p->reader))){
        Logger::error("Failed: OpenMultiSourceFrameReader");
        return false;
    }

    return true;
}


std::optional<Frame> Kinect2::get_kinect_data() {

    if(m_p->reader == nullptr){
        Logger::error("Kinect not opened");
        return {};
    }

    Bench::start("Kinect2::get_kinect_data"sv);
    Bench::start("Kinect2::acquire_multi_sources_frame"sv);

    // update frame mode and id
    m_p->processedFrame->mode = m_p->mode;
    m_p->processedFrame->frameId = m_p->currentFrame;

    // get data
    if(!acquire_multi_sources_frame()){
        clean_frame();
        return {};
    }

    m_p->processedFrame->timeStampGetFrame = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    Bench::stop();

    // store intrinsics
    if(m_p->processedFrame->intrinsics.size() == 0){
        m_p->processedFrame->intrinsics = intrinsics();
    }

    Bench::start("Kinect2::get_depth_data"sv);
    // # depth
    if(depth_channel_required(m_p->mode)){
        if(acquire_depth_frame()){
            if(!get_depth_data()){
                clean_frame();
                return {};
            }
        }else{
            clean_frame();
            return {};
        }
    }  
    Bench::stop();
    Bench::start("Kinect2::get_color_data"sv);
    // # color
    if(color_channel_required(m_p->mode)){
        if(acquire_color_frame()){
            get_color_data();
        }else{
            clean_frame();
            return {};
        }
    }
    Bench::stop();
    Bench::start("Kinect2::get_infra_data"sv);
    // # infra
    if(infra_channel_required(m_p->mode)){
        if(acquire_infra_frame()){
            get_infra_data();
        }else{
            clean_frame();
            return {};
        }
    }
    Bench::stop();
    Bench::start("Kinect2::get_long_exposure_infra_data"sv);
    // # long exposure infra
    if(long_infra_channel_required(m_p->mode)){
        if(acquire_long_exposure_infra_frame()){
            get_long_exposure_infra_data();
        }else{
            clean_frame();
            return {};
        }
    }
    Bench::stop();
    Bench::start("Kinect2::get_body_data"sv);
    // # body
    if(body_channel_required(m_p->mode)){
        if(acquire_body_frame()){
            get_body_data();
        }        
    }
    Bench::stop();
    Bench::start("Kinect2::post_computing_depth_data"sv);

    // post computing data
    if(depth_channel_required(m_p->mode)){
        post_computing_depth_data();
    }

    Bench::stop();
    Bench::start("Kinect2::post_computing_color_data"sv);

    if(color_channel_required(m_p->mode)){
        post_computing_color_data();
    }

    Bench::stop();
    Bench::start("Kinect2::post_computing_infra_data"sv);

    if(infra_channel_required(m_p->mode) || long_infra_channel_required(m_p->mode)){
        post_computing_infra_data();
    }

    Bench::stop();
    Bench::start("Kinect2::post_computing_body_data"sv);

    if(body_channel_required(m_p->mode)){
        post_computing_body_data();
    }   

    Bench::stop();
    Bench::start("Kinect2::process"sv);

    // processing
    if(m_p->mode == FrameRequest::compressed_color_1920x1080){
        process_compressed_color_1920x1080();
    }else if(m_p->mode == FrameRequest::compressed_color_512x424){
        process_compressed_color_512x424();
    }else if(m_p->mode == FrameRequest::compressed_color_cloud ){
        process_compressed_color_cloud();
    }else if(m_p->mode == FrameRequest::compressed_color_mesh){
        process_compressed_color_mesh();
    }else if(m_p->mode == FrameRequest::depth_512x424){
        process_depth_512x424();
    }else if(m_p->mode == FrameRequest::infra_512x424 || m_p->mode == FrameRequest::long_exposure_infra_512x424){
        process_infra_512x424();
    }        
    Bench::stop();
    Bench::start("Kinect2::release"sv);

    // increase frame id
    ++m_p->currentFrame;

    // get end process timestamp
    m_p->processedFrame->timeStampEndProcessing = std::chrono::high_resolution_clock::now().time_since_epoch().count();

    // clean
    clean_frame();

    Bench::stop();

    Bench::start("Kinect2::copy_frame"sv);
    auto copiedFrame = m_p->processedFrame->copy_frame();
    Bench::stop();
    Bench::stop();

    return std::move(copiedFrame);
}

FrameRequest Kinect2::mode() const{
    return m_p->mode;
}

bool Kinect2::save_camera_intrisics(std::string filePath){

    if(filePath.size() == 0){
        return false;
    }

    intrinsics();

    std::ofstream file;
    std::string name(filePath);
    file.open(name);
    if(file.is_open()){
        file << "Focal length:\n" << m_p->camIntrinsics.FocalLengthX  << " " << m_p->camIntrinsics.FocalLengthY << "\n";
        file << "Principal point:\n" << m_p->camIntrinsics.PrincipalPointX  << " " << m_p->camIntrinsics.PrincipalPointY << "\n";
        file << "Radial distortion:\n" << m_p->camIntrinsics.RadialDistortionSecondOrder << " " << m_p->camIntrinsics.RadialDistortionFourthOrder << " " << m_p->camIntrinsics.RadialDistortionSixthOrder  << "\n";
    }else{
        return false;
    }
    file.close();

    return true;
}

bool Kinect2::save_camera_space_depth_table(std::string filePath){

    // get map table
    if(m_p->cameraSpaceTable == nullptr){
       delete m_p->cameraSpaceTable;
        m_p->cameraSpaceTable = nullptr;
    }
    m_p->mapper->GetDepthFrameToCameraSpaceTable(&m_p->cameraSpaceTableLength, &m_p->cameraSpaceTable);

    if(filePath.size() == 0){
        return false;
    }

    std::ofstream file;
    std::string name(filePath);
    file.open(name);
    if(file.is_open()){
        for(size_t ii = 0; ii < m_p->cameraSpaceTableLength; ++ii){
            file << m_p->cameraSpaceTable[ii].X << " " <<  m_p->cameraSpaceTable[ii].Y << "\n";
        }
    }else{
        return false;
    }
    file.close();

    return true;
}

//void Kinect2::copy_frame(Kinect2Frame *f){

////    m_p->swapLocker.lock();
//    m_p->availableFrame->parameters = parameters;
//    if(!m_p->availableFrame->copy_to(f)){
//        Logger::error("Available frame color size is 0");
//    }
////    m_p->swapLocker.unlock();
//    std::cout << "[COP2_" <<  std::this_thread::get_id() << "]";
//}


std_v1<float> Kinect2::intrinsics() const{

    m_p->mapper->GetDepthCameraIntrinsics(&m_p->camIntrinsics);
    return {
        m_p->camIntrinsics.FocalLengthX,
        m_p->camIntrinsics.FocalLengthY,
        m_p->camIntrinsics.PrincipalPointX,
        m_p->camIntrinsics.PrincipalPointY,
        m_p->camIntrinsics.RadialDistortionSecondOrder,
        m_p->camIntrinsics.RadialDistortionFourthOrder,
        m_p->camIntrinsics.RadialDistortionSixthOrder
    };
}

void Kinect2::get_color_data(){

    Bench::start("Kinect2::get_color_data_0"sv);
    using namespace std;

    UINT bufferSize = 0;
    std::uint8_t* pBuffer = nullptr;
    m_p->colorframe->AccessRawUnderlyingBuffer( &bufferSize, &pBuffer ); // YUY2
    m_p->colorframe->get_RelativeTime(&m_p->processedFrame->relativeTimeColor);
    Pt4<uint8_t> *colors = reinterpret_cast<Pt4<uint8_t>*>(pBuffer);

    Bench::stop();


    if(depth_channel_required(m_p->mode)){

        Bench::start("Kinect2::get_color_data_1"sv);

        for_each(execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){

            if(!is_kinect2_depth_valid((*m_p->cloudData)[id].z()) || !m_p->validityDepth2Rgb[id]){
                (*m_p->colorData)[id] = {0,0,0};
                return;
            }

            size_t idC = m_p->indicesDepthToColor[id];
            if(idC%2 == 0){

                Pt3<uint8_t> &c1 = (*m_p->colorData)[idC];
                idC = idC/2;

                const Pt4<uint8_t> &color = colors[idC];

                // convert to rgb
                const int y0 = clamp(static_cast<int>(parameters.yFactor*color.x()), 0, 255);
                auto ci = 298 * (y0 - 16);

                const int u0 = clamp(static_cast<int>(parameters.uFactor*color.y()), 0, 255);
                const auto d = u0 - 128;

                const int v0 = clamp(static_cast<int>(parameters.vFactor*color.w()), 0, 255);
                const auto e = v0 - 128;

                c1.x() = static_cast<uint8_t>(clamp((ci + (409 * e) + 128)              >> 8, 0, 255)); // red
                c1.y() = static_cast<uint8_t>(clamp((ci + (-100 * d) - (208 * e) + 128) >> 8, 0, 255)); // green
                c1.z() = static_cast<uint8_t>(clamp((ci + (516 * d) + 128)              >> 8, 0, 255)); // blue

            }else{
                Pt3<uint8_t> &c2 = (*m_p->colorData)[idC];
                idC = (idC-1)/2;

                const Pt4<uint8_t> &color = colors[idC];
                const int y0 = clamp(static_cast<int>(parameters.yFactor*color.x()), 0, 255);
                const int u0 = clamp(static_cast<int>(parameters.uFactor*color.y()), 0, 255);
                const int y1 = clamp(static_cast<int>(parameters.yFactor*color.z()), 0, 255);
                const int v0 = clamp(static_cast<int>(parameters.vFactor*color.w()), 0, 255);

                // convert to rgb
                auto c = y0 - 16;
                const auto d = u0 - 128;
                const auto e = v0 - 128;

                auto ci = 298 * c;
                const auto v1 = (516 * d) + 128;
                const auto v2 = (-100 * d) - (208 * e) + 128;
                const auto v3 = (409 * e) + 128;

                c = y1 - 16;
                ci = 298 * c;

                c2.x() = static_cast<uint8_t>(clamp((ci + v3) >> 8, 0, 255)); // red
                c2.y() = static_cast<uint8_t>(clamp((ci + v2) >> 8, 0, 255)); // green
                c2.z() = static_cast<uint8_t>(clamp((ci + v1) >> 8, 0, 255)); // blue
            }
        });

        Bench::stop();
    }else{

        Bench::start("Kinect2::get_color_data_2"sv);

        for_each(execution::par_unseq, begin(m_p->indicesRawColors), end(m_p->indicesRawColors), [&](size_t &id){
            // get yuv
            const Pt4<uint8_t> &color = colors[id];
            const int y0 = clamp(static_cast<int>(parameters.yFactor*color.x()), 0, 255);
            const int u0 = clamp(static_cast<int>(parameters.uFactor*color.y()), 0, 255);
            const int y1 = clamp(static_cast<int>(parameters.yFactor*color.z()), 0, 255);
            const int v0 = clamp(static_cast<int>(parameters.vFactor*color.w()), 0, 255);

            // convert to rgb
            auto c = y0 - 16;
            const auto d = u0 - 128;
            const auto e = v0 - 128;

            const auto idC = id*2;
            Pt3<uint8_t> &c1 = (*m_p->colorData)[idC];
            auto ci = 298 * c;
            const auto v1 = (516 * d) + 128;
            const auto v2 = (-100 * d) - (208 * e) + 128;
            const auto v3 = (409 * e) + 128;
            c1.x() = static_cast<uint8_t>(clamp((ci + v3) >> 8, 0, 255)); // red
            c1.y() = static_cast<uint8_t>(clamp((ci + v2) >> 8, 0, 255)); // green
            c1.z() = static_cast<uint8_t>(clamp((ci + v1) >> 8, 0, 255)); // blue

            c = y1 - 16;
            ci = 298 * c;

            Pt3<uint8_t> &c2 =  (*m_p->colorData)[idC+1];
            c2.x() = static_cast<uint8_t>(clamp((ci + v3) >> 8, 0, 255)); // red
            c2.y() = static_cast<uint8_t>(clamp((ci + v2) >> 8, 0, 255)); // green
            c2.z() = static_cast<uint8_t>(clamp((ci + v1) >> 8, 0, 255)); // blue
        });

        Bench::stop();
    }
}


bool Kinect2::get_depth_data() {

    Bench::start("Kinect2::get_depth_data_0"sv);

    // get data
    m_p->depthBuffer = nullptr;
    m_p->depthFrame->AccessUnderlyingBuffer(&m_p->depthBufferSize, &m_p->depthBuffer);
    Bench::stop();

    int sumDepthValues = std::accumulate(m_p->depthBuffer, m_p->depthBuffer + m_p->depthBufferSize, 0);
    if(m_p->previousSumDepthValues > 0){
        if(sumDepthValues == m_p->previousSumDepthValues){
            // identical frame
            Logger::error("Identical depth frame");
            return false;
        }
    }
    m_p->previousSumDepthValues = sumDepthValues;

    Bench::start("Kinect2::get_depth_data_1"sv);

    m_p->depthFrame->get_RelativeTime(&m_p->processedFrame->relativeTimeDepth);
    m_p->depthFrame->get_DepthMinReliableDistance(&m_p->processedFrame->minReliableDistance);
    m_p->depthFrame->get_DepthMaxReliableDistance(&m_p->processedFrame->maxReliableDistance);

    Bench::stop();
    Bench::start("Kinect2::get_depth_data_2"sv);

    // map data
    m_p->mapper->MapDepthFrameToCameraSpace(
                depth_count, m_p->depthBuffer, // Depth frame data and size of depth frame
                depth_count, reinterpret_cast<CameraSpacePoint*>(m_p->cloudData->data())); // Output CameraSpacePoint array and size

    Bench::stop();
    Bench::start("Kinect2::get_depth_data_3"sv);

    m_p->mapper->MapDepthFrameToColorSpace(
            depth_count, m_p->depthBuffer, // Depth frame data and size of depth frame
            depth_count, m_p->depth2rgb.data()); // Output ColorSpacePoint array and size

    for_each(std::execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){
        const auto &m = m_p->depth2rgb[id];
        m_p->validityDepth2Rgb[id]   = !(m.X < 0 || m.Y < 0 || m.X > color_width || m.Y > color_height);
        m_p->indicesDepthToColor[id] = {static_cast<size_t>(m.X) + color_width*(static_cast<size_t>(m.Y))};

    });


    Bench::stop();
    return true;
}

void Kinect2::get_infra_data(){

    // get data
    unsigned int size;
    unsigned short* buffer = nullptr;
    m_p->infraFrame->AccessUnderlyingBuffer(&size, &buffer);
    m_p->infraFrame->get_RelativeTime(&m_p->processedFrame->relativeTimeInfra);
    std::copy(buffer, buffer + size, std::begin(*m_p->processedFrame->infraData));
}

void Kinect2::get_long_exposure_infra_data(){

    // get data
    unsigned int size;
    unsigned short* buffer = nullptr;
    m_p->longExposureInfraFrame->AccessUnderlyingBuffer(&size, &buffer);
    m_p->longExposureInfraFrame->get_RelativeTime(&m_p->processedFrame->relativeTimeLongInfra);
    std::copy(buffer, buffer + size, std::begin(*m_p->processedFrame->infraData));
}

void Kinect2::get_body_data(){

    Bench::start("Kinect2::get_body_data_0"sv);

    std_a1<IBody*,BODY_COUNT> bodies = {nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};
    if(!check_func_sucess(m_p->bodyFrame->GetAndRefreshBodyData(static_cast<unsigned int>(bodies.size()), bodies.data()))){
        return;
    }
    m_p->bodyFrame->get_RelativeTime(&m_p->processedFrame->relativeTimeBody);

    Bench::stop();
    Bench::start("Kinect2::get_body_data_1"sv);

    for (size_t ii = 0; ii < bodies.size(); ii++) {

        auto body = bodies[ii];
        auto &data = m_p->processedFrame->bodiesData[ii];
        std::uint64_t trackingId = 0;
        if(S_OK == body->get_TrackingId(&trackingId)){ // Gets the tracking ID.
            data.id = trackingId;
        }

        BOOLEAN isTracked = false;
        if(S_OK == body->get_IsTracked(&isTracked)){ // Retrieves a boolean value that indicates if the body is tracked.
            data.tracked = isTracked;
        }
        BOOLEAN isRestricted = false;
        if(S_OK == body->get_IsRestricted(&isRestricted)){ // Retrieves a boolean value that indicates if the body is restricted from a full range of motion.
            data.restricted = isRestricted;
        }
        DetectionResult engaged;
        if(S_OK == body->get_Engaged(&engaged)){ // Gets the level of user engagement.
            data.engaged = static_cast<Detection>(engaged);
        }
        HandState leftHandState;
        if(S_OK == body->get_HandLeftState(&leftHandState)){ // Gets the left hand state.
            data.leftHandState = static_cast<HandStateT>(leftHandState);
        }
        TrackingConfidence leftHandConfidence;
        if(S_OK == body->get_HandLeftConfidence(&leftHandConfidence)){ // Gets the tracking confidence for the left hand.
            data.leftHandHightConfidence = leftHandConfidence == TrackingConfidence::TrackingConfidence_High;
        }
        HandState rightHandState;
        if(S_OK == body->get_HandRightState(&rightHandState)){ // Gets the right hand state.
            data.rightHandState = static_cast<HandStateT>(rightHandState);
        }
        TrackingConfidence rightHandConfidence;
        if(S_OK == body->get_HandRightConfidence(&rightHandConfidence)){ // Gets the tracking confidence for the right hand.
            data.rightHandHightConfidence = rightHandConfidence == TrackingConfidence::TrackingConfidence_High;
        }
        PointF lean;
        if(S_OK == body->get_Lean(&lean)){ // Gets the amount a body is leaning, which is a number between -1 (leaning left or back) and 1 (leaning right or front)
            data.lean = {lean.X,lean.Y};
        }
        TrackingState leanTracking;
        if(S_OK == body->get_LeanTrackingState(&leanTracking)){ // Gets the lean tracking state, which indicates if the body is tracked.
            data.leanTracking = static_cast<TrackingStateT>(leanTracking);
        }

        DWORD clippedEdges;
        if(S_OK == body->get_ClippedEdges(&clippedEdges)){ // Gets the clipped edges.
            // ...
        }

        std_a1<Joint,JointType_Count> joints;
        if(S_OK == body->GetJoints(JointType_Count, joints.data())){
            for(auto &joint : joints){
                auto type = static_cast<BodyJointType>(joint.JointType);
                if(data.joints.count(type)==0){
                    data.joints[type] = BodyJoint{};
                }
                data.joints[type].state = static_cast<TrackingStateT>(joint.TrackingState);
                const auto &p = joint.Position;
                data.joints[type].pos = {p.X,p.Y,p.Z};
            }
        }

        std_a1<JointOrientation,JointType_Count> jointsOrientations;
        if(S_OK == body->GetJointOrientations(JointType_Count, jointsOrientations.data())){

            for(auto &jointOrientation : jointsOrientations){

                auto type = static_cast<BodyJointType>(jointOrientation.JointType);
                if(data.joints.count(type)==0){
                    data.joints[type] = BodyJoint{};
                }
                const auto &q =jointOrientation.Orientation;
                data.joints[type].rotQuaternion = {q.x,q.y,q.z,q.w};
            }
        }

        // body->GetActivityDetectionResults() // Gets the activity detection results from IBody.
        // body->GetAppearanceDetectionResults() // Gets the appearance.
        // body->GetExpressionDetectionResults(); // always return null, (cross platform with xbox)
        body->Release();
    }

    Bench::stop();

//    auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);
//    auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(t3-t2);
//    std::cout << "get_body: " << d1.count() << " " << d2.count() <<"\n";

//    m_p->bodyFrame->get_FloorClipPlane();
//    m_p->bodyFrame->get_BodyFrameSource();
}

void Kinect2::post_computing_color_data(){

    using namespace std;

    if(parameters.filterDepthWithColor){

        auto &depth = *m_p->cloudData.get();
        const auto &p = parameters;

        for_each(execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){

            if(is_kinect2_depth_valid(depth[id].z()) && m_p->validityDepth2Rgb[id]){

                auto color = (*m_p->colorData)[m_p->indicesDepthToColor[id]];

                // test euclidian distance

                auto delta = sqrt(
                    (color[0]-p.filterColor[0])*(color[0]-p.filterColor[0]) +
                    (color[1]-p.filterColor[1])*(color[1]-p.filterColor[1]) +
                    (color[2]-p.filterColor[2])*(color[2]-p.filterColor[2])
                );
                if(delta > p.maxDiffColor.x()){
                    depth[id].z() = invalid_value;
                    return;
                }


//                if(abs(static_cast<int>(color.x())-static_cast<int>(p.filterColor.x())) > p.maxDiffColor.x()){
//                    depth[id].z() = kinect2_invalid_value;
//                    return;
//                }
//                if(abs(static_cast<int>(color.y())-static_cast<int>(p.filterColor.y())) > p.maxDiffColor.y()){
//                    depth[id].z() = kinect2_invalid_value;
//                    return;
//                }
//                if(abs(static_cast<int>(color.z())-static_cast<int>(p.filterColor.z())) > p.maxDiffColor.z()){
//                    depth[id].z() = kinect2_invalid_value;
//                    return;
//                }
            }
        });
    }

}


void Kinect2::post_computing_depth_data(){

    using namespace std;

    Bench::start("Kinect2::post_computing_depth_data_0"sv);



    auto binaryDepthData       = m_p->binaryDepth.ptr();
    auto erodedBinaryDepthData = m_p->erodedBinaryDepth.ptr();

    for_each(execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){

        auto &depth = (*m_p->cloudData)[id];

        // reset binary mat
        binaryDepthData[id] = 0;

        // remove inf pts
        if(!isnormal(depth.x()) || !isnormal(depth.y()) || !isnormal(depth.z())){
            depth = invalid_point;
            return;
        }

        // width/height filter
        const auto &id2d = m_p->indices2dDepths[id];
        if(id2d.first < parameters.minWidth  || id2d.first > parameters.maxWidth ||  id2d.second < parameters.minHeight || id2d.second > parameters.maxHeight){
            depth = invalid_point;
            return;
        }

        // depth filter
        if(depth.z() < parameters.minDepthValue || depth.z() > parameters.maxDepthValue){
            depth = invalid_point;
            return;
        }

        // binary mat
        binaryDepthData[id] = 255;
    });

    Bench::stop();
    Bench::start("Kinect2::post_computing_depth_data_1"sv);

    // erosion
    if(parameters.doErosion){

        cv::Mat elementErode = cv::getStructuringElement(
            parameters.erosionType,
            cv::Size( 2*parameters.erosionSize + 1, 2*parameters.erosionSize+1),
            cv::Point( parameters.erosionSize, parameters.erosionSize)
        );
        cv::erode( m_p->binaryDepth, m_p->erodedBinaryDepth, elementErode);

        for_each(execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){

            if(erodedBinaryDepthData[id] < parameters.minErosionValue){
                (*m_p->cloudData)[id].z() = invalid_value;
            }
        });
    }

    Bench::stop();
    Bench::start("Kinect2::post_computing_depth_data_2"sv);

    // local depth
    constexpr size_t widthPlusOne = depth_width +1;
    constexpr size_t widthMinusOne = depth_width -1;

    // neighbours number
    const bool doMinNeighboursFiltering = parameters.minNeighboursLoops > 0;
    if(doMinNeighboursFiltering){

        for(size_t numLoop = 0; numLoop < parameters.minNeighboursLoops; ++numLoop){

            for_each(execution::par_unseq, begin(m_p->indicesDepthsWithoutBorders), end(m_p->indicesDepthsWithoutBorders), [&](size_t &id){

                auto depth = (*m_p->cloudData)[id];
                const auto depthZ = depth.z();
                if(!is_kinect2_depth_valid(depthZ)){
                    return;
                }

                size_t count = 0;
                const size_t idA = id - widthPlusOne;
                const size_t idB = idA + 1;
                const size_t idC = idB + 1;
                const size_t idD = id - 1;
                const size_t idE = id + 1;
                const size_t idF = id + widthMinusOne;
                const size_t idG = idF + 1;
                const size_t idH = idG + 1;

                if(is_kinect2_depth_valid((*m_p->cloudData)[idA].z())){
                    ++count;
                }
                if(is_kinect2_depth_valid((*m_p->cloudData)[idB].z())){
                    ++count;
                }
                if(is_kinect2_depth_valid((*m_p->cloudData)[idC].z())){
                    ++count;
                }
                if(is_kinect2_depth_valid((*m_p->cloudData)[idD].z())){
                    ++count;
                }
                if(is_kinect2_depth_valid((*m_p->cloudData)[idE].z())){
                    ++count;
                }
                if(is_kinect2_depth_valid((*m_p->cloudData)[idF].z())){
                    ++count;
                }
                if(is_kinect2_depth_valid((*m_p->cloudData)[idG].z())){
                    ++count;
                }
                if(is_kinect2_depth_valid((*m_p->cloudData)[idH].z())){
                    ++count;
                }

                m_p->depthMask[id] = count < parameters.nbMinNeighboursNb;
            });

            for_each(execution::par_unseq, begin(m_p->indicesDepthsWithoutBorders), end(m_p->indicesDepthsWithoutBorders), [&](size_t &id){
                if(m_p->depthMask[id]){
                    (*m_p->cloudData)[id].z() = invalid_value;
                }
                m_p->depthMask[id] = false;
            });
        }
    }


    Bench::stop();
    Bench::start("Kinect2::post_computing_depth_data_3"sv);


    const bool doLocalDiffFiltering = parameters.maxLocalDiff > 0.f;
    if(doLocalDiffFiltering){

        const float mLocal =  parameters.maxLocalDiff/1000.f;
        for_each(execution::par_unseq, begin(m_p->indicesDepthsWithoutBorders), end(m_p->indicesDepthsWithoutBorders), [&](size_t &id){

            m_p->depthMask[id] = false;

            const auto depthZ = (*m_p->cloudData)[id].z();
            if(!is_kinect2_depth_valid(depthZ)){
                return;
            }

            float meanDiff = 0.f;
            size_t count = 0;
            const size_t idA = id - widthPlusOne;
            const size_t idB = idA + 1;
            const size_t idC = idB + 1;
            const size_t idD = id - 1;
            const size_t idE = id + 1;
            const size_t idF = id + widthMinusOne;
            const size_t idG = idF + 1;
            const size_t idH = idG + 1;

            if(is_kinect2_depth_valid((*m_p->cloudData)[idA].z())){
                meanDiff += abs((*m_p->cloudData)[idA].z()-depthZ);
                ++count;
            }
            if(is_kinect2_depth_valid((*m_p->cloudData)[idB].z())){
                meanDiff += abs((*m_p->cloudData)[idB].z()-depthZ);
                ++count;
            }
            if(is_kinect2_depth_valid((*m_p->cloudData)[idC].z())){
                meanDiff += abs((*m_p->cloudData)[idC].z()-depthZ);
                ++count;
            }
            if(is_kinect2_depth_valid((*m_p->cloudData)[idD].z())){
                meanDiff += abs((*m_p->cloudData)[idD].z()-depthZ);
                ++count;
            }
            if(is_kinect2_depth_valid((*m_p->cloudData)[idE].z())){
                meanDiff += abs((*m_p->cloudData)[idE].z()-depthZ);
                ++count;
            }
            if(is_kinect2_depth_valid((*m_p->cloudData)[idF].z())){
                meanDiff += abs((*m_p->cloudData)[idF].z()-depthZ);
                ++count;
            }
            if(is_kinect2_depth_valid((*m_p->cloudData)[idG].z())){
                meanDiff += abs((*m_p->cloudData)[idG].z()-depthZ);
                ++count;
            }
            if(is_kinect2_depth_valid((*m_p->cloudData)[idH].z())){
                meanDiff += abs((*m_p->cloudData)[idH].z()-depthZ);
                ++count;
            }

            m_p->depthMask[id] = (count == 0) ? false : (meanDiff/count > mLocal);
        });

        for_each(execution::par_unseq, begin(m_p->indicesDepthsWithoutBorders), end(m_p->indicesDepthsWithoutBorders), [&](size_t &id){
            if(m_p->depthMask[id]){
                (*m_p->cloudData)[id].z() = invalid_value;
            }
            m_p->depthMask[id] = false;
        });
    }

    Bench::stop();
    Bench::start("Kinect2::post_computing_depth_data_4"sv);

    if(parameters.doTemporalFilter){
        for_each(std::execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){
            if(is_kinect2_depth_valid(m_p->cloudData->at(id).z()) && is_kinect2_depth_valid(m_p->depthDataTemporal->at(id).z())){
                m_p->depthDataTemporal->at(id).z() = 0.5f*(m_p->cloudData->at(id).z()+m_p->depthDataTemporal->at(id).z());
            }
        });
        std::swap(m_p->cloudData,m_p->depthDataTemporal);
    }

    Bench::stop();
    Bench::start("Kinect2::post_computing_depth_data_5"sv);

    if(parameters.offsetAfterMin > 0.f){
        float min = std::numeric_limits<float>::max();
        for_each(execution::seq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){            

            const auto depthZ = (*m_p->cloudData)[id].z();
            if(!is_kinect2_depth_valid(depthZ)){
                return;
            }
            if(depthZ < min){
                min = depthZ;
            }

        });
        for_each(execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){
            if((*m_p->cloudData)[id].z() > min + parameters.offsetAfterMin){
                (*m_p->cloudData)[id].z() = invalid_value;
            }
        });
    }

    // test
    if(parameters.vmin > 0.f){
        const auto squareMaxDist = parameters.vmin*parameters.vmin;
        Pt3f mean = {};
        size_t count = 0;
        for_each(std::execution::seq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){

            const auto depthZ = (*m_p->cloudData)[id].z();
            if(!is_kinect2_depth_valid(depthZ)){
                return;
            }

            mean += (*m_p->cloudData)[id];
            count++;
        });
        if(count > 0){
            mean /= count;
        }

        std::cout << squareMaxDist << " " << mean << "\n";
        for_each(execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){
            if(square_norm(Vec3f{geo::vec((*m_p->cloudData)[id], mean)}) > squareMaxDist){
                (*m_p->cloudData)[id].z() = invalid_value;
            }
        });
    }

    Bench::stop();
}

void Kinect2::post_computing_infra_data(){

    using namespace std;
    uint16_t validV   = parameters.infraInvert ? 0 : 1;
    uint16_t invalidV = parameters.infraBinary ? (parameters.infraInvert ? 1 : 0) : 0;

    transform(execution::par_unseq, begin(*m_p->processedFrame->infraData), end(*m_p->processedFrame->infraData), begin(*m_p->processedFrame->infraData),[&](uint16_t inf){
        if(inf < parameters.minInfra || inf > parameters.maxInfra){
            return invalidV;
        }else if(parameters.infraBinary){
            return validV;
        }
        return inf;
    });

    auto minMax = minmax_element(begin(*m_p->processedFrame->infraData),end(*m_p->processedFrame->infraData));
    auto min = (*m_p->processedFrame->infraData)[static_cast<size_t>((minMax.first - begin(*m_p->processedFrame->infraData)))];
    auto max = (*m_p->processedFrame->infraData)[static_cast<size_t>((minMax.second - begin(*m_p->processedFrame->infraData)))];

    auto diff = max - min;
    if(diff==0){
        diff = 1;
    }

    auto diffRange = parameters.maxInfraRange-parameters.minInfraRange;
    auto minV = static_cast<double>(numeric_limits<uint16_t>::min());
    auto maxV = static_cast<double>(numeric_limits<uint16_t>::max());
    transform(execution::par_unseq, begin(*m_p->processedFrame->infraData), end(*m_p->processedFrame->infraData), begin(*m_p->processedFrame->infraData),[&](uint16_t inf){
        return static_cast<uint16_t>(clamp<double>(((1.*inf -min)/diff)*diffRange + parameters.minInfraRange, minV, maxV));
    });
}

void Kinect2::post_computing_body_data(){

    for(auto &body : m_p->processedFrame->bodiesData){
        for(auto &joint : body.joints){
            const auto &pos = joint.second.pos;
            if(pos.z() < parameters.minDepthValue || pos.z() > parameters.maxDepthValue){
                joint.second.state = TrackingStateT::not_tracked;
            }
        }
    }
}

void Kinect2::process_depth_512x424(){

    // copy depth
    std::copy(m_p->depthBuffer, m_p->depthBuffer + m_p->depthBufferSize, m_p->processedFrame->depthData->data());

    // reverse depth
    for(size_t ii = 0; ii < depth_height; ++ii){
        auto *col = &(*m_p->processedFrame->depthData)[ii*depth_width];
        std::reverse(col, col + depth_width-1);
    }
}

void Kinect2::process_infra_512x424(){
    // reverse infra
    for(size_t ii = 0; ii < infrared_height; ++ii){
        auto *col = &(*m_p->processedFrame->infraData)[ii*infrared_width];
        std::reverse(col, col + infrared_width-1);
    }
}

void Kinect2::process_compressed_color_1920x1080(){

    // reverse colors
    for(size_t ii = 0; ii < color_height; ++ii){
        auto *col = &(*m_p->colorData)[ii*color_width];
        std::reverse(col, col + color_width-1);
    }

    const int jpegQuality = parameters.jpegCompressionRate;
    unsigned char *buffer = reinterpret_cast<unsigned char*>(m_p->colorData->data());
    const int width  = color_width;
    const int height = color_height;

    // compress color (hight cost)
    int ret = tjCompress2(m_p->jpegCompressor, buffer, width, 0, height, TJPF_RGB,
              &m_p->tjCompressedImage, &m_p->processedFrame->jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_FASTDCT);
    if(ret == -1){
        Logger::error(("tjCompress2 error with code: ") + std::to_string(tjGetErrorCode(m_p->jpegCompressor)));
        return;
    }

    if(m_p->processedFrame->compressedImage.size() < m_p->processedFrame->jpegColorSize){
        m_p->processedFrame->compressedImage.resize(m_p->processedFrame->jpegColorSize);
    }
    std::copy(m_p->tjCompressedImage, m_p->tjCompressedImage + m_p->processedFrame->jpegColorSize, std::begin(m_p->processedFrame->compressedImage));
}

void Kinect2::process_compressed_color_512x424(){

    std::for_each(std::execution::par_unseq, std::begin(m_p->indicesDepths), std::end(m_p->indicesDepths), [&](size_t &id){

        if(is_kinect2_depth_valid((*m_p->cloudData)[id].z()) && m_p->validityDepth2Rgb[id]){
            // color
//            const auto &m = m_p->depth2rgb[id];
            size_t idX =m_p->indicesDepthToColor[id];// {static_cast<size_t>(m.X) + kinect2_color_width*(static_cast<size_t>(m.Y))};
            (*m_p->processedColorData)[id] = {(*m_p->colorData)[idX].x(),(*m_p->colorData)[idX].y(),(*m_p->colorData)[idX].z()};
        }else{
            (*m_p->processedColorData)[id] = {0,0,0};
        }
    });

    // reverse colors
    for(size_t ii = 0; ii < depth_height; ++ii){
        auto *col = &(*m_p->processedColorData)[ii*depth_width];
        std::reverse(col, col + depth_width-1);
    }

    const int jpegQuality = parameters.jpegCompressionRate;
    unsigned char *buffer = reinterpret_cast<unsigned char*>(m_p->processedColorData->data());
    const int width  = depth_width;
    const int height = depth_height;

    int ret = tjCompress2(m_p->jpegCompressor, buffer, width, 0, height, TJPF_RGB,
              &m_p->tjCompressedImage, &m_p->processedFrame->jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_FASTDCT);
    if(ret == -1){
        Logger::error(("tjCompress2 error with code: ") + std::to_string(tjGetErrorCode(m_p->jpegCompressor)));
        return;
    }

    if(m_p->processedFrame->compressedImage.size() < m_p->processedFrame->jpegColorSize){
        m_p->processedFrame->compressedImage.resize(m_p->processedFrame->jpegColorSize);
    }
    std::copy(m_p->tjCompressedImage, m_p->tjCompressedImage + m_p->processedFrame->jpegColorSize, std::begin(m_p->processedFrame->compressedImage));
}


void Kinect2::process_compressed_color_cloud(){

    using namespace std;

    for_each(execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){

        if(is_kinect2_depth_valid((*m_p->cloudData)[id].z()) && m_p->validityDepth2Rgb[id]){
            // color
            const auto &m = m_p->depth2rgb[id];
            size_t idX = {static_cast<size_t>(m.X) + color_width*(static_cast<size_t>(m.Y))};
            (*m_p->processedColorData)[id] = {(*m_p->colorData)[idX].x(),(*m_p->colorData)[idX].y(),(*m_p->colorData)[idX].z()};
            (*m_p->processedFrame->depthData)[id] = static_cast<uint16_t>(std::clamp((*m_p->cloudData)[id].z(),0.f,8.f)*1000.f);

        }else{
            (*m_p->processedColorData)[id] = geo::Pt3<uint8_t>{0,0,0};
            (*m_p->processedFrame->depthData)[id] = 0;
        }
    });

//    // reverse colors
//    for(size_t ii = 0; ii < kinect2_depth_height; ++ii){
//        auto *col = &(*m_p->processedColorData)[ii*kinect2_depth_width];
//        reverse(col, col + kinect2_depth_width-1);
//    }

//    // reverse depth
//    for(size_t ii = 0; ii < kinect2_depth_height; ++ii){
//        auto *d = &(*m_p->processedFrame->depthData)[ii*kinect2_depth_width];
//        reverse(d, d + kinect2_depth_width-1);
//    }


    const int jpegQuality = parameters.jpegCompressionRate;
    unsigned char *buffer = reinterpret_cast<unsigned char*>(m_p->processedColorData->data());
    const int width  = depth_width;
    const int height = depth_height;

    int ret = tjCompress2(m_p->jpegCompressor, buffer, width, 0, height, TJPF_RGB,
              &m_p->tjCompressedImage, &m_p->processedFrame->jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_FASTDCT);
    if(ret == -1){
        Logger::error(("tjCompress2 error with code: ") + std::to_string(tjGetErrorCode(m_p->jpegCompressor)));
        return;
    }

    if(m_p->processedFrame->compressedImage.size() < m_p->processedFrame->jpegColorSize){
        m_p->processedFrame->compressedImage.resize(m_p->processedFrame->jpegColorSize);
    }
    std::copy(m_p->tjCompressedImage, m_p->tjCompressedImage + m_p->processedFrame->jpegColorSize, std::begin(m_p->processedFrame->compressedImage));

    // compress depth integers casted in 32 bits
    m_p->processedFrame->compressedDepthSize = m_p->depthCompressor.encode(
        reinterpret_cast<uint32_t*>(m_p->processedFrame->depthData->data()), depth_count/2,
        reinterpret_cast<uint32_t*>(m_p->processedFrame->compressedDepthData.data()), depth_count/2 + 1024
    );
}

void Kinect2::process_compressed_color_mesh(){

    using namespace std;

    for_each(std::execution::par_unseq, begin(m_p->indicesDepths), end(m_p->indicesDepths), [&](size_t &id){

        if(is_kinect2_depth_valid((*m_p->cloudData)[id].z()) && m_p->validityDepth2Rgb[id]){
            // color
            const auto &m = m_p->depth2rgb[id];
            size_t idX = {static_cast<size_t>(m.X) + color_width*(static_cast<size_t>(m.Y))};
            (*m_p->processedColorData)[id] = {(*m_p->colorData)[idX].x(),(*m_p->colorData)[idX].y(),(*m_p->colorData)[idX].z()};
            (*m_p->processedFrame->depthData)[id] = static_cast<uint16_t>(std::clamp((*m_p->cloudData)[id].z(),0.f,8.f)*1000.f);

        }else{
            (*m_p->processedColorData)[id] = geo::Pt3<uint8_t>{0,0,0};
            (*m_p->processedFrame->depthData)[id] = 0;
        }
    });

//    // reverse colors
//    for(size_t ii = 0; ii < kinect2_depth_height; ++ii){
//        auto *col = &(*m_p->processedColorData)[ii*kinect2_depth_width];
//        reverse(col, col + kinect2_depth_width-1);
//    }

//    // reverse depth
//    for(size_t ii = 0; ii < kinect2_depth_height; ++ii){
//        auto *d = &(*m_p->processedFrame->depthData)[ii*kinect2_depth_width];
//        reverse(d, d + kinect2_depth_width-1);
//    }

    const int jpegQuality = parameters.jpegCompressionRate;
    unsigned char *buffer= reinterpret_cast<unsigned char*>(m_p->processedColorData->data());
    const int width  = depth_width;
    const int height = depth_height;

    int ret = tjCompress2(m_p->jpegCompressor, buffer, width, 0, height, TJPF_RGB,
              &m_p->tjCompressedImage, &m_p->processedFrame->jpegColorSize, TJSAMP_444, jpegQuality, TJFLAG_FASTDCT);
    if(ret == -1){
        Logger::error(("tjCompress2 error with code: ") + std::to_string(tjGetErrorCode(m_p->jpegCompressor)));
        return;
    }

    if(m_p->processedFrame->compressedImage.size() < m_p->processedFrame->jpegColorSize){
        m_p->processedFrame->compressedImage.resize(m_p->processedFrame->jpegColorSize);
    }
    std::copy(m_p->tjCompressedImage, m_p->tjCompressedImage + m_p->processedFrame->jpegColorSize, std::begin(m_p->processedFrame->compressedImage));

    // compress depth integers casted in 32 bits
    m_p->processedFrame->compressedDepthSize = m_p->depthCompressor.encode(
        reinterpret_cast<uint32_t*>(m_p->processedFrame->depthData->data()), depth_count/2,
        reinterpret_cast<uint32_t*>(m_p->processedFrame->compressedDepthData.data()), depth_count/2 + 1024
    );
}

void Kinect2::release_color_data(){
    if (m_p->colorframe != nullptr){
        m_p->colorframe->Release();
        m_p->colorframe = nullptr;
    }
    if (m_p->colorFrameRef != nullptr){
        m_p->colorFrameRef->Release();
        m_p->colorFrameRef = nullptr;
    }
}

void Kinect2::release_depth_data(){
    if (m_p->depthFrame != nullptr){
        m_p->depthFrame->Release();
        m_p->depthFrame = nullptr;
    }
    if (m_p->depthFrameRef != nullptr){
        m_p->depthFrameRef->Release();
        m_p->depthFrameRef = nullptr;
    }
}

void Kinect2::release_infra_data(){
    if (m_p->infraFrame != nullptr){
        m_p->infraFrame->Release();
        m_p->infraFrame = nullptr;
    }
    if (m_p->infraFrameRef != nullptr){
        m_p->infraFrameRef->Release();
        m_p->infraFrameRef = nullptr;
    }
}

void Kinect2::release_long_exposure_infra_data(){
    if (m_p->longExposureInfraFrame != nullptr){
        m_p->longExposureInfraFrame->Release();
        m_p->longExposureInfraFrame = nullptr;
    }
    if (m_p->longExposureInfraFrameRef != nullptr){
        m_p->longExposureInfraFrameRef->Release();
        m_p->longExposureInfraFrameRef = nullptr;
    }
}

void Kinect2::release_body_data(){

    if (m_p->bodyFrame != nullptr){
        m_p->bodyFrame->Release();
        m_p->bodyFrame = nullptr;
    }
    if (m_p->bodyFrameRef != nullptr){
        m_p->bodyFrameRef->Release();
        m_p->bodyFrameRef = nullptr;
    }
}

void Kinect2::release_multi_source_data(){

    if (m_p->multiSouceFrame != nullptr){
        m_p->multiSouceFrame->Release();
        m_p->multiSouceFrame = nullptr;
    }
}

void Kinect2::clean_frame(){
    release_depth_data();
    release_color_data();
    release_infra_data();
    release_long_exposure_infra_data();
    release_body_data();
    release_multi_source_data();
}




//Pt4<float> rgb_to_hsl(const Pt4<std::uint8_t> &rgba){

//    float H,S,L;

//    float r = (rgba[0] / 255.0f);
//    float g = (rgba[1] / 255.0f);
//    float b = (rgba[2] / 255.0f);

//    float min = std::min(std::min(r, g), b);
//    float max = std::max(std::max(r, g), b);
//    float delta = max - min;

//    L = (max + min) / 2;
//    if (almost_equal(delta,0.f)){
//        H = 0.f;
//        S = 0.0f;
//    }else{
//        S = (L <= 0.5f) ? (delta / (max + min)) : (delta / (2 - max - min));

//        float hue;

//        if (almost_equal(r,max)){
//            hue = ((g - b) / 6.f) / delta;
//        }else if (almost_equal(g,max)){
//            hue = (1.0f / 3) + ((b - r) / 6.f) / delta;
//        }
//        else{
//            hue = (2.0f / 3.f) + ((r - g) / 6.f) / delta;
//        }

//        if (hue < 0.f){
//            hue += 1.f;
//        }
//        if (hue > 1.f){
//            hue -= 1.f;
//        }

//        H = hue * 360.f;
//    }
//    return Pt4<float>{H,S,L,1.f*rgba.w()};
//}

//float hue_to_rgb(float v1, float v2, float vH) {

//    if (vH < 0.f){
//        vH += 1.f;
//    }

//    if (vH > 1.f){
//        vH -= 1.f;
//    }

//    if ((6.f * vH) < 1.f){
//        return (v1 + (v2 - v1) * 6.f * vH);
//    }

//    if ((2.f * vH) < 1.f){
//        return v2;
//    }

//    if ((3.f * vH) < 2.f){
//        return (v1 + (v2 - v1) * ((2.0f / 3.f) - vH) * 6.f);
//    }

//    return v1;
//}

//Pt4<std::uint8_t> hsl_to_rgb(Pt4<float> hsva){

//    float H = hsva.x();
//    float S = hsva.y();
//    float L = hsva.z();

//    if (almost_equal(S,0.f)){
//        auto v = static_cast<std::uint8_t>(L * 255.f);
//        return Pt4<std::uint8_t>{v,v,v,static_cast<std::uint8_t>(hsva.w())};
//    }else{
//        float v1, v2;
//        float hue = H / 360.f;

//        v2 = (L < 0.5f) ? (L * (1 + S)) : ((L + S) - (L * S));
//        v1 = 2 * L - v2;

//        return Pt4<std::uint8_t>{static_cast<std::uint8_t>(255 * hue_to_rgb(v1, v2, hue + (1.0f / 3.f))),
//                        static_cast<std::uint8_t>(255 * hue_to_rgb(v1, v2, hue)),
//                        static_cast<std::uint8_t>(255 * hue_to_rgb(v1, v2, hue - (1.0f / 3.f))),static_cast<std::uint8_t>(hsva.w())};
//    }
//}



//void Kinect2::process_color_1920x1080(){
//    // reverse colors
//    for(size_t ii = 0; ii < kinect2_color_height; ++ii){
//        auto *col = &(*colorData)[ii*kinect2_color_width];
//        std::reverse(col, col + kinect2_color_width-1);
//    }
//}

//void Kinect2::process_color_512x424(){

//    std::for_each(std::execution::par_unseq, std::begin(m_p->indicesDepths), std::end(m_p->indicesDepths), [&](size_t &id){

//        if(!m_p->validityDepth2Rgb[id]){
//            (*processedColorData)[id] = {0,0,0,255};
//        }else{
//            // color
//            const auto &m = m_p->depth2rgb[id];
//            size_t idX = {static_cast<size_t>(m.X) + kinect2_color_width*(static_cast<size_t>(m.Y))};
//            (*processedColorData)[id] = {(*colorData)[idX].x(),(*colorData)[idX].y(),(*colorData)[idX].z(), 255};
//        }
//    });

//    // reverse colors
//    for(size_t ii = 0; ii < kinect2_depth_height; ++ii){
//        auto *col = &(*processedColorData)[ii*kinect2_depth_width];
//        std::reverse(col, col + kinect2_depth_width-1);
//    }
//}

//void Kinect2::process_cloud(){


//    currentCloudSize = 0;
//    std::for_each(std::execution::seq, std::begin(m_p->indicesDepths), std::end(m_p->indicesDepths), [&](size_t &id){

//        if(is_kinect2_depth_valid((*cloudData)[id].z()) && m_p->validityDepth2Rgb[id]){
//            // pos
//            (*processedCloudData)[currentCloudSize] = (*cloudData)[id];
//            // color
//            const auto &m = m_p->depth2rgb[id];
//            size_t idx = (static_cast<unsigned int>(m.X)) + kinect2_color_width*(static_cast<unsigned int>(m.Y));
//            (*processedColorData)[currentCloudSize] = (*colorData)[idx];
//            // cloud size
//            currentCloudSize++;
//        }else{
//            // color
//            (*processedColorData)[currentCloudSize] = {0,0,0,255};
//        }
//    });
//}

//void Kinect2::process_mesh(){

//    std::uint32_t idT = 0;
//    currentCloudSize = 0;
//    std::for_each(std::execution::seq, std::begin(m_p->indicesDepths), std::end(m_p->indicesDepths), [&](size_t &id){
//        if(is_kinect2_depth_valid((*cloudData)[id].z()) && m_p->validityDepth2Rgb[id]){

//            // pos
//            (*processedCloudData)[currentCloudSize] = (*cloudData)[id];
//            // color
//            const auto &m = m_p->depth2rgb[id];
//            size_t idx = (static_cast<unsigned int>(m.X)) + kinect2_color_width*(static_cast<unsigned int>(m.Y));
//            (*processedColorData)[currentCloudSize] = (*colorData)[idx];
//            // cloud size
//            currentCloudSize++;
//            // triangle id validity
//            m_p->validIdPerPointMesh[id] = idT++;
//        }else{
//            // color
//            (*processedColorData)[id] = {0,0,0,255};
//            // triangle id validity
//            m_p->validIdPerPointMesh[id] = idT;
//        }
//    });

//    idT = 0;
//    for(std::uint32_t ii = 0; ii < kinect2_depth_width-1; ++ii){
//        for(std::uint32_t jj = 0; jj < kinect2_depth_height-1; ++jj){

//            const std::uint32_t idTopLeft     = jj*kinect2_depth_width+ii;
//            const std::uint32_t idTopRight    = (jj)*kinect2_depth_width+(ii+1);
//            const std::uint32_t idBottomLeft  = (jj+1)*kinect2_depth_width+(ii);
//            const std::uint32_t idBottomRight = (jj+1)*kinect2_depth_width+(ii+1);

//            const bool idTopLeftValid       = is_kinect2_depth_valid((*cloudData)[idTopLeft].z())     && m_p->validityDepth2Rgb[idTopLeft];
//            const bool idTopRightValid      = is_kinect2_depth_valid((*cloudData)[idTopRight].z())    && m_p->validityDepth2Rgb[idTopRight];
//            const bool idBottomLeftValid    = is_kinect2_depth_valid((*cloudData)[idBottomLeft].z())  && m_p->validityDepth2Rgb[idBottomLeft];
//            const bool idBottomRightValid   = is_kinect2_depth_valid((*cloudData)[idBottomRight].z()) && m_p->validityDepth2Rgb[idBottomRight];

//            int invalidCount = (idTopLeftValid? 1 : 0) + (idTopRightValid? 1 : 0) + (idBottomLeftValid? 1 : 0) + (idBottomRightValid? 1 : 0);
//            if(invalidCount < 3){
//                continue;
//            }

//            if(invalidCount == 4){
//                if(std::abs((*cloudData)[idTopLeft].z()-(*cloudData)[idBottomRight].z()) < std::abs((*cloudData)[idTopRight].z()-(*cloudData)[idBottomLeft].z())){
//                    (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopLeft],
//                                                                  m_p->validIdPerPointMesh[idBottomRight],
//                                                                  m_p->validIdPerPointMesh[idBottomLeft]};
//                    (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopLeft],
//                                                                  m_p->validIdPerPointMesh[idTopRight],
//                                                                  m_p->validIdPerPointMesh[idBottomRight]};
//                }else{
//                    (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopLeft],
//                                                                  m_p->validIdPerPointMesh[idTopRight],
//                                                                  m_p->validIdPerPointMesh[idBottomLeft]};
//                    (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopRight],
//                                                                  m_p->validIdPerPointMesh[idBottomRight],
//                                                                  m_p->validIdPerPointMesh[idBottomLeft]};
//                }
//                continue;
//            }

//            if(!idTopLeftValid){
//                (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopRight],
//                                                              m_p->validIdPerPointMesh[idBottomRight],
//                                                              m_p->validIdPerPointMesh[idBottomLeft]};
//            }
//            if(!idTopRightValid){
//                (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopLeft],
//                                                              m_p->validIdPerPointMesh[idBottomRight],
//                                                              m_p->validIdPerPointMesh[idBottomLeft]};
//            }
//            if(!idBottomLeftValid){
//                (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopLeft],
//                                                              m_p->validIdPerPointMesh[idTopRight],
//                                                              m_p->validIdPerPointMesh[idBottomRight]};
//            }
//            if(!idBottomRightValid){
//                (*processedtrianglesData)[idT++] = Pt3<std::uint32_t>{m_p->validIdPerPointMesh[idTopLeft],
//                                                              m_p->validIdPerPointMesh[idTopRight],
//                                                            m_p->validIdPerPointMesh[idBottomLeft]};
//            }
//        }
//    }

//}

