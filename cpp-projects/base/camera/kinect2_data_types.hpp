
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

// std
#include <mutex>
#include <optional>
#include <sstream>

// base
#include "geometry/point2.hpp"
#include "geometry/point3.hpp"
#include "geometry/point4.hpp"
#include "geometry/voxel.hpp"
#include "utility/vector.hpp"
#include "utility/string.hpp"
#include "utility/tuple_array.hpp"

namespace tool::camera::K2{

using namespace std::literals::string_view_literals;

enum class Detection : std::uint8_t{
    unknow= 0,
    no,
    maybe,
    yes,
};

enum class HandStateT : std::uint8_t{
    unknow= 0,
    not_tracked,
    open,
    closed,
    lasso
};

enum class TrackingStateT : std::uint8_t{
    not_tracked=0, // body tracked
    inferred,
    tracked
};

enum class BodyJointType : std::uint8_t{
    spine_base	= 0,
    spine_mid,
    neck,
    head,
    shoulder_left,
    elbow_left,
    wrist_left,
    hand_left,
    shoulder_right,
    elbow_right,
    wrist_right,
    hand_right,
    hip_left,
    knee_left,
    ankle_left,
    foot_left,
    hip_right,
    knee_right,
    ankle_right,
    foot_right,
    spine_shoulder,
    hand_tip_left,
    thumb_left,
    hand_tip_right,
    thumb_right,
    SizeEnum
};


using SV = std::string_view;
using JointName = SV;
using TJoint = std::tuple<
    BodyJointType,                          JointName>;
[[maybe_unused]] static constexpr TupleArray<BodyJointType::SizeEnum, TJoint> jointsM{{
    TJoint{BodyJointType::spine_base,       "spine_base"sv},
    TJoint{BodyJointType::spine_mid,        "spine_mid"sv},
    TJoint{BodyJointType::neck,             "neck"sv},
    TJoint{BodyJointType::head,             "head"sv},
    TJoint{BodyJointType::shoulder_left,    "shoulder_left"sv},
    TJoint{BodyJointType::elbow_left,       "elbow_left"sv},
    TJoint{BodyJointType::wrist_left,       "wrist_left"sv},
    TJoint{BodyJointType::hand_left,        "hand_left"sv},
    TJoint{BodyJointType::shoulder_right,   "shoulder_right"sv},
    TJoint{BodyJointType::elbow_right,      "elbow_right"sv},
    TJoint{BodyJointType::wrist_right,      "wrist_right"sv},
    TJoint{BodyJointType::hand_right,       "hand_right"sv},
    TJoint{BodyJointType::hip_left,         "hip_left"sv},
    TJoint{BodyJointType::knee_left,        "knee_left"sv},
    TJoint{BodyJointType::ankle_left,       "ankle_left"sv},
    TJoint{BodyJointType::foot_left,        "foot_left"sv},
    TJoint{BodyJointType::hip_right,        "hip_right"sv},
    TJoint{BodyJointType::knee_right,       "knee_right"sv},
    TJoint{BodyJointType::ankle_right,      "ankle_right"sv},
    TJoint{BodyJointType::foot_right,       "foot_right"sv},
    TJoint{BodyJointType::spine_shoulder,   "spine_shoulder"sv},
    TJoint{BodyJointType::hand_tip_left,    "hand_tip_left"sv},
    TJoint{BodyJointType::thumb_left,       "thumb_left"sv},
    TJoint{BodyJointType::hand_tip_right,   "hand_tip_right"sv},
    TJoint{BodyJointType::thumb_right,      "thumb_right"sv},
}};


enum class FrameRequest : uint8_t {
    compressed_color_cloud=0,
    compressed_color_mesh,
    compressed_color_512x424,
    compressed_color_1920x1080,
    depth_512x424,                  // 512 * 424 * (sizeof(float)) -> d
    infra_512x424,                  // 512 * 424 * (sizeof(ushort)) -> v
    long_exposure_infra_512x424,    // 512 * 424 * (sizeof(ushort)) -> v
    undefined,
    SizeEnum
};

enum MorphShapes : unsigned char{
    MORPH_RECT    = 0,
    MORPH_CROSS   = 1,
    MORPH_ELLIPSE = 2
};

// constants / aliases
// # color (Pt3 uint8 / Pt4 uint8)
constexpr std::uint32_t color_width          = 1920;
constexpr std::uint32_t color_height         = 1080;
constexpr std::uint32_t color_count          = color_width * color_height;
[[maybe_unused]] constexpr std::uint32_t color_data_size      = color_count * 4;
[[maybe_unused]] constexpr std::uint32_t raw_color_data_size  = color_count * 2;
using RgbData    = std_a1<geo::Pt3<std::uint8_t>,color_count>;
using RgbDataUP  = std::unique_ptr<RgbData>;
using RgbaData   = std_a1<geo::Pt4<std::uint8_t>,color_count>;
using RgbaDataUP = std::unique_ptr<RgbaData>;

// #  infrared
constexpr std::uint32_t infrared_width      = 512;
constexpr std::uint32_t infrared_height     = 424;
constexpr std::uint32_t infrared_count      = infrared_width * infrared_height;
[[maybe_unused]] constexpr std::uint32_t infrared_data_size  = infrared_count * 2;
using InfraData      = std_a1<std::uint16_t,infrared_count>;
using InfraDataUP    = std::unique_ptr<InfraData>;

// #  depth (Int16)
constexpr std::uint32_t depth_width      = 512;
constexpr std::uint32_t depth_height     = 424;
constexpr std::uint32_t depth_count      = depth_width * depth_height;
[[maybe_unused]] constexpr std::uint32_t depth_data_size  = depth_count * 2;
using DepthData   = std_a1<uint16_t,(depth_count)>;
using DepthDataUP = std::unique_ptr<DepthData>;

// #  cloud (Pt3f)
using CloudData   = std_a1<geo::Pt3f,depth_count>;
using CloudDataUP = std::unique_ptr<CloudData>;

// #  colored cloud (Voxel)
using ColoredCloudData   = std_a1<geo::Voxel,depth_count>;
using ColoredCloudDataUP = std::unique_ptr<ColoredCloudData>;

// #  triangles id
constexpr std::uint32_t triangles_count  = (depth_width-1) * (depth_height-1) * 2;
[[maybe_unused]] constexpr std::uint32_t triangles_id_count = triangles_count * 3;

using TrianglesData   = std_a1<geo::Pt3<std::uint32_t>, triangles_count>;
using TrianglesDataUP = std::unique_ptr<TrianglesData>;

// # bodies joint
constexpr std::uint32_t body_joint_data_size64    = 4;
constexpr std::uint32_t body_joints_count         = 25;
constexpr std::uint32_t bodies_count              = 6;
constexpr std::uint32_t bodies_joints_count       = bodies_count * body_joints_count;
constexpr std::uint32_t bodies_joints_data_size64 = bodies_joints_count * body_joint_data_size64;
[[maybe_unused]] constexpr std::uint32_t bodies_joints_data_size8  = bodies_joints_data_size64 * 8;

constexpr std::uint32_t voxels_data_size64           = depth_count;
constexpr std::uint32_t triangles_data_size64        = triangles_count;

constexpr float invalid_value = -10.f;
[[maybe_unused]] constexpr geo::Pt3f invalid_point(invalid_value,invalid_value,invalid_value);

// mode mapping
// # for each component category : name
using Requirment  = bool;
using ColorR      = Requirment;
using DepthR      = Requirment;
using InfraR      = Requirment;
using LongInfraR  = Requirment;
using BodyR       = Requirment;
using WidthFrame  = int;
using HeightFrame = int;
using FrameName   = SV;
using FR        = FrameRequest;

using TFrame = std::tuple<
           FR,                              ColorR, DepthR, InfraR, LongInfraR, BodyR, WidthFrame,       HeightFrame,     FrameName>;
static constexpr TupleArray<FR::SizeEnum, TFrame> frames= {{
    TFrame{FR::compressed_color_cloud,      true,   true,   false,  false,      true,  depth_width,      depth_height,    "compressed_color_cloud"sv},
    TFrame{FR::compressed_color_mesh,       true,   true,   false,  false,      true,  depth_width,      depth_height,    "compressed_color_mesh"sv},
    TFrame{FR::compressed_color_512x424,    true,   true,   false,  false,      false, depth_width,      depth_height,    "compressed_image_color_512x424"sv},
    TFrame{FR::compressed_color_1920x1080,  true,   false,  false,  false,      false, color_width,      color_height,    "compressed_image_color_1920x1080"sv},
    TFrame{FR::depth_512x424,               false,  true,   false,  false,      false, depth_width,      depth_height,    "image_depth_512x424"sv},
    TFrame{FR::infra_512x424,               false,  false,  true,   false,      false, infrared_width,   infrared_height, "image_infra_512x424"sv},
    TFrame{FR::long_exposure_infra_512x424, false,  false,  false,  true,       false, infrared_width,   infrared_height, "image_long_exposure_infra_512x424"sv},
    TFrame{FR::undefined,                   false,  false,  false,  false,      false, 0,                0,               "undefined"sv},
}};

[[maybe_unused]] static bool color_channel_required(FrameRequest r) {
    return frames.at<0,1>(r);
}
[[maybe_unused]] static bool depth_channel_required(FrameRequest r) {
    return frames.at<0,2>(r);
}
[[maybe_unused]] static bool infra_channel_required(FrameRequest r) {
    return frames.at<0,3>(r);
}
[[maybe_unused]] static bool long_infra_channel_required(FrameRequest r) {
    return frames.at<0,4>(r);
}
[[maybe_unused]] static bool body_channel_required(FrameRequest r) {
    return frames.at<0,5>(r);
}
[[maybe_unused]] static auto all_requests_names(){
    return frames.elements_not_matching_columns_values<0,8>(FR::undefined);
}

[[maybe_unused]] static constexpr FrameRequest index_to_mode(size_t id){
    if(id < static_cast<uint8_t>(FR::undefined) && id >= 0){
        return static_cast<FR>(id);
    }
    return FR::undefined;
}

[[maybe_unused]] static constexpr uint8_t mode_to_index(FrameRequest r){
    return static_cast<uint8_t>(r);
}


//static constexpr std::array<std::tuple<K2FR, bool, bool, bool, bool, bool, bool, bool, int, int, const char*>,static_cast<size_t>(K2FR::SizeEnum)> kinect2Mapping ={{
//    /** mode                             color  depth  infra  long_inf body   body_i audio  width                    height                   name*/
//    {K2FR::compressed_color_cloud,       true,  true,  false, false,   true,  false, false, k2_depth_width,     k2_depth_height,    "compressed_color_cloud"},
//    {K2FR::compressed_color_mesh,        true,  true,  false, false,   true,  false, false, k2_depth_width,     k2_depth_height,    "compressed_color_mesh"},
//    {K2FR::compressed_color_512x424,     true,  true,  false, false,   false, false, false, k2_depth_width,     k2_depth_height,    "compressed_image_color_512x424"},
//    {K2FR::compressed_color_1920x1080,   true,  false, false, false,   false, false, false, k2_color_width,     k2_color_height,    "compressed_image_color_1920x1080"},
//    {K2FR::depth_512x424,                false, true,  false, false,   false, false, false, k2_depth_width,     k2_depth_height,    "image_depth_512x424"},
//    {K2FR::infra_512x424,                false, false, true,  false,   false, false, false, k2_infrared_width,  k2_infrared_height, "image_infra_512x424"},
//    {K2FR::long_exposure_infra_512x424,  false, false, false, true,    false, false, false, k2_infrared_width,  k2_infrared_height, "image_long_exposure_infra_512x424"},
//    {K2FR::undefined,                    false, false, false, false,   false, false, false, 0,                       0,                       "undefined"},
//}};




//static constexpr std::optional<const char*> name(K2FrameRequest mode) {
//    for (auto& m : kinect2Mapping) {
//        if (mode == std::get<0>(m)) {
//            return std::get<10>(m);
//        }
//    }
//    return {};
//}



//static constexpr std::optional<int> width(K2FrameRequest mode) {
//    for (auto& m : kinect2Mapping) {
//        if (mode == std::get<0>(m)) {
//            return std::get<8>(m);
//        }
//    }
//    return {};
//}
//static constexpr std::optional<int> height(K2FrameRequest mode) {
//    for (auto& m : kinect2Mapping) {
//        if (mode == std::get<0>(m)) {
//            return std::get<9>(m);
//        }
//    }
//    return {};
//}


using CloudInt64Data      = std_a1<std::uint64_t, voxels_data_size64 + bodies_joints_data_size64>;
using CloudInt64DataUP    = std::unique_ptr<CloudInt64Data>;

using MeshInt64Data       = std_a1<std::uint64_t, voxels_data_size64 + triangles_data_size64 + bodies_joints_data_size64>;
using MeshInt64DataUP     = std::unique_ptr<MeshInt64Data>;

// network types
//struct Voxel64{
//    std::uint64_t x : 14, y : 14, z : 12, r : 8, g : 8, b : 8;
//};
//struct Voxel5_64{ // 5 depths + visibility
//    std::uint64_t d1 : 12, d2 : 12, d3 : 12, d4 : 12, d5 : 12, v1 : 1, v2 : 1, v3 : 1, v4 : 1, v5 : 1, r : 1;
//};
// (512*424 217088) (43418*5 217090) (rest 2) -> 0.33mo -> 33mo (100fps)

//struct Color_x64{
//    std::uint64_t c1 : 8, c2 : 8, c3 : 8, c4 : 8, c5 : 8, c6 : 8, c7 : 8, c8 : 8; // 2 pixels + 2 composantes
//};
// (512*424*3 651264) (81408*8 651264) (rest 0) -> 0.62mo -> 62mo (100fps)
// -> 100 mo (100 fps)

struct Joint4x64{
    std::uint64_t jointPosX : 32, jointPosY : 32;
    std::uint64_t jointPosZ : 32, jointQuaX : 32;
    std::uint64_t jointQuaY : 32, jointQuaZ : 32;
    std::uint64_t jointQuaW : 32, jointState : 2, jointid : 5,
                  bodyId : 17, bodyTracked : 1, bodyRestricted : 1,
                  leftHandState : 2, rightHandState : 2, leftHandConfidence : 1, rightHandConfidence : 1;
};
//struct TriId64{
//    std::uint64_t id1 : 21, id2 : 21, id3 : 21, e = 1; // 63/64
//};

// convert functions
//static auto voxel_to_int(const geo::Voxel &v);
//static auto int_to_voxel(const Voxel64 &vi);


struct BodyJoint{
    geo::Pt3f pos;
    geo::Pt4f rotQuaternion;
    TrackingStateT state  = TrackingStateT::not_tracked;
};

struct BodyInfos{
    std::uint64_t id = 0;
    bool tracked = false;
    bool restricted = false;
    Detection engaged        = Detection::unknow;
    HandStateT leftHandState  = HandStateT::unknow;
    HandStateT rightHandState = HandStateT::unknow;
    bool leftHandHightConfidence    = false;
    bool rightHandHightConfidence   = false;
    geo::Pt2f lean;
    TrackingStateT leanTracking = TrackingStateT::not_tracked;
    std::unordered_map<BodyJointType, BodyJoint> joints;
};

[[maybe_unused]] static Joint4x64 joint_to_int(const BodyInfos &b, const BodyJointType type,  const BodyJoint &j){
    Joint4x64 ji;
    ji.bodyId               = b.id;
    ji.bodyTracked          = b.tracked;
    ji.jointid              = static_cast<std::uint64_t>(type);
    ji.jointState           = static_cast<std::uint64_t>(j.state);
    ji.leftHandState        = static_cast<std::uint64_t>(b.leftHandState);
    ji.rightHandState       = static_cast<std::uint64_t>(b.rightHandState);
    ji.leftHandConfidence   = b.leftHandHightConfidence ? 1 : 0;
    ji.rightHandConfidence  = b.rightHandHightConfidence ? 1 : 0;

    std_v1<float> p = {j.pos.x(),j.pos.y(),j.pos.z()};
    std_v1<std::uint32_t> pi = {0,0,0};
    std::copy(reinterpret_cast<std::uint32_t*>(p.data()), reinterpret_cast<std::uint32_t*>(p.data())+3, pi.data());
    ji.jointPosX = pi[0];
    ji.jointPosY = pi[1];
    ji.jointPosZ = pi[2];

    std_v1<float> q = {j.rotQuaternion.x(),j.rotQuaternion.y(),j.rotQuaternion.z(), j.rotQuaternion.w()};
    std_v1<std::uint32_t> qi = {0,0,0,0};
    std::copy(reinterpret_cast<std::uint32_t*>(q.data()), reinterpret_cast<std::uint32_t*>(q.data())+4, qi.data());
    ji.jointQuaX = qi[0];
    ji.jointQuaY = qi[1];
    ji.jointQuaZ = qi[2];
    ji.jointQuaW = qi[3];

    return ji;
}

//  body id / body tracked / body restricted
[[maybe_unused]] static std::tuple<std::uint64_t,  bool, bool, BodyJointType, BodyJoint> int_to_joint(const Joint4x64 &ji){

    BodyJoint j;
    j.state       = static_cast<TrackingStateT>(ji.jointState);

    std_v1<std::uint32_t> pi = {
        static_cast<std::uint32_t>(ji.jointPosX),
        static_cast<std::uint32_t>(ji.jointPosY),
        static_cast<std::uint32_t>(ji.jointPosZ)
    };
    std::copy(std::begin(pi), std::end(pi),  reinterpret_cast<std::uint32_t*>(j.pos.v.data()));

    std_v1<std::uint32_t> qi = {
        static_cast<std::uint32_t>(ji.jointQuaX),
        static_cast<std::uint32_t>(ji.jointQuaY),
        static_cast<std::uint32_t>(ji.jointQuaZ),
        static_cast<std::uint32_t>(ji.jointQuaW)
    };
    std::copy(std::begin(qi), std::end(qi),  reinterpret_cast<std::uint32_t*>(j.rotQuaternion.v.data()));

    return {ji.bodyId, ji.bodyTracked, ji.bodyRestricted, static_cast<BodyJointType>(ji.jointid), j};
}




// display data

struct ImageDisplayData{

    ImageDisplayData(){
        // TODO
    }

    std_v1<unsigned char> colors;
    size_t sizeColors;
    std::mutex dataLocker;
};

struct CloudDisplayData{

    CloudDisplayData(){
        points  = std::make_unique<CloudData>();
        normals = std::make_unique<CloudData>();
        colors  = std::make_unique<CloudData>();
    }

    CloudDataUP points = nullptr;
    CloudDataUP colors = nullptr;
    CloudDataUP normals = nullptr;

    size_t sizePts;
    std::mutex dataLocker;
};

struct MeshDisplayData{

    MeshDisplayData(){
        points      = std::make_unique<CloudData>();
        normals     = std::make_unique<CloudData>();
        colors      = std::make_unique<CloudData>();
        triangles   = std::make_unique<TrianglesData>();
    }

    CloudDataUP points = nullptr;
    CloudDataUP colors = nullptr;
    CloudDataUP normals = nullptr;
    TrianglesDataUP triangles = nullptr;

    size_t sizePts;
    size_t sizeTris;
    std::mutex dataLocker;
};

struct BodiesDisplayData{
    std_a1<BodyInfos,6> bodies;
    std::mutex dataLocker;
};


// frame data
struct FrameData{

    // infos
//    std::uint32_t currentFrame;
//    std_v1<float> intrinsics;
    // timestamp
    // times
    // frame id
    // sizes

    FrameData(){
        depth = std::make_unique<DepthData>();
        infra = std::make_unique<InfraData>();
    }

    DepthDataUP depth   = nullptr;
    InfraDataUP infra   = nullptr;

    std_v1<unsigned char> compressedColor;
    std_v1<unsigned char> compressedCloud;
};
using FrameDataUP = std::unique_ptr<FrameData>;


struct SavedData{

    std::int64_t timeStamp = 0;

    size_t frameId = 0;

    float focalLengthX=0.f;
    float focalLengthY=0.f;
    float principalPointX=0.f;
    float principalPointY=0.f;

    size_t sizeDepths=0;
    size_t sizeColors=0;
    std_v1<std::uint32_t> depths;
    std_v1<unsigned char> colors;

    std_a1<BodyInfos,6> bodies;
};


struct Settings{

    // # width / height
    unsigned int minWidth = 0;
    unsigned int maxWidth = depth_width;
    unsigned int minHeight = 0;
    unsigned int maxHeight =depth_height;

    // infra
    unsigned short minInfra = 0;
    unsigned short maxInfra = 10000;
    unsigned short minInfraRange = 0;
    unsigned short maxInfraRange = 255;

    // # color
    float yFactor = 1.f;
    float uFactor = 1.f;
    float vFactor = 1.f;

    // # depth
    float minDepthValue = 0.3f;
    float maxDepthValue = 8.f;
    float maxLocalDiff  = 0.1f;
    float offsetAfterMin = -1.f;
    float vmin = 0.f;
    float vmax = 0.5f;
    float gmin = 0.1f;

    // compression
    unsigned char jpegCompressionRate = 80;

    // smoothing
    unsigned char smoothingKernelSize;
    unsigned char smoothingMethod;

    // # neigbhours
    unsigned char nbMinNeighboursNb = 1;
    unsigned char minNeighboursLoops = 1;

    // erosion
    unsigned char erosionSize = 2;
    MorphShapes erosionType = MORPH_ELLIPSE; // opencv
    unsigned char minErosionValue = 255;

    // fps
    unsigned char fps = 30;

    // filtered color
    geo::Pt3<std::uint8_t> filterColor  = geo::Pt3<std::uint8_t>(255,0,0);
    geo::Pt3<std::uint8_t> maxDiffColor = geo::Pt3<std::uint8_t>(10,40,40);

    // flogs
    bool filterDepthWithColor = false;
    bool infraInvert = false;
    bool infraBinary = false;
    bool smoothingEnabled = false;
    bool doErosion = false;
    bool doTemporalFilter = false;

    std::string to_string() const;
    static Settings from_string(const std::string &parametersStr);
};


struct Frame{

    Frame(bool initData);

    Frame copy_frame();

    FrameRequest mode;
    Settings parameters;
    std_v1<float> intrinsics;

    // output formated kinect data
    // # infos
    std::int64_t timeStampGetFrame = 0;
    std::int64_t timeStampEndProcessing = 0;
    std::int64_t relativeTimeColor = 0;
    std::int64_t relativeTimeDepth = 0;
    std::int64_t relativeTimeInfra = 0;
    std::int64_t relativeTimeLongInfra = 0;
    std::int64_t relativeTimeBody = 0;
    std::uint16_t minReliableDistance = 0; // 500 constant?
    std::uint16_t maxReliableDistance = 0; // 4500
    std::uint32_t frameId = 0;
    long unsigned int jpegColorSize = 0;
    size_t compressedDepthSize = 0;
    // # color
    std_v1<unsigned char> compressedImage;
    // # depth
    std_v1<std::uint16_t> compressedDepthData;
    // # infra
    InfraDataUP infraData  = nullptr;  // (V)
    // # depth
    DepthDataUP depthData = nullptr; // (V)
    // # bodies
    std_a1<BodyInfos,6> bodiesData;    // 6 bodies, 25 joints

};
using FrameUP = std::unique_ptr<Frame>;


}
