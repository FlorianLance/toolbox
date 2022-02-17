
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

namespace tool::camera{

using namespace std::literals::string_view_literals;

enum class K2Detection : std::uint8_t{
    unknow= 0,
    no,
    maybe,
    yes,
};

enum class K2HandStateT : std::uint8_t{
    unknow= 0,
    not_tracked,
    open,
    closed,
    lasso
};

enum class K2TrackingStateT : std::uint8_t{
    not_tracked=0, // body tracked
    inferred,
    tracked
};

enum class K2BodyJointType : std::uint8_t{
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
using K2JointName = SV;
using K2TJoint = std::tuple<
    K2BodyJointType,                          K2JointName>;
[[maybe_unused]] static constexpr TupleArray<K2BodyJointType::SizeEnum, K2TJoint> k2JointsM{{
    K2TJoint{K2BodyJointType::spine_base,       "spine_base"sv},
    K2TJoint{K2BodyJointType::spine_mid,        "spine_mid"sv},
    K2TJoint{K2BodyJointType::neck,             "neck"sv},
    K2TJoint{K2BodyJointType::head,             "head"sv},
    K2TJoint{K2BodyJointType::shoulder_left,    "shoulder_left"sv},
    K2TJoint{K2BodyJointType::elbow_left,       "elbow_left"sv},
    K2TJoint{K2BodyJointType::wrist_left,       "wrist_left"sv},
    K2TJoint{K2BodyJointType::hand_left,        "hand_left"sv},
    K2TJoint{K2BodyJointType::shoulder_right,   "shoulder_right"sv},
    K2TJoint{K2BodyJointType::elbow_right,      "elbow_right"sv},
    K2TJoint{K2BodyJointType::wrist_right,      "wrist_right"sv},
    K2TJoint{K2BodyJointType::hand_right,       "hand_right"sv},
    K2TJoint{K2BodyJointType::hip_left,         "hip_left"sv},
    K2TJoint{K2BodyJointType::knee_left,        "knee_left"sv},
    K2TJoint{K2BodyJointType::ankle_left,       "ankle_left"sv},
    K2TJoint{K2BodyJointType::foot_left,        "foot_left"sv},
    K2TJoint{K2BodyJointType::hip_right,        "hip_right"sv},
    K2TJoint{K2BodyJointType::knee_right,       "knee_right"sv},
    K2TJoint{K2BodyJointType::ankle_right,      "ankle_right"sv},
    K2TJoint{K2BodyJointType::foot_right,       "foot_right"sv},
    K2TJoint{K2BodyJointType::spine_shoulder,   "spine_shoulder"sv},
    K2TJoint{K2BodyJointType::hand_tip_left,    "hand_tip_left"sv},
    K2TJoint{K2BodyJointType::thumb_left,       "thumb_left"sv},
    K2TJoint{K2BodyJointType::hand_tip_right,   "hand_tip_right"sv},
    K2TJoint{K2BodyJointType::thumb_right,      "thumb_right"sv},
}};


enum class K2FrameRequest : uint8_t {
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

enum K2MorphShapes : unsigned char{
    MORPH_RECT    = 0,
    MORPH_CROSS   = 1,
    MORPH_ELLIPSE = 2
};

// constants / aliases
// # color (Pt3 uint8 / Pt4 uint8)
constexpr std::uint32_t k2_color_width          = 1920;
constexpr std::uint32_t k2_color_height         = 1080;
constexpr std::uint32_t k2_color_count          = k2_color_width * k2_color_height;
[[maybe_unused]] constexpr std::uint32_t k2_color_data_size      = k2_color_count * 4;
[[maybe_unused]] constexpr std::uint32_t k2_raw_color_data_size  = k2_color_count * 2;
using K2RgbData    = std_a1<geo::Pt3<std::uint8_t>,k2_color_count>;
using K2RgbDataUP  = std::unique_ptr<K2RgbData>;
using K2RgbaData   = std_a1<geo::Pt4<std::uint8_t>,k2_color_count>;
using K2RgbaDataUP = std::unique_ptr<K2RgbaData>;

// #  infrared
constexpr std::uint32_t k2_infrared_width      = 512;
constexpr std::uint32_t k2_infrared_height     = 424;
constexpr std::uint32_t k2_infrared_count      = k2_infrared_width * k2_infrared_height;
[[maybe_unused]] constexpr std::uint32_t k2_infrared_data_size  = k2_infrared_count * 2;
using K2InfraData      = std_a1<std::uint16_t,k2_infrared_count>;
using K2InfraDataUP    = std::unique_ptr<K2InfraData>;

// #  depth (Int16)
constexpr std::uint32_t k2_depth_width      = 512;
constexpr std::uint32_t k2_depth_height     = 424;
constexpr std::uint32_t k2_depth_count      = k2_depth_width * k2_depth_height;
[[maybe_unused]] constexpr std::uint32_t k2_depth_data_size  = k2_depth_count * 2;
using K2DepthData   = std_a1<uint16_t,(k2_depth_count)>;
using K2DepthDataUP = std::unique_ptr<K2DepthData>;

// #  cloud (Pt3f)
using K2CloudData   = std_a1<geo::Pt3f,k2_depth_count>;
using K2CloudDataUP = std::unique_ptr<K2CloudData>;

// #  colored cloud (Voxel)
using K2ColoredCloudData   = std_a1<geo::Voxel,k2_depth_count>;
using K2ColoredCloudDataUP = std::unique_ptr<K2ColoredCloudData>;

// #  triangles id
constexpr std::uint32_t k2_triangles_count  = (k2_depth_width-1) * (k2_depth_height-1) * 2;
[[maybe_unused]] constexpr std::uint32_t k2_triangles_id_count = k2_triangles_count * 3;

using K2TrianglesData   = std_a1<geo::Pt3<std::uint32_t>, k2_triangles_count>;
using K2TrianglesDataUP = std::unique_ptr<K2TrianglesData>;

// # bodies joint
constexpr std::uint32_t k2_body_joint_data_size64    = 4;
constexpr std::uint32_t k2_body_joints_count         = 25;
constexpr std::uint32_t k2_bodies_count              = 6;
constexpr std::uint32_t k2_bodies_joints_count       = k2_bodies_count * k2_body_joints_count;
constexpr std::uint32_t k2_bodies_joints_data_size64 = k2_bodies_joints_count * k2_body_joint_data_size64;
[[maybe_unused]] constexpr std::uint32_t k2_bodies_joints_data_size8  = k2_bodies_joints_data_size64 * 8;

constexpr std::uint32_t k2_voxels_data_size64           = k2_depth_count;
constexpr std::uint32_t k2_triangles_data_size64        = k2_triangles_count;

constexpr float k2_invalid_value = -10.f;
[[maybe_unused]] constexpr geo::Pt3f k2_invalid_point(k2_invalid_value,k2_invalid_value,k2_invalid_value);

// mode mapping
// # for each component category : name
using K2Requirment  = bool;
using K2ColorR      = K2Requirment;
using K2DepthR      = K2Requirment;
using K2InfraR      = K2Requirment;
using K2LongInfraR  = K2Requirment;
using K2BodyR       = K2Requirment;
using K2WidthFrame  = int;
using K2HeightFrame = int;
using K2FrameName   = SV;
using K2FR        = K2FrameRequest;

using K2TFrame = std::tuple<
           K2FR,                              K2ColorR, K2DepthR, K2InfraR, K2LongInfraR, K2BodyR, K2WidthFrame,       K2HeightFrame,     K2FrameName>;
static constexpr TupleArray<K2FR::SizeEnum, K2TFrame> frames= {{
    K2TFrame{K2FR::compressed_color_cloud,      true,   true,   false,  false,      true,  k2_depth_width,      k2_depth_height,    "compressed_color_cloud"sv},
    K2TFrame{K2FR::compressed_color_mesh,       true,   true,   false,  false,      true,  k2_depth_width,      k2_depth_height,    "compressed_color_mesh"sv},
    K2TFrame{K2FR::compressed_color_512x424,    true,   true,   false,  false,      false, k2_depth_width,      k2_depth_height,    "compressed_image_color_512x424"sv},
    K2TFrame{K2FR::compressed_color_1920x1080,  true,   false,  false,  false,      false, k2_color_width,      k2_color_height,    "compressed_image_color_1920x1080"sv},
    K2TFrame{K2FR::depth_512x424,               false,  true,   false,  false,      false, k2_depth_width,      k2_depth_height,    "image_depth_512x424"sv},
    K2TFrame{K2FR::infra_512x424,               false,  false,  true,   false,      false, k2_infrared_width,   k2_infrared_height, "image_infra_512x424"sv},
    K2TFrame{K2FR::long_exposure_infra_512x424, false,  false,  false,  true,       false, k2_infrared_width,   k2_infrared_height, "image_long_exposure_infra_512x424"sv},
    K2TFrame{K2FR::undefined,                   false,  false,  false,  false,      false, 0,                0,               "undefined"sv},
}};

[[maybe_unused]] static bool color_channel_required(K2FrameRequest r) {
    return frames.at<0,1>(r);
}
[[maybe_unused]] static bool depth_channel_required(K2FrameRequest r) {
    return frames.at<0,2>(r);
}
[[maybe_unused]] static bool infra_channel_required(K2FrameRequest r) {
    return frames.at<0,3>(r);
}
[[maybe_unused]] static bool long_infra_channel_required(K2FrameRequest r) {
    return frames.at<0,4>(r);
}
[[maybe_unused]] static bool body_channel_required(K2FrameRequest r) {
    return frames.at<0,5>(r);
}
[[maybe_unused]] static auto all_requests_names(){
    return frames.elements_not_matching_columns_values<0,8>(K2FR::undefined);
}

[[maybe_unused]] static constexpr K2FrameRequest index_to_mode(size_t id){
    if(id < static_cast<uint8_t>(K2FR::undefined) && id >= 0){
        return static_cast<K2FR>(id);
    }
    return K2FR::undefined;
}

[[maybe_unused]] static constexpr uint8_t mode_to_index(K2FrameRequest r){
    return static_cast<uint8_t>(r);
}

using K2CloudInt64Data      = std_a1<std::uint64_t, k2_voxels_data_size64 + k2_bodies_joints_data_size64>;
using K2CloudInt64DataUP    = std::unique_ptr<K2CloudInt64Data>;

using K2MeshInt64Data       = std_a1<std::uint64_t, k2_voxels_data_size64 + k2_triangles_data_size64 + k2_bodies_joints_data_size64>;
using K2MeshInt64DataUP     = std::unique_ptr<K2MeshInt64Data>;

// network types
struct K2Joint4x64{
    std::uint64_t jointPosX : 32, jointPosY : 32;
    std::uint64_t jointPosZ : 32, jointQuaX : 32;
    std::uint64_t jointQuaY : 32, jointQuaZ : 32;
    std::uint64_t jointQuaW : 32, jointState : 2, jointid : 5,
                  bodyId : 17, bodyTracked : 1, bodyRestricted : 1,
                  leftHandState : 2, rightHandState : 2, leftHandConfidence : 1, rightHandConfidence : 1;
};

struct K2BodyJoint{
    geo::Pt3f pos;
    geo::Pt4f rotQuaternion;
    K2TrackingStateT state  = K2TrackingStateT::not_tracked;
};

struct K2BodyInfos{
    std::uint64_t id = 0;
    bool tracked = false;
    bool restricted = false;
    K2Detection engaged        = K2Detection::unknow;
    K2HandStateT leftHandState  = K2HandStateT::unknow;
    K2HandStateT rightHandState = K2HandStateT::unknow;
    bool leftHandHightConfidence    = false;
    bool rightHandHightConfidence   = false;
    geo::Pt2f lean;
    K2TrackingStateT leanTracking = K2TrackingStateT::not_tracked;
    std::unordered_map<K2BodyJointType, K2BodyJoint> joints;
};

[[maybe_unused]] static K2Joint4x64 joint_to_int(const K2BodyInfos &b, const K2BodyJointType type,  const K2BodyJoint &j){
    K2Joint4x64 ji;
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
[[maybe_unused]] static std::tuple<std::uint64_t,  bool, bool, K2BodyJointType, K2BodyJoint> int_to_joint(const K2Joint4x64 &ji){

    K2BodyJoint j;
    j.state       = static_cast<K2TrackingStateT>(ji.jointState);

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

    return {ji.bodyId, ji.bodyTracked, ji.bodyRestricted, static_cast<K2BodyJointType>(ji.jointid), j};
}


// display data
struct K2ImageDisplayData{

    K2ImageDisplayData(){
        // TODO
    }

    std_v1<unsigned char> colors;
    size_t sizeColors;
    std::mutex dataLocker;
};

struct K2CloudDisplayData{

    K2CloudDisplayData(){
        points  = std::make_unique<K2CloudData>();
        normals = std::make_unique<K2CloudData>();
        colors  = std::make_unique<K2CloudData>();
    }

    K2CloudDataUP points = nullptr;
    K2CloudDataUP colors = nullptr;
    K2CloudDataUP normals = nullptr;

    size_t sizePts;
    std::mutex dataLocker;
};

struct K2MeshDisplayData{

    K2MeshDisplayData(){
        points      = std::make_unique<K2CloudData>();
        normals     = std::make_unique<K2CloudData>();
        colors      = std::make_unique<K2CloudData>();
        triangles   = std::make_unique<K2TrianglesData>();
    }

    K2CloudDataUP points = nullptr;
    K2CloudDataUP colors = nullptr;
    K2CloudDataUP normals = nullptr;
    K2TrianglesDataUP triangles = nullptr;

    size_t sizePts;
    size_t sizeTris;
    std::mutex dataLocker;
};

struct K2BodiesDisplayData{
    std_a1<K2BodyInfos,6> bodies;
    std::mutex dataLocker;
};


// frame data
struct K2FrameData{

    // infos
//    std::uint32_t currentFrame;
//    std_v1<float> intrinsics;
    // timestamp
    // times
    // frame id
    // sizes

    K2FrameData(){
        depth = std::make_unique<K2DepthData>();
        infra = std::make_unique<K2InfraData>();
    }

    K2DepthDataUP depth   = nullptr;
    K2InfraDataUP infra   = nullptr;

    std_v1<unsigned char> compressedColor;
    std_v1<unsigned char> compressedCloud;
};
using K2FrameDataUP = std::unique_ptr<K2FrameData>;


struct K2SavedData{

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

    std_a1<K2BodyInfos,6> bodies;
};


struct K2Settings{

    // # width / height
    unsigned int minWidth = 0;
    unsigned int maxWidth = k2_depth_width;
    unsigned int minHeight = 0;
    unsigned int maxHeight =k2_depth_height;

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
    K2MorphShapes erosionType = MORPH_ELLIPSE; // opencv
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
    static K2Settings from_string(const std::string &parametersStr);
};


struct K2Frame{

    K2Frame(bool initData);

    K2Frame copy_frame();

    K2FrameRequest mode;
    K2Settings parameters;
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
    K2InfraDataUP infraData  = nullptr;  // (V)
    // # depth
    K2DepthDataUP depthData = nullptr; // (V)
    // # bodies
    std_a1<K2BodyInfos,6> bodiesData;    // 6 bodies, 25 joints

};
using K2FrameUP = std::unique_ptr<K2Frame>;


}
