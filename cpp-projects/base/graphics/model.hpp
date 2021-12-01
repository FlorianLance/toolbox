

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
#include <optional>

// base
#include "utility/vector.hpp"
#include "utility/string.hpp"
#include "geometry/matrix4.hpp"
#include "geometry/quaternion.hpp"

// local
#include "geometry/mesh.hpp"
#include "texture.hpp"
#include "material.hpp"

#include "assimp/scene.h"
#include "assimp/matrix4x4.h"

namespace tool::graphics {

template <typename acc>
struct GMesh{
    std::string name;
    Material *material = nullptr;
    geo::Mesh<acc> mesh;
};

struct BoneInfo{
//    geo::Mat4f offset = geo::Mat4f(true);
    aiMatrix4x4t<float> offset;
    geo::Mat4f final = geo::Mat4f(true);
};

struct AnimationKeys{

    std_v1<double> positionTimes;
    std_v1<geo::Pt3f> positionKeys;

    std_v1<double> rotationTimes;
    std_v1<geo::Quatf> rotationKeys;

    std_v1<double> scalingTimes;
    std_v1<geo::Pt3f> scalingKeys;
};

struct Animation{
    std::string name;
    double duration;
    double ticksPerSecond;
};



struct BonesHierarchy{
    std::string boneName;
//    geo::Mat4f tr;
    aiMatrix4x4t<float> tr;
    std_v1<BonesHierarchy> children;
};


template <typename acc>
class Model{

public:

    // info
    std::string name;
    std::string directory = "";

    // meshes
    std_v1<std::shared_ptr<GMesh<acc>>> gmeshes;

    // materials
    std_v1<Material> m_materials;

    // textures
    std::unordered_map<std::string, Texture2D> m_textures;

    // animations
    std_v1<Animation> animations;
    std::unordered_map<std::string, std::unordered_map<std::string,AnimationKeys>> animationsKeys;

    // bones
    std_v1<BoneInfo> bonesInfo;
    std::map<std::string, unsigned int> bonesMapping;
    BonesHierarchy bonesHierachy;

    aiMatrix4x4t<acc> globalInverseTr;
    geo::Mat4<acc> globalInverseTr2;

    // transform
    geo::Mat4<acc> transform;    

    // hierarchy
    Model *parent = nullptr;
    std_v1<std::shared_ptr<Model>> children;

public:

    Model(){
    }

    Model(Model *parentModel) : parent(parentModel){
    }

    bool is_root()const{
        return parent == nullptr;
    }


    std_v1<Model*> get_all_tree_models(){

        size_t size = 0;
        for(const auto &child : children){
            size += child->children.size();
        }

        std_v1<Model*> models;
        models.reserve(size+1);
        models.emplace_back(this);

        for(const auto &child : children){
            auto childModels = child->get_all_tree_models();
            models.insert(std::end(models), std::begin(childModels), std::end(childModels));
        }

        return models;
    }

    void get_models_with_no_children(std_v1<Model*> &models) {

        if(children.size() == 0){
            models.emplace_back(this);
            return;
        }

        for(auto &child : children){
            child->get_models_with_no_children(models);
        }
    }

    void clean(){

        std_v1<size_t> childrenToRemove;
        for(size_t ii = 0; ii < children.size(); ++ii){
            if(children[ii]->gmeshes.size() == 0 && children[ii]->children.size() == 0 && children[ii]->animations.size() == 0){
                childrenToRemove.emplace_back(ii);
            }
        }

        std::reverse(std::begin(childrenToRemove), std::end(childrenToRemove));
        for(const auto &id : childrenToRemove){
            children.erase(std::begin(children) + static_cast<int>(id));
        }

        if(gmeshes.size() == 0 && children.size() == 0 && animations.size() == 0){
            parent->clean();
        }
    }

    void display_hierarchy() const{

        if(!parent){
            std::cout << "### Root node: " << name << " with " <<  gmeshes.size() << " meshes and " << animations.size() << " animations and " << children.size() << " sub nodes. \n";
        }else{
            std::cout <<"Node: " << name << " with " <<  gmeshes.size() << " meshes and " << animations.size() << " animations and " << children.size() << " sub nodes. \n";
        }

        std::cout << "Meshes:\n";
        for(const auto gmesh : gmeshes){
            std::cout << "-> " << gmesh->name << "\n";
            const geo::Mesh<acc> *mesh = &gmesh->mesh;
            std::cout << "  # " << mesh->vertices.size() << " " << mesh->normals.size() << " " << mesh->triIds.size() << " " << mesh->tangents.size() << " " << mesh->bones.size() << "\n";
        }

        std::cout << "Animations:\n";
        for(const auto &animation : animations){
            std::cout << "->" << animation.name << " " << animation.duration << " " << animation.ticksPerSecond << "\n";
        }

        for(const auto &child : children){
            child->display_hierarchy();
        }
    }

    size_t find_position(AnimationKeys *animation, float time) const{
        for(size_t ii = 0; ii < animation->positionTimes.size()-1; ++ii){
            if(time < static_cast<float>(animation->positionTimes[ii+1])){
                return ii;
            }
        }
        return 0;
    }

    size_t find_rotation(AnimationKeys *animation, float time) const{
        for(size_t ii = 0; ii < animation->rotationTimes.size()-1; ++ii){
            if(time < static_cast<float>(animation->rotationTimes[ii+1])){
                return ii;
            }
        }
        return 0;
    }

    size_t find_scaling(AnimationKeys *animation, float time) const{
        for(size_t ii = 0; ii < animation->scalingTimes.size()-1; ++ii){
            if(time < static_cast<float>(animation->scalingTimes[ii+1])){
                return ii;
            }
        }
        return 0;
    }

    geo::Pt3f calculate_interpolated_scaling(AnimationKeys *animation, float time){

        if(animation->scalingKeys.size() == 1){
            return animation->scalingKeys[0];
        }

        size_t scalingId = find_scaling(animation, time);
        size_t nextScalingIndex = scalingId +1;

        float deltaTime = static_cast<float>(animation->scalingTimes[nextScalingIndex] - animation->scalingTimes[scalingId]);
        float factor = (time - static_cast<float>(animation->scalingTimes[scalingId]))/deltaTime;

        geo::Pt3f start = animation->scalingKeys[scalingId];
        geo::Pt3f end   = animation->scalingKeys[nextScalingIndex];
        geo::Pt3f delta = end - start;

        return start + delta * factor;
    }

    geo::Pt3f calculate_interpolated_position(AnimationKeys *animation, float time){

        if(animation->positionKeys.size() == 1){
            return animation->positionKeys[0];
        }

        size_t positionId = find_position(animation, time);
        size_t nextPositionIndex = positionId +1;

        float deltaTime = static_cast<float>(animation->positionTimes[nextPositionIndex] - animation->positionTimes[positionId]);
        float factor = (time - static_cast<float>(animation->positionTimes[positionId]))/deltaTime;

        geo::Pt3f start = animation->positionKeys[positionId];
        geo::Pt3f end   = animation->positionKeys[nextPositionIndex];
        geo::Pt3f delta = end - start;

        return start + delta * factor;
    }

    geo::Quatf calculate_interpolated_rotation(AnimationKeys *animation, float time){

        if(animation->rotationKeys.size() == 1){
            return animation->rotationKeys[0];
        }

        size_t rotationId = find_rotation(animation, time);
        size_t nextRotationIndex = rotationId +1;

        float deltaTime = static_cast<float>(animation->rotationTimes[nextRotationIndex] - animation->rotationTimes[rotationId]);
        float factor = (time - static_cast<float>(animation->rotationTimes[rotationId]))/deltaTime;

        geo::Quatf start = animation->rotationKeys[rotationId];
        geo::Quatf end   = animation->rotationKeys[nextRotationIndex];

        geo::Quatf delta = normalize(slerp(start, end, factor));

        aiQuaternion StartRotationQ;
        StartRotationQ.x = start.x;
        StartRotationQ.y = start.y;
        StartRotationQ.z = start.z;
        StartRotationQ.w = start.w;

        aiQuaternion EndRotationQ;
        EndRotationQ.x = end.x;
        EndRotationQ.y = end.y;
        EndRotationQ.z = end.z;
        EndRotationQ.w = end.w;

        aiQuaternion Out;
        aiQuaternion::Interpolate(Out, StartRotationQ, EndRotationQ, factor);
        Out = Out.Normalize();

        return {Out.x,Out.y,Out.z,Out.w};


        return delta;
    }



    void get_bones_transform(BonesHierarchy *bh, const std::string &animationName, float time, const aiMatrix4x4t<acc> &parentTr){

        auto nodeTr = bh->tr;

        if(animationsKeys.count(animationName) != 0){

            if(animationsKeys[animationName].count(bh->boneName) != 0){

                AnimationKeys *keys = &animationsKeys[animationName][bh->boneName];

                // compute interpolation
                geo::Pt3f scaling   = calculate_interpolated_scaling(keys, time);
                geo::Quatf rotation = calculate_interpolated_rotation(keys, time);
                geo::Pt3f position  = calculate_interpolated_position(keys, time);

//                auto a  = rotation.to_euler_angles();
//                auto ea = geo::Pt3f{rad_2_deg(a.x()), rad_2_deg(a.y()), rad_2_deg(a.z())};
//                nodeTr  = geo::Mat4f::transform(scaling, geo::Pt3f{}, geo::Pt3f{});
//                nodeTr = geo::Mat4f(true);
//                std::cout << nodeTr << "\n";
//                std::cout << "------tr: " << scaling << " " << ea << " " << position << "\n";
//                nodeTr = geo::Mat4<float>::translation_matrix(position)*geo::Mat4<float>::rotation_matrix(ea)*geo::Mat4<float>::scale_matrix(scaling);


                aiVector3D Scaling;
                Scaling.x = scaling.x();
                Scaling.y = scaling.y();
                Scaling.z = scaling.z();

                aiQuaternion Rotation;
                Rotation.x = rotation.x;
                Rotation.y = rotation.y;
                Rotation.z = rotation.z;
                Rotation.w = rotation.w;

                aiVector3D Position;
                Position.x = position.x();
                Position.y = position.y();
                Position.z = position.z();

                nodeTr = aiMatrix4x4t(Scaling, Rotation, Position);


//                aiVector3t<float> aiTr,aiRot,aiSc;
//                q.Decompose(aiSc,aiRot,aiTr);
//                auto r = geo::Pt3f{aiRot.x,aiRot.y,aiRot.z};
//                nodeTr = geo::Mat4f::transform(
//                    geo::Pt3f{aiSc.x,aiSc.y,aiSc.z},
////                    geo::Pt3f{rad_2_deg(r.x()),rad_2_deg(r.y()),rad_2_deg(r.z())},
//                    geo::Pt3f{(r.x()),(r.y()),(r.z())},
//                    geo::Pt3f{aiTr.x,aiTr.y,aiTr.z}
//                );


//                const auto &m = q;//.Transpose();
//                nodeTr = geo::Mat4f
//                {
//                    m.a1,m.a2,m.a3,m.a4,
//                    m.b1,m.b2,m.b3,m.b4,
//                    m.c1,m.c2,m.c3,m.c4,
//                    m.d1,m.d2,m.d3,m.d4,
//                };


            }
        }

       auto globalTr = parentTr * nodeTr;
//        geo::Mat4f globalTr = nodeTr*parentTr;
        if(animationsKeys.count(animationName) != 0){

            std::string bName = bh->boneName;
//            std::cout << "----------> " << bName << " ";
//            if(bonesMapping.count(bName) != 0){


//            }
//            auto pos = std::find(std::begin(bonesMapping));
//            if (pos != std::string::npos) {
//                bName.erase(pos, bName.size()-pos);
//            }

            if(bonesMapping.count(bName) != 0){
//                std::cout << "find bone " << bName << "\n";

                auto id = bonesMapping[bName];
                auto m = globalInverseTr* globalTr * bonesInfo[id].offset;

//                bonesInfo[id].final =
//                {
//                    m.a1,m.a2,m.a3,m.a4,
//                    m.b1,m.b2,m.b3,m.b4,
//                    m.c1,m.c2,m.c3,m.c4,
//                    m.d1,m.d2,m.d3,m.d4,
//                };
                bonesInfo[id].final =
                {
                    m.a1,m.b1,m.c1,m.d1,
                    m.a2,m.b2,m.c2,m.d2,
                    m.a3,m.b3,m.c3,m.d3,
                    m.a4,m.b4,m.c4,m.d4,
                };

//                std::cout << "final " << bonesInfo[id].final << "\n";

//                bonesInfo[id].final = globalInverseTr.value()* globalTr * bonesInfo[id].offset;
//                bonesInfo[id].final = globalInverseTr.value();//* bonesInfo[id].offset;

//                bonesInfo[id].final =  bonesInfo[id].offset* globalTr* globalInverseTr.value();
//                std::cout << "final: " << id << " " << bonesInfo[id].final << "\n";
            }else{
//                std::cerr << "Cannot find bone " << bName << " in mapping: " << bonesMapping.size() << "\n";
            }
        }

        for(auto &b : bh->children){
            get_bones_transform(&b, animationName, time, globalTr);
        }
    }


    void update_animation(const std::string &animationName, float time){

        const Animation *animationP = nullptr;


        for(const auto &animation : animations){
            if(animation.name == animationName){
                animationP = &animation;
                break;
            }
        }
        if(animationP == nullptr){
            return;
        }


        float TicksPerSecond = almost_equal(animationP->ticksPerSecond, 0.0) ? static_cast<float>(animationP->ticksPerSecond) : 25.f;
        float TimeInTicks = time * TicksPerSecond;
        float AnimationTime = fmod(TimeInTicks, static_cast<float>(animationP->duration));
        //        std::cout << "TicksPerSecond: " << TicksPerSecond << " " << animations.size() <<  "\n";
        //        std::cout << "TimeInTicks: " << TimeInTicks << "\n";
        //        std::cout << "animations[animationName].duration: " << animationP->duration << "\n";
//        std::cout << "AnimationTime: " << AnimationTime << "\n";

        // retrieve bones transforms
        aiMatrix4x4t<float> m;

//        std::cout << "update_animation " <<  animationName << " " << AnimationTime << "\n";
//        std::cout << "start " <<  AnimationTime <<"\n";
        get_bones_transform(&bonesHierachy, animationName, AnimationTime, m);

    }
};
}

