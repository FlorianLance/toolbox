
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


//// std
//#include <optional>
//#include <array>
//#include <iostream>

//// local
//#include "geometry/aabb3.hpp"
//#include "utility/vector_utility.hpp"

//namespace tool::geo {

//struct Color4{
////    std::array<float,4> rgba;
//    float r,g,b,a;
//};

//template<typename acc>
//struct Voxel{
//    Point3<acc> p;
//    Color4 c;
//};

////     4 ----- 5
////   / |    /  |
//// 7 ----- 6   |
//// |   0 --|-- 1
//// | /     | /
//// 3 ----- 2

//// +Y  +Z
////  | /
////  0 --- +X



//template<typename acc>
//constexpr std::array<Point3<acc>, 8> mu = {
////    Pt3<acc>{+1,+1,-1}, // 0
////    Pt3<acc>{-1,+1,-1}, // 1
////    Pt3<acc>{-1,+1,+1}, // 2
////    Pt3<acc>{+1,+1,+1}, // 3
////    Pt3<acc>{+1,-1,-1}, // 4
////    Pt3<acc>{-1,-1,-1}, // 5
////    Pt3<acc>{-1,-1,+1}, // 6
////    Pt3<acc>{+1,-1,+1}, // 7
//    Pt3<acc>{-1,-1,+1}, // 0
//    Pt3<acc>{+1,-1,+1}, // 1
//    Pt3<acc>{+1,-1,-1}, // 2
//    Pt3<acc>{-1,-1,-1}, // 3
//    Pt3<acc>{-1,+1,+1}, // 4
//    Pt3<acc>{+1,+1,+1}, // 5
//    Pt3<acc>{+1,+1,-1}, // 6
//    Pt3<acc>{-1,+1,-1}, // 7

//};

//constexpr std::array<Point3<size_t>, 8> mu3 = {
//    Pt3<size_t>{0,0,1}, // 0
//    Pt3<size_t>{1,0,1}, // 1
//    Pt3<size_t>{1,0,0}, // 2
//    Pt3<size_t>{0,0,0}, // 3
//    Pt3<size_t>{0,1,1}, // 4
//    Pt3<size_t>{1,1,1}, // 5
//    Pt3<size_t>{1,1,0}, // 6
//    Pt3<size_t>{0,1,0}  // 7
//};


////constexpr std::array<std::array<Pt3<int>,8>,8> mu4{{

//////    {Pt3<int>{-1,-1,+0}, Pt3<int>{+0,-1,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{-1,+0,+0}, Pt3<int>{-1,-1,+1}, Pt3<int>{+0,-1,+1}, Pt3<int>{+0,+0,+1}, Pt3<int>{-1,+0,+1}}, // 0
//////    {Pt3<int>{+0,-1,+0}, Pt3<int>{+1,-1,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{+0,-1,+1}, Pt3<int>{+1,-1,+1}, Pt3<int>{+1,+0,+1}, Pt3<int>{+0,+0,+1}}, // 1
//////    {Pt3<int>{+0,-1,-1}, Pt3<int>{+1,-1,-1}, Pt3<int>{+1,+0,-1}, Pt3<int>{+0,+0,-1}, Pt3<int>{+0,-1,+0}, Pt3<int>{+1,-1,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+0,+0,+0}}, // 2
//////    {Pt3<int>{-1,-1,-1}, Pt3<int>{+0,-1,-1}, Pt3<int>{+0,+0,-1}, Pt3<int>{-1,+0,-1}, Pt3<int>{-1,-1,+0}, Pt3<int>{+0,-1,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{-1,+0,+0}}, // 3
//////    {Pt3<int>{-1,+0,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{+0,+1,+0}, Pt3<int>{-1,+1,+0}, Pt3<int>{-1,+0,+1}, Pt3<int>{+0,+0,+1}, Pt3<int>{+0,+1,+1}, Pt3<int>{-1,+1,+1}}, // 4
//////    {Pt3<int>{+0,+0,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+1,+1,+0}, Pt3<int>{+0,+1,+0}, Pt3<int>{+0,+0,+1}, Pt3<int>{+1,+0,+1}, Pt3<int>{+1,+1,+1}, Pt3<int>{+0,+1,+1}}, // 5
//////    {Pt3<int>{+0,+0,-1}, Pt3<int>{+1,+0,-1}, Pt3<int>{+1,+1,-1}, Pt3<int>{+0,+1,-1}, Pt3<int>{+0,+0,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+1,+1,+0}, Pt3<int>{+0,+1,+0}}, // 6
//////    {Pt3<int>{-1,+0,-1}, Pt3<int>{+0,+0,-1}, Pt3<int>{+0,+1,-1}, Pt3<int>{-1,+1,-1}, Pt3<int>{-1,+0,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{+0,+1,+0}, Pt3<int>{-1,+1,+0}}, // 7

////    {Pt3<int>{-1,-1,+0}, Pt3<int>{+0,-1,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{-1,+0,+0}, Pt3<int>{-1,-1,+1}, Pt3<int>{+0,-1,+1}, Pt3<int>{+0,+0,+1}, Pt3<int>{-1,+0,+1}}, // 0
////    {Pt3<int>{+0,-1,+0}, Pt3<int>{+1,-1,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{+0,-1,+1}, Pt3<int>{+1,-1,+1}, Pt3<int>{+1,+0,+1}, Pt3<int>{+0,+0,+1}}, // 1
////    {Pt3<int>{+0,-1,-1}, Pt3<int>{+1,-1,-1}, Pt3<int>{+1,+0,-1}, Pt3<int>{+0,+0,-1}, Pt3<int>{+0,-1,+0}, Pt3<int>{+1,-1,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+0,+0,+0}}, // 2
////    {Pt3<int>{-1,-1,-1}, Pt3<int>{+0,-1,-1}, Pt3<int>{+0,+0,-1}, Pt3<int>{-1,+0,-1}, Pt3<int>{-1,-1,+0}, Pt3<int>{+0,-1,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{-1,+0,+0}}, // 3
////    {Pt3<int>{-1,+0,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{+0,+1,+0}, Pt3<int>{-1,+1,+0}, Pt3<int>{-1,+0,+1}, Pt3<int>{+0,+0,+1}, Pt3<int>{+0,+1,+1}, Pt3<int>{-1,+1,+1}}, // 4
////    {Pt3<int>{+0,+0,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+1,+1,+0}, Pt3<int>{+0,+1,+0}, Pt3<int>{+0,+0,+1}, Pt3<int>{+1,+0,+1}, Pt3<int>{+1,+1,+1}, Pt3<int>{+0,+1,+1}}, // 5
////    {Pt3<int>{+0,+0,-1}, Pt3<int>{+1,+0,-1}, Pt3<int>{+1,+1,-1}, Pt3<int>{+0,+1,-1}, Pt3<int>{+0,+0,+0}, Pt3<int>{+1,+0,+0}, Pt3<int>{+1,+1,+0}, Pt3<int>{+0,+1,+0}}, // 6
////    {Pt3<int>{-1,+0,-1}, Pt3<int>{+0,+0,-1}, Pt3<int>{+0,+1,-1}, Pt3<int>{-1,+1,-1}, Pt3<int>{-1,+0,+0}, Pt3<int>{+0,+0,+0}, Pt3<int>{+0,+1,+0}, Pt3<int>{-1,+1,+0}}, // 7
////}};




//template<typename acc>
//struct VoxelOctreeNode;

//template<typename acc>
//using VoxelOctreeNodeUP = std::unique_ptr<VoxelOctreeNode<acc>>;

//template<typename acc>
//struct VoxelOctreeNode {

//    VoxelOctreeNode(AABB3<acc> b, Pt3<size_t> id = {0,0,0}) : bounds(b), idNode(id){}

//    AABB3<acc> bounds;
//    std::unique_ptr<std::array<VoxelOctreeNodeUP<acc>,8>> children = nullptr;

//    void get_finals_nodes(std_v1<VoxelOctreeNode<acc>*> &finalNodes) {

//        if(ids.size() > 0){
//            finalNodes.emplace_back(this);
//            return;
//        }

//        if(children){
//            for(const auto &child : *children){
//                child->get_finals_nodes(finalNodes);
//            }
//        }
//    }

//    template<typename acc>
//    auto sorted_nodes(size_t depth) {

//        using VoxN = VoxelOctreeNode<float>*;
//        size_t side = static_cast<size_t>(math::ipow(2,depth));
//        std_v3<VoxN> sortedNodes(side, std_v2<VoxN>(side, std_v1<VoxN>(side, nullptr)));

//        std_v1<VoxelOctreeNode<acc>*> nodes;
//        get_finals_nodes(nodes);

//        for(const auto node : nodes){
//            sortedNodes[node->idNode.x()][node->idNode.y()][node->idNode.z()] = node;
//        }
//        return sortedNodes;
//    }

////private:

//    std::vector<size_t> ids;
//    Pt3<size_t> idNode;
//};




//template<typename acc>
//void split_tree(const std::vector<Voxel<acc>> &voxels, VoxelOctreeNode<acc> *node, int depth, std::vector<size_t> idVoxels = {}){

//    if(depth-- <= 0){
//        // store ids
//        node->ids = std::move(idVoxels);
//        return;
//    }

//    // fill id
//    if(idVoxels.size() == 0){
//        idVoxels.resize(voxels.size());
//        std::iota(idVoxels.begin(), idVoxels.end(), 0);
//    }

//    if(!node->children){
//        node->children = std::make_unique<std::array<VoxelOctreeNodeUP<acc>,8>>();

//        const Pt3<acc> o    = node->bounds.origin;
//        const Vec3<acc> s   = node->bounds.size * acc{0.5};
//        for(size_t ii = 0; ii < 8; ++ii){
//            (*node->children)[ii] = std::make_unique<VoxelOctreeNode<acc>>(AABB3<acc>{o + mu<acc>[ii]*s, s}, node->idNode + mu3[ii]* math::ipow<size_t>(2,depth));
//        }
////        std::cout << " d " << ipow<size_t>(2,depth) << "\n";
//    }

//    std::array<std::vector<size_t>, 8> idsPerChild;
//    for(size_t ii = 0; ii < idsPerChild.size(); ++ii){
//       idsPerChild[ii].reserve(idVoxels.size());
//    }

//    for(size_t ii = 0; ii < idVoxels.size(); ++ii){
//        bool insideAABB = false;
//        for(size_t jj = 0; jj < node->children->size(); ++jj){
//            if(point_in_aabb(voxels[idVoxels[ii]].p,(*node->children)[jj]->bounds)){
//                idsPerChild[jj].emplace_back(idVoxels[ii]);
//                insideAABB = true;
//                break;
//            }
//        }
//        if(!insideAABB){
//            std::cout << "not inside anything: " << voxels[idVoxels[ii]].p  << "\n";
//        }
//    }

//    for(size_t ii = 0; ii < idsPerChild.size(); ++ii){
//        if(idsPerChild[ii].size() > 0){
//            split_tree(voxels, (*node->children)[ii].get(), depth, std::move(idsPerChild[ii]));
//        }
//    }
//}


//}
