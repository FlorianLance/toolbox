
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
#include <memory>
#include <fstream>
#include <iostream>
#include <map>

// local
#include "geometry/point4.hpp"
#include "geometry/matrix4.hpp"
#include "geometry/shapes/triangle3.hpp"
#include "geometry/shapes/aabb3.hpp"
#include "utility/vector.hpp"
#include "utility/string.hpp"


namespace tool::geo {

//template<typename acc>
//struct BVHNode;

//template<typename acc>
//using BVHNodeP = std::unique_ptr<BVHNode<acc>>;

//template<typename acc>
//struct BVHNode{

//    std_v1<AABB3<acc>> get_all_bounds() const{

//        std_v1<AABB3<acc>> boundsA;
//        size_t nbTri = 0;
//        for(const auto& child : children){
//            if(child != nullptr){
//                auto boundsB = child->get_all_bounds();
//                std::move(boundsB.begin(), boundsB.end(), std::back_inserter(boundsA));
//                nbTri += child->triangles.size();
//            }
//        }

//        if(nbTri > 0){
//            boundsA.push_back(bounds);
//        }

//        return boundsA;
//    }

//    AABB3<acc> bounds;
//    std_v1<size_t> triangles;
//    std_a1<BVHNodeP<acc>,8> children  = {nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};
//};

#define NUM_BONES_PER_VERTEX 4

struct BoneData{

    std_a1<int,NUM_BONES_PER_VERTEX> idS;
    std_a1<float, NUM_BONES_PER_VERTEX> weights;

    BoneData(){
        std::fill(std::begin(idS), std::end(idS), 0);
        std::fill(std::begin(weights), std::end(weights), 0.f);
    }

    bool add_bone_data(int boneID, float weight){

        for(size_t ii = 0; ii < idS.size(); ++ii){
            if(almost_equal(weights[ii], 0.f)){
                idS[ii]     = boneID;
                weights[ii] = weight;
                return true;
            }
        }
        return false;
    }

};




template<typename acc>
struct Mesh{

    std_v1<Pt3<acc>> vertices;
    std_v1<Vec3<acc>> normals;
    std_v1<TriIds> triIds;
    std_v1<Pt2f> tCoords;
    std_v1<Pt4f> colors;
    std_v1<Pt4<acc>> tangents;    
    std_v1<BoneData> bones;

    Mesh(){
    }

    Mesh(const Mesh &m){
        *this = m;
    }

    Mesh& operator=(const Mesh &m){
        vertices  = m.vertices;
        normals   = m.normals;
        triIds    = m.triIds;
        tCoords   = m.tCoords;
        colors    = m.colors;
        tangents  = m.tangents;
        bones     = m.bones;
        return *this;
    }

    Mesh(Mesh&& m){
        *this = std::move(m);
    }

    Mesh& operator=(Mesh&& m){
        vertices    = std::move(m.vertices);
        normals     = std::move(m.normals);
        triIds      = std::move(m.triIds);
        tCoords     = std::move(m.tCoords);
        colors      = std::move(m.colors);
        tangents    = std::move(m.tangents);
        bones       = std::move(m.bones);
        return *this;
    }

    inline Triangle3<acc> triangle(size_t id) const{
        return Triangle3<acc>{vertices[triIds[id].ids[0]],vertices[triIds[id].ids[1]],vertices[triIds[id].ids[2]]};
    }

    inline Vec3<acc> triangle_normal(size_t id) const{
        return ((vertices[triIds[id].ids[0]] + vertices[triIds[id].ids[1]] + vertices[triIds[id].ids[2]])/3);
    }


    void generate_normals(){

        if(vertices.size() == 0 || triIds.size() == 0){
            return;
        }

        // compute triangles normals
        std_v1<Vec3<acc>> triNormals;
        triNormals.resize(triIds.size());
        for(size_t ii = 0; ii < triIds.size(); ++ii){
            triNormals[ii] = normalize(cross(
                Vec3<acc>{vec(vertices[triIds[ii].id1()], vertices[triIds[ii].id2()])},
                Vec3<acc>{vec(vertices[triIds[ii].id3()], vertices[triIds[ii].id1()])} )
            );
        }

        // resize array
        normals.resize(vertices.size());

        // compute normals
        std_v1<bool> verticesInTrianglesMsk(vertices.size(), false);
        for(size_t ii = 0; ii < triIds.size(); ++ii){

            for(int jj = 0; jj < 3; ++jj){

                size_t id = triIds[ii].ids[jj];
                verticesInTrianglesMsk[id] = true;

                Vec3<acc> currNormal = triNormals[ii];
                if(jj >= 1){
                    if(dot(currNormal, normals[id]) < 0){
                        currNormal = invert(currNormal);
                    }
                }
                normals[id] += currNormal;
            }
        }

        // normalize and inverse
        Pt3<acc> center(0,0,0);
        for(auto&& v: vertices){
            center += v;
        }
        center /= static_cast<acc>(vertices.size());

        for(size_t ii = 0; ii < vertices.size(); ++ii){

            if(!verticesInTrianglesMsk[ii]){// in this case vertices doesn't belong to a triangle
                normals[ii] = Vec3<acc>{vec(vertices[ii], center)};
            }
            normals[ii] = invert(normalize(normals[ii]));
        }
    }

    void generate_tangents(){

        if(normals.size() != vertices.size()){
            std::cerr << "[Mesh] generate_tangents: no normals defined.\n";
            return;
        }

        if(normals.size() != tCoords.size()){
            std::cerr << "[Mesh] generate_tangents: no tex coords defined.\n";
            return;
        }

        std_v1<Pt3<acc>> tan1Accum(vertices.size());
        std_v1<Pt3<acc>> tan2Accum(vertices.size());
        tangents.resize(vertices.size());

        for(const auto &tri : triIds){

            const Pt3<acc> &p1 = vertices[tri.ids.x()];
            const Pt3<acc> &p2 = vertices[tri.ids.y()];
            const Pt3<acc> &p3 = vertices[tri.ids.z()];

            const Vec2f &tc1 = tCoords[tri.ids.x()];
            const Vec2f &tc2 = tCoords[tri.ids.y()];
            const Vec2f &tc3 = tCoords[tri.ids.z()];

            Vec3<acc> q1 = p2 - p1;
            Vec3<acc> q2 = p3 - p1;
            acc s1 = tc2.x() - tc1.x(), s2 = tc3.x() - tc1.x();
            acc t1 = tc2.y() - tc1.y(), t2 = tc3.y() - tc1.y();
            acc r = 1 / (s1 * t2 - s2 * t1);
            Vec3<acc> tan1{(t2*q1.x() - t1*q2.x()) * r, (t2*q1.y() - t1*q2.y()) * r, (t2*q1.z() - t1*q2.z()) * r};
            Vec3<acc> tan2{(s1*q2.x() - s2*q1.x()) * r, (s1*q2.y() - s2*q1.y()) * r, (s1*q2.z() - s2*q1.z()) * r};

            tan1Accum[tri.ids.x()] += tan1;
            tan1Accum[tri.ids.y()] += tan1;
            tan1Accum[tri.ids.z()] += tan1;
            tan2Accum[tri.ids.x()] += tan2;
            tan2Accum[tri.ids.y()] += tan2;
            tan2Accum[tri.ids.z()] += tan2;
        }

        for(size_t ii = 0; ii < vertices.size(); ++ii){

            const Vec3<acc> &n = normals[ii];
            Vec3<acc> &t1 = tan1Accum[ii];
            Vec3<acc> &t2 = tan2Accum[ii];

            // Gram-Schmidt orthogonalize
            tangents[ii] = Vec4<acc>{normalize(t1 -(dot(n,t1) * n)), acc{1}};

            // Store handedness in w
            tangents[ii].w() = (dot(cross(n,t1), t2 ) < acc{0}) ? acc{-1} : acc{1};
        }
    }

    void convertFacesToAdjancencyFormat(){

//        std_v1<Pt3<acc>> vertices;
//        std_v1<Vec3<acc>> normals;
//        std_v1<TriIds> triIds;
//        std_v1<Pt2f> tCoords;
//        std_v1<Pt4f> colors;
//        std_v1<Pt4<acc>> tangents;
//        std_v1<BoneData> bones;

        // Elements with adjacency info
//        std::vector<GLuint> elAdj(faces.size() * 2);

//        // Copy and make room for adjacency info
//        for( GLuint i = 0; i < faces.size(); i+=3)
//        {
//            elAdj[i*2 + 0] = faces[i];
//            elAdj[i*2 + 1] = std::numeric_limits<GLuint>::max();
//            elAdj[i*2 + 2] = faces[i+1];
//            elAdj[i*2 + 3] = std::numeric_limits<GLuint>::max();
//            elAdj[i*2 + 4] = faces[i+2];
//            elAdj[i*2 + 5] = std::numeric_limits<GLuint>::max();
//        }

//        // Find matching edges
//        for( GLuint i = 0; i < elAdj.size(); i+=6)
//        {
//            // A triangle
//            GLuint a1 = elAdj[i];
//            GLuint b1 = elAdj[i+2];
//            GLuint c1 = elAdj[i+4];

//            // Scan subsequent triangles
//            for(GLuint j = i+6; j < elAdj.size(); j+=6)
//            {
//                GLuint a2 = elAdj[j];
//                GLuint b2 = elAdj[j+2];
//                GLuint c2 = elAdj[j+4];

//                // Edge 1 == Edge 1
//                if( (a1 == a2 && b1 == b2) || (a1 == b2 && b1 == a2) )
//                {
//                    elAdj[i+1] = c2;
//                    elAdj[j+1] = c1;
//                }
//                // Edge 1 == Edge 2
//                if( (a1 == b2 && b1 == c2) || (a1 == c2 && b1 == b2) )
//                {
//                    elAdj[i+1] = a2;
//                    elAdj[j+3] = c1;
//                }
//                // Edge 1 == Edge 3
//                if ( (a1 == c2 && b1 == a2) || (a1 == a2 && b1 == c2) )
//                {
//                    elAdj[i+1] = b2;
//                    elAdj[j+5] = c1;
//                }
//                // Edge 2 == Edge 1
//                if( (b1 == a2 && c1 == b2) || (b1 == b2 && c1 == a2) )
//                {
//                    elAdj[i+3] = c2;
//                    elAdj[j+1] = a1;
//                }
//                // Edge 2 == Edge 2
//                if( (b1 == b2 && c1 == c2) || (b1 == c2 && c1 == b2) )
//                {
//                    elAdj[i+3] = a2;
//                    elAdj[j+3] = a1;
//                }
//                // Edge 2 == Edge 3
//                if( (b1 == c2 && c1 == a2) || (b1 == a2 && c1 == c2) )
//                {
//                    elAdj[i+3] = b2;
//                    elAdj[j+5] = a1;
//                }
//                // Edge 3 == Edge 1
//                if( (c1 == a2 && a1 == b2) || (c1 == b2 && a1 == a2) )
//                {
//                    elAdj[i+5] = c2;
//                    elAdj[j+1] = b1;
//                }
//                // Edge 3 == Edge 2
//                if( (c1 == b2 && a1 == c2) || (c1 == c2 && a1 == b2) )
//                {
//                    elAdj[i+5] = a2;
//                    elAdj[j+3] = b1;
//                }
//                // Edge 3 == Edge 3
//                if( (c1 == c2 && a1 == a2) || (c1 == a2 && a1 == c2) )
//                {
//                    elAdj[i+5] = b2;
//                    elAdj[j+5] = b1;
//                }
//            }
//        }

//        // Look for any outside edges
//        for( GLuint i = 0; i < elAdj.size(); i+=6)
//        {
//            if( elAdj[i+1] == std::numeric_limits<GLuint>::max() ) elAdj[i+1] = elAdj[i+4];
//            if( elAdj[i+3] == std::numeric_limits<GLuint>::max() ) elAdj[i+3] = elAdj[i];
//            if( elAdj[i+5] == std::numeric_limits<GLuint>::max() ) elAdj[i+5] = elAdj[i+2];
//        }

//        // Copy all data back into el
//        faces = elAdj;
    }


//    AABB3<acc> AABB() const{

//        if(vertices.size() == 0){
//            return AABB3<acc>();
//        }

//        Pt3<acc> min = vertices[0];
//        Pt3<acc> max = vertices[0];
//        for(const auto &vertex : vertices){
//            min.x() = std::min(vertex.x(), min.x());
//            min.y() = std::min(vertex.y(), min.y());
//            min.z() = std::min(vertex.z(), min.z());

//            max.x() = std::max(vertex.x(), max.x());
//            max.y() = std::max(vertex.y(), max.y());
//            max.z() = std::max(vertex.z(), max.z());
//        }
//        return aabb_from_points(min, max);
//    }


    void check() const{

        if(vertices.size() == 0){
            std::cout << "no vertices\n";
            return;
        }

//        auto minI = std::min_element(std::begin(vertices), std::end(vertices));
//        auto maxI = std::max_element(std::begin(vertices), std::end(vertices));
        auto comp = ([](const Pt3<acc> &pt){
            return pt == Pt3<acc>{};
        });

        auto countNull = std::count_if(std::begin(vertices), std::end(vertices), comp);
        auto sum = std::accumulate(std::begin(vertices), std::end(vertices), Pt3<acc>{});
        auto mean = sum / static_cast<acc>(vertices.size());
        std::cout << "          [point] " << sum << " " << mean  << " " << countNull << "\n";

        if(normals.size() == 0){
            std::cout << "no normals\n";
            return;
        }

        countNull = std::count_if(std::begin(normals), std::end(normals), comp);
        sum = std::accumulate(std::begin(normals), std::end(normals), Pt3<acc>{});
        mean = sum / static_cast<acc>(normals.size());
        std::cout << "          [normal] " << sum << " " << mean  << " " << countNull << "\n";


        if(bones.size() != 0){



            size_t total0 = 0;
            size_t totalNon0 = 0;

            size_t count0 = 0;
            size_t count1 = 0;
            size_t count2 = 0;
            size_t count3 = 0;
            size_t count4 = 0;

            std::unordered_map<size_t, size_t> bonesIdCount;
            for(const auto &bone : bones){

                size_t localCount = 0;

                for(size_t ii = 0; ii < bone.weights.size(); ++ii){
                    const auto &w = bone.weights[ii];
                    const auto &id = bone.idS[ii];
                    if(!almost_equal(w, 0.f)){
                        ++localCount;

                        if(bonesIdCount.count(id) == 0){
                            bonesIdCount[id] = 1;
                        }else{
                            ++bonesIdCount[id];
                        }
                    }
                }

                count0 += localCount == 0 ? 1 : 0;
                count1 += localCount == 1 ? 1 : 0;
                count2 += localCount == 2 ? 1 : 0;
                count3 += localCount == 3 ? 1 : 0;
                count4 += localCount == 4 ? 1 : 0;

                total0 += bone.weights.size()-localCount;
                totalNon0 += localCount;

            }

            std::cout << "          [bones]\n"
                      << "              count0: " <<  count0 << " count1: " << count1 << " count2: " << count2 << " count3: " << count3 << " count4: " << count4 << "\n"
                      << "              pTotalNon0: " << (1.f*totalNon0/(vertices.size()*4))<< " pTotal0: " << (1.f*total0/(vertices.size()*4)) << "\n"
                      << "              p0: " << (1.f*count0 / (vertices.size()))  << " p1: " << (1.f*count1 / (vertices.size()))  << " p2: " << (1.f*count2 / (vertices.size()))
                      << " p3: " << (1.f*count3 / (vertices.size()))  << " p4: " << (1.f*count4 / (vertices.size())) << "\n";

//            std::cout << "bones mapping\n";
//            for(auto &m : bonesMapping){
//                auto id = m.second;

//                if(bonesIdCount.count(id) != 0){
//                    std::cout << "bone " << m.first << " " << bonesIdCount[id] << " \n" <<  bonesInfo[id].offset << "\n" << bonesInfo[id].final << "\n";
//                    bonesIdCount.erase(id);
//                }else{
//                    std::cout << "bone " << m.first << " no referenced \n";
//                }
//            }

            std::cout << "id remaining: " << bonesIdCount.size() << "\n";
        }
    }

//    bool save_to_obj(const std::string &path) const{

//        std::ofstream flowOBJ(path);

//        // open file to write
//        if(!flowOBJ.is_open()){
//            std::cerr << "-Error : save_to_obj -> Can't open obj file " << path << std::endl;
//            return false;
//        }

//        if(vertices.size() == 0){
//            std::cerr << "-Error : save_to_obj-> surface is empty " << std::endl;
//            return false;
//        }

//        std::ostringstream osV;
//        std_v1<Pt4<float>>::const_iterator itC = colors.cbegin();
//        for(typename std_v1<Pt3<acc>>::const_iterator itV = vertices.cbegin(); itV != vertices.cend(); ++itV)
//        {
//            osV.str(std::string());
//            osV << "v " << itV->x() << " " << itV->y() << " " << itV->z();
//            osV << "\n";
//            flowOBJ << osV.str();
//        }

//        for(auto itT = triIds.begin(); itT != triIds.end(); ++itT){
//            osV.str(std::string());
//            osV << "f ";
//            osV << (itT->ids[0] +1) << " " << (itT->ids[1] + 1) << " " << (itT->ids[2] + 1);
//            osV << "\n";
//            flowOBJ << osV.str();
//        }

//        return true;
//    }



//    BVHNodeP<acc> accelerator = nullptr;
};


//template<typename acc>
//bool load_from_obj(const std::string &path, Mesh<acc> &mesh){

//    std::ifstream fileStream(path);
//    if(!fileStream.is_open()){
//        std::cerr << "-Error : load_from_obj -> Can't open file " << path << std::endl;
//        return false;
//    }

//    bool endFile = false;
//    int nbVertices = 0;
//    int nbNormals = 0;
//    int nbTextureCoords = 0;
//    int nbTri = 0;

//    while(!endFile){

//        std::string line;
//        std::getline(fileStream, line);
//        if(line.size() == 0){
//            endFile = true;
//            break;
//        }

//        if(line[0] == 'v'){
//            if(line[1] == ' ')
//                ++nbVertices;
//            else if(line[1] == 'n')
//                ++nbNormals;
//            else if(line[1] == 't')
//                ++nbTextureCoords;

//        }
//        else if(line[0] == 'f'){
//            ++nbTri;
//        }else if(line[0] == '#'){
//            continue;
//        }else if(line[0] == 'm'){
//            continue;
//        }
//    }

//    std_v1<Pt3<acc>> points(nbVertices);
//    std_v1<Pt3<acc>> normals(nbNormals);
//    std_v1<Pt4<acc>> colors(nbVertices);
//    std_v1<Pt2<float>> textureCoords(nbTextureCoords);
//    std_v1<TriIds> triIds(nbTri);


//    fileStream.clear();
//    fileStream.seekg(0, std::ios::beg);
//    endFile = false;

//    int currentVertexId = 0;
//    int currentNormalId = 0;
//    int currentTextureCoordId = 0;
//    int currentTriId = 0;

//    while(!endFile){

//        std::string line;
//        std::getline(fileStream, line);
//        if(line.size() == 0){
//            endFile = true;
//            break;
//        }

//        std::vector<std::string> elements = str::split(line, ' ');
//        if(elements[0] == "v"){

//            if(elements.size() >= 4){
//                points[currentVertexId] = Pt3<acc>(static_cast<acc>(std::stod(elements[1])),
//                                                   static_cast<acc>(std::stod(elements[2])),
//                                                   static_cast<acc>(std::stod(elements[3])));
//            }

//            if(elements.size() == 7){
//                colors[currentVertexId] = Pt4<float>(std::stof(elements[4]),std::stof(elements[5]),std::stof(elements[6]), 0.f);
//            }
//            currentVertexId++;

//        }else if(elements[0] == "vt"){

//            if(elements.size() == 3){
//                textureCoords[currentTextureCoordId++] = Pt2<float>{std::stof(elements[1]),std::stof(elements[2])};
//            }
//        }else if(elements[0] == "vn"){

//            if(elements.size() == 4){
//                normals[currentNormalId++] = Pt3<acc>(static_cast<acc>(std::stod(elements[1])),
//                                                      static_cast<acc>(std::stod(elements[2])),
//                                                      static_cast<acc>(std::stod(elements[3])));
//            }
//        }else if(elements[0] == "f"){

//            if(elements.size() == 4){
//                triIds[currentTriId++] = TriIds(std::stoi(str::split(elements[1], '/')[0])-1,std::stoi(str::split(elements[2], '/')[0])-1,std::stoi(str::split(elements[3], '/')[0])-1);
//            }
//        }
//    }

//    mesh.vertices = std::move(points);
//    mesh.normals  = std::move(normals);
//    mesh.triIds   = std::move(triIds);
//    mesh.tCoords  = std::move(textureCoords);
//    mesh.colors   = std::move(colors);


//    return true;
//}


//// mesh
//template<typename acc>
//void split_BVHNode(BVHNode<acc> *node, Mesh<acc> &mesh, int depth){

//    if(depth-- == 0){
//        return;
//    }

//    // only split if this node contains triangles
//    if(node->triangles.size() > 0){

//        for(auto &child : node->children){
//            child = std::make_unique<BVHNode<acc>>();
//        }

//        Pt3<acc> c  = node->bounds.origin;
//        Vec3<acc> e = node->bounds.size*acc{0.5};
//        node->children[0]->bounds = AABB3<acc>(c + Vec3<acc>(-e.x(), +e.y(), -e.z()), e);
//        node->children[1]->bounds = AABB3<acc>(c + Vec3<acc>(+e.x(), +e.y(), -e.z()), e);
//        node->children[2]->bounds = AABB3<acc>(c + Vec3<acc>(-e.x(), +e.y(), +e.z()), e);
//        node->children[3]->bounds = AABB3<acc>(c + Vec3<acc>(+e.x(), +e.y(), +e.z()), e);
//        node->children[4]->bounds = AABB3<acc>(c + Vec3<acc>(-e.x(), -e.y(), -e.z()), e);
//        node->children[5]->bounds = AABB3<acc>(c + Vec3<acc>(+e.x(), -e.y(), -e.z()), e);
//        node->children[6]->bounds = AABB3<acc>(c + Vec3<acc>(-e.x(), -e.y(), +e.z()), e);
//        node->children[7]->bounds = AABB3<acc>(c + Vec3<acc>(+e.x(), -e.y(), +e.z()), e);
//    }

//    // if this node was just split
//    if(node->triangles.size() > 0){

//        for(auto &child : node->children){

//            child->triangles.reserve(node->triangles.size());
//            for(auto triId : node->triangles){

//                if(triangle_AABB(mesh.triangle(triId), child->bounds)){
//                    child->triangles.emplace_back(triId);
//                }
//            }
//        }
//    }

//    node->triangles.clear();
//    for(auto &child : node->children){
//        if(child != nullptr){
//            if(child->triangles.size() > 0){
//                split_BVHNode(child.get(), mesh, depth);
//            }else{
//                child = nullptr;
//            }
//        }
//    }
//}

//template<typename acc>
//void accelerate_mesh(Mesh<acc> &mesh){

//    if(mesh.accelerator){
//        return;
//    }

//    mesh.accelerator = std::make_unique<BVHNode<acc>>();
//    mesh.accelerator->bounds    = mesh.AABB();
//    mesh.accelerator->triangles.resize(mesh.triIds.size());

//    std::iota(mesh.accelerator->triangles.begin(), mesh.accelerator->triangles.end(), acc{0});
//    split_BVHNode(mesh.accelerator.get(), mesh, 3);
//}
}

