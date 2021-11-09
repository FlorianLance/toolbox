
/*******************************************************************************
** Toolbox-opengl-utility                                                     **
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


// base
#include "graphics/model.hpp"

// local
#include "shapes.hpp"
#include "gl_draw.hpp"
#include "opengl/shader/shader.hpp"

namespace tool::gl {

class Drawer{

public:

    float scaleHint = 1;

    virtual void draw(gl::ShaderProgram *shader = nullptr){
        static_cast<void>(shader);
        if(drawableObject != nullptr){
            if(texturesNames.size() > 0){
                TBO::bind_textures(texturesNames);
            }
            drawableObject->render();
        }
    }

    virtual void draw_adjacency(gl::ShaderProgram *shader = nullptr){
        static_cast<void>(shader);
        if(drawableObject != nullptr){
            if(texturesNames.size() > 0){
                TBO::bind_textures(texturesNames);
            }
            drawableObject->render_adjacency();
        }
    }

    virtual void draw_patches(gl::ShaderProgram *shader = nullptr){
        static_cast<void>(shader);
        if(drawableObject != nullptr){
            if(texturesNames.size() > 0){
                TBO::bind_textures(texturesNames);
            }
            drawableObject->render_patches();
        }
    }

    virtual void draw_instances(gl::ShaderProgram *shader = nullptr){

        static_cast<void>(shader);
        if(drawableObject != nullptr){
            if(texturesNames.size() > 0){
                TBO::bind_textures(texturesNames);
            }
            // ...
        }
    }

    gl::Drawable *object() const{
        return drawableObject.get();
    }

protected:
    std::unique_ptr<gl::Drawable> drawableObject = nullptr;
    std_v1<TextureName> texturesNames;
};


class HierarchyDrawer : public Drawer{
protected:
    std_v1<std::unique_ptr<Drawer>> drawableObjects;
    std_v1<std::unique_ptr<Drawer>> children;
};




// procedural geometry
// # 3D lines
class AxesDrawer : public Drawer{
public:
    void init();
};

class FrustumDrawer : public Drawer{
public:
    void init();
};


// # 2D shapes
class FullscreenQuadDrawer : public Drawer{
public:
    void init();
};

class PlaneDrawer : public Drawer{
public:
    void init(float xSize = 8.f, float zSize = 8.f, std_v1<TextureName> textures = {});
};

class GridDrawer : public Drawer{
public:
    void init();
};

// # 3D shapes
// ## points
class CloudPointsDrawer : public Drawer{
public:

    void init(size_t size, geo::Pt2f *points, geo::Pt3f *colors = nullptr);
    void init(std_v1<geo::Pt2f> *points, std_v1<geo::Pt3f> *colors = nullptr);

    void init(size_t size, geo::Pt3f *points, geo::Pt3f *colors = nullptr);
    void init(std_v1<geo::Pt3f> *points, std_v1<geo::Pt3f> *colors = nullptr);
};

// ## triangles
class SkyboxDrawer : public Drawer{
public:
    void init(TextureName cubemap);
};

class SphereDrawer : public Drawer{
public:
    void init(float radius, size_t slices, size_t nbStacks, std_v1<TextureName> textures = {});
};

class CubeDrawer : public Drawer{
public:
    void init(float side = 2.f, std_v1<TextureName> textures = {});
};

class TorusDrawer : public Drawer{
public:
    void init(std_v1<TextureName> textures = {});
};

class TeapotDrawer : public Drawer{
public:
    void init(std_v1<TextureName> textures = {});
};

// loaded geometry
// # mesh
class MeshDrawer : public Drawer{
public:
    void init(geo::Mesh<float> *mesh);
};

class GMeshDrawer : public Drawer{
    std::unordered_map<TextureType, std_v1<std::pair<TextureName,TBO>>> m_textures;
public:

    // init textures from material
    void init(const std::shared_ptr<graphics::GMesh<float>> &gmesh);
    // use input textures
    void init(const std::shared_ptr<graphics::GMesh<float>> &gmesh, const std_v1<graphics::TextureInfo> &texturesInfo);
};

// # model
class ModelDrawer : public HierarchyDrawer{
    std::weak_ptr<graphics::Model<float>> modelP;
public:
    void init(const std::weak_ptr<graphics::Model<float>> &model);
    void init(const std::weak_ptr<graphics::Model<float>> &model, const std_v1<graphics::TextureInfo> &texturesInfo);
    void draw(gl::ShaderProgram *shader = nullptr) override;
    void draw_adjacency(gl::ShaderProgram *shader = nullptr) override;
    void update_animation(const std::string &animationName, float time);

    graphics::Model<float> *model(){
        if(auto m = modelP.lock()){
            return m.get();
        }
        return nullptr;
    }

private:

    void set_bones_uniform(graphics::Model<float> *model, gl::ShaderProgram *shader);
};




}
