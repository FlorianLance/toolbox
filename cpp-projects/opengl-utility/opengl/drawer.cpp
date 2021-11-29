
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

#include "drawer.hpp"
#include "gl_texture.hpp"

using namespace tool;
using namespace tool::gl;


void AxesDrawer::init(){
    drawableObject = std::make_unique<Axes>(1.f);
}

void GridDrawer::init(){
    drawableObject = std::make_unique<Grid>(0.2f,0.2f, 100,100);
}

void FullscreenQuadDrawer::init(){
    drawableObject = std::make_unique<FullscreenQuad>();
}

void TorusDrawer::init(std_v1<TextureName> textures){
    texturesNames  = textures;
//    drawableObject = std::make_unique<Torus>(0.7f, 0.3f, 30, 30);
    drawableObject = std::make_unique<Torus>(0.7f * 1.5f, 0.3f * 1.5f, 50,50);

}

void PlaneDrawer::init(float xSize, float zSize, std_v1<TextureName> textures){
    texturesNames  = textures;
    drawableObject = std::make_unique<Plane>( static_cast<GLfloat>(xSize), static_cast<GLfloat>(zSize), 1, 1, GLfloat{10}, GLfloat{7});
}

void SkyboxDrawer::init(std::optional<TextureName> cubemap){
    if(cubemap.has_value()){
        texturesNames.emplace_back(cubemap.value());
    }
    drawableObject = std::make_unique<Skybox>(100.f);
}

void SphereDrawer::init(float radius, size_t slices, size_t nbStacks, std_v1<TextureName> textures){
    texturesNames  = textures;
    drawableObject = std::make_unique<Sphere>(radius, static_cast<GLuint>(slices), static_cast<GLuint>(nbStacks));
}

void CubeDrawer::init(float side, std_v1<TextureName> textures){
    texturesNames  = textures;
    drawableObject = std::make_unique<Cube>(side);
}

void CloudPointsDrawer::init(size_t size, geo::Pt2f *points, geo::Pt3f *colors){
    drawableObject = std::make_unique<gl::Cloud>(size, points, colors);
}

void CloudPointsDrawer::init(std_v1<geo::Pt2f> *points, std_v1<geo::Pt3f> *colors){
    drawableObject = std::make_unique<gl::Cloud>(points, colors);
}

void CloudPointsDrawer::init(size_t size, geo::Pt3f *points, geo::Pt3f *colors){
    drawableObject = std::make_unique<gl::Cloud>(size, points, colors);
}

void CloudPointsDrawer::init(std_v1<geo::Pt3f> *points, std_v1<geo::Pt3f> *colors){
    drawableObject = std::make_unique<gl::Cloud>(points, colors);
}

void VoxelsDrawer::init(size_t size, geo::Pt3<int> *voxels, geo::Pt3f *colors){
    drawableObject = std::make_unique<gl::Voxels>(size, voxels, colors);
}

void MeshDrawer::init(geo::Mesh<float> *mesh){
    drawableObject = std::make_unique<gl::Mesh>(mesh);
}

void GMeshDrawer::init(const std::shared_ptr<graphics::GMesh<float>> &gmesh){

    for(auto &texturesT : gmesh->material->texturesInfo){
        for(auto textureInfo : texturesT.second){
            gl::Texture2D tbo;
            tbo.load_texture(textureInfo.texture, textureInfo.options);
            m_textures[textureInfo.options.type].emplace_back(std::make_pair(TextureName{tbo.id()},std::move(tbo)));
        }
    }

    // add textures in good order
    if(m_textures.count(TextureType::diffuse)!=0){
        texturesNames.emplace_back(m_textures[TextureType::diffuse][0].second.id());
    }
    if(m_textures.count(TextureType::normal)!=0){
        texturesNames.emplace_back(m_textures[TextureType::normal][0].second.id());
    }
    if(m_textures.count(TextureType::height)!=0){
        texturesNames.emplace_back(m_textures[TextureType::height][0].second.id());
    }

    drawableObject = std::make_unique<gl::Mesh>(&gmesh->mesh);
}

void GMeshDrawer::init(const std::shared_ptr<graphics::GMesh<float>> &gmesh, const std_v1<tool::graphics::TextureInfo> &texturesInfo){

    for(auto textureInfo : texturesInfo){
        gl::Texture2D tbo;
        tbo.load_texture(textureInfo.texture);
        m_textures[textureInfo.options.type].emplace_back(std::make_pair(TextureName{tbo.id()},std::move(tbo)));
    }

    // add textures in good order
    if(m_textures.count(TextureType::diffuse)!=0){
        texturesNames.emplace_back(m_textures[TextureType::diffuse][0].second.id());
    }
    if(m_textures.count(TextureType::normal)!=0){
        texturesNames.emplace_back(m_textures[TextureType::normal][0].second.id());
    }
    if(m_textures.count(TextureType::height)!=0){
        texturesNames.emplace_back(m_textures[TextureType::height][0].second.id());
    }

    drawableObject = std::make_unique<gl::Mesh>(&gmesh->mesh);
}

void ModelDrawer::init(const std::weak_ptr<graphics::Model<float>> &model){

    modelP = model;
    if(auto m = modelP.lock()){

        for(auto &gmesh : m->gmeshes){
            auto gmeshD = std::make_unique<GMeshDrawer>();
            gmeshD->init(gmesh);
            drawableObjects.emplace_back(std::move(gmeshD));
        }

        for(auto &modelChild : m->children){
            auto modelDrawer = std::make_unique<ModelDrawer>();
            modelDrawer->init(modelChild);
            children.emplace_back(std::move(modelDrawer));
        }                
    }
}

void ModelDrawer::init(const std::weak_ptr<graphics::Model<float>> &model, const std_v1<graphics::TextureInfo> &texturesInfo){

    modelP = model;
    if(auto m = modelP.lock()){

        for(auto &gmesh : m->gmeshes){
            auto gmeshD = std::make_unique<GMeshDrawer>();
            gmeshD->init(gmesh, texturesInfo);
            drawableObjects.emplace_back(std::move(gmeshD));
        }

        for(auto &modelChild : m->children){
            auto modelDrawer = std::make_unique<ModelDrawer>();
            modelDrawer->init(modelChild);
            children.emplace_back(std::move(modelDrawer));
        }
    }
}

void ModelDrawer::draw(gl::ShaderProgram *shader){

    if(auto model = modelP.lock()){

        set_bones_uniform(model.get(), shader);

        for(auto &gmesh : drawableObjects){
            gmesh->draw();
        }

        for(auto &child : children){
            child->draw(shader);
        }
    }
}

void ModelDrawer::draw_adjacency(ShaderProgram *shader){

    if(auto model = modelP.lock()){

        set_bones_uniform(model.get(), shader);

        for(auto &gmesh : drawableObjects){
            gmesh->draw_adjacency();
        }

        for(auto &child : children){
            child->draw_adjacency(shader);
        }
    }
}

void ModelDrawer::update_animation(const std::string &animationName, float time){

    if(auto model = modelP.lock()){
        model->update_animation(animationName, time);
    }

    for(auto &child : children){
        dynamic_cast<ModelDrawer*>(child.get())->update_animation(animationName, time);
    }
}

void ModelDrawer::set_bones_uniform(graphics::Model<float> *model, ShaderProgram *shader){

    if(shader == nullptr){
        return;
    }

    if(model->animations.size() > 0){

        std_v1<geo::Mat4f> transforms;
        for(size_t ii = 0; ii < model->bonesInfo.size(); ++ii){
            transforms.emplace_back(model->bonesInfo[ii].final);
        }
        if(transforms.size() > 100){
            transforms.resize(100);
        }

        shader->set_uniform("BonesM[0]", transforms);
    }
}

void TeapotDrawer::init(std_v1<TextureName> textures){
    texturesNames  = textures;
    drawableObject = std::make_unique<Teapot>(14);
}


void FrustumDrawer::init(){
    drawableObject = std::make_unique<Frustum>();
}

