
#include "samples.hpp"

// std
#include <random>
#include <algorithm>

// imgui
#include "imgui/imgui.h"
#include "imgui-sfml/imgui-SFML.h"

// base
#include "utility/files.hpp"

// opengl-utility
#include "opengl/buffer/buffer-utility.hpp"
#include "opengl/utility/gl_utility.hpp"




using namespace tool::geo;
using namespace tool::graphics;
using attachment = tool::gl::FrameBufferAttachment;


void Sample::update(float elapsedSeconds){

    this->elapsedSeconds = elapsedSeconds;

    // update lights positions
    float lightRotationSpeed = 1.5f;
    float deltaT = elapsedSeconds * lightRotationSpeed;

    alpha = deltaT;

    auto ray= 20.f;
    auto x = ray * cos(alpha);
    auto z = ray * sin(alpha);
    if(moveLight){
        geo::Pt3f offset = {0,5.f,0.f};
        mobileLightPos1 = geo::Pt4f{offset + geo::Pt3f{x,0,z}, 1.f};
        offset = {0,4.f,1.f};
        mobileLightPos2 = geo::Pt4f{offset + geo::Pt3f{-x,0,z}, 1.f};
    }
}


void Sample::draw(tool::gl::Drawer *drawer){

    if(drawLights){
        draw_lights();
    }
    draw_skybox();

    if(drawFloor){
        //  draw_floor();
    }

    // update animation of current drawer
    nbAnimations = 0;
    if(auto modelDrawer = dynamic_cast<gl::ModelDrawer*>(drawer)){
        if(auto model = modelDrawer->model()){

            nbAnimations = static_cast<int>(model->animations.size());

            if(idAnimation < nbAnimations){

                durationAnimation = model->animations[idAnimation].duration;

                modelDrawer->update_animation(
                    model->animations[idAnimation].name,
                    stopAnimation ? timeAnimation : elapsedSeconds
                );
            }
        }
    }
}

void Sample::update_imgui(){
    if (ImGui::CollapsingHeader("Common parameters")){

        ImGui::Text("########### Misc:");
        ImGui::Checkbox("Draw floor", &drawFloor);

        ImGui::Text("########### Skybox:");
        ImGui::Checkbox("SB-draw", &drawSkybox);
        ImGui::SliderFloat3("SB-rotation", skyboxRot.v.data(), -360.f, 360.f, "ratio = %.1f");

        ImGui::Text("########### Lights:");
        ImGui::Checkbox("L-move", &moveLight);
        ImGui::Checkbox("L-draw", &drawLights);
        ImGui::SliderFloat3("L-Ambiant", lInfo.La.v.data(), 0.f, 1.f, "r = %.2f");
        ImGui::SliderFloat3("L-Diffuse", lInfo.Ld.v.data(), 0.f, 1.f, "r = %.2f");
        ImGui::SliderFloat3("L-Specular", lInfo.Ls.v.data(), 0.f, 1.f, "r = %.2f");

        ImGui::Text("########### Blinn-phong material:");
        ImGui::SliderFloat3("BPM-Coeff ambiant",  mInfo.Ka.v.data(), 0.f, 1.f, "r = %.2f");
        ImGui::SliderFloat3("BPM-Coeff diffuse",  mInfo.Kd.v.data(), 0.f, 1.f, "r = %.2f");
        ImGui::SliderFloat3("BPM-Coeff specular", mInfo.Ks.v.data(), 0.f, 1.f, "r = %.2f");
        ImGui::SliderFloat("BPM-Shininess",       &mInfo.Shininess, 0.f, 1000.f, "v = %.1f");

        ImGui::Text("########### Model:");
        ImGui::Text("### Coords:");
        ImGui::SliderInt3("M-Numbers", nb.v.data(), 1, 10);
        ImGui::SliderFloat3("M-Position", modelPos.v.data(), -10.f, 10.f, "ratio = %.2f");
        ImGui::SliderFloat3("M-Rotation", modelRot.v.data(), -360.f, 360.f, "ratio = %.1f");
        ImGui::SliderFloat("M-Scale", &scale, 0.01f, 5.f, "ratio = %.2f");
        ImGui::Text("### Animation:");
        ImGui::SliderInt("MA-id", &idAnimation, 0, nbAnimations);
        ImGui::Checkbox("MA-Stop animation", &stopAnimation);
        ImGui::SliderFloat("MA-Time animation", &timeAnimation, 0.f, durationAnimation, "ratio = %.2f");
    }
}

void Sample::draw_nb(gl::ShaderProgram *shader, tool::gl::Drawer *drawer){

    float s = scale*drawer->scaleHint;
    auto p = modelPos.conv<double>();
    for(int ii = 0; ii < nb.x(); ++ii){
        for(int jj = 0; jj < nb.y(); ++jj){
            for(int kk = 0; kk < nb.z(); ++kk){
                model = Mat4d::transform2({s,s,s}, modelRot.conv<double>(),
                            {p.x() + 1.f*(ii-nb.x()/2), p.y() + 1.f*(jj-nb.y()/2), p.z() + 1.f*(kk-nb.z()/2)});
                shader->set_model_matrix(model);
                update_matrices();
                shader->set_camera_matrices_uniforms(camM);
                drawer->draw(shader);
            }
        }
    }
}

void Sample::update_matrices(){
    camM = camera->compute_camera_matrices(model);
}

void Sample::update_matrices(Mat4d view, Mat4d proj){
    camM = Camera::compute_camera_matrices(model, view, proj);
}

void Sample::update_matrices(Mat4d model, Mat4d view, Mat4d proj){
    this->model = model;
    camM = Camera::compute_camera_matrices(model, view, proj);
}

void Sample::update_matrices_with_identity(){
    model = geo::Mat4d(true);
    update_matrices(geo::Mat4d(true), geo::Mat4d(true));
}

void Sample::draw_screen_quad(tool::gl::ShaderProgram *shader){

    update_matrices_with_identity();
    shader->set_camera_matrices_uniforms(camM);

    if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
        drawer->draw(shader);
    }
}

void Sample::draw_floor(){

    graphics::MaterialInfo floorM;
    floorM.Ka = {0.5f, 0.5f, 0.5f};
    floorM.Kd = {0.5f, 0.5f, 0.5f};
    floorM.Ks = {0.8f, 0.8f, 0.8f};
    floorM.Shininess = 10.0f;

    if(auto shader = shadersM->get_ptr("ch5/scene-texture"); shader != nullptr){

        model = Mat4d::transform({10.,10.,10.},Vec3d{0.,0.,0},{0.,-4.,0.});
        update_matrices();

        shader->use();
        shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
        shader->set_uniform("Light.La", Vec3f{0.2f,0.2f,0.2f});
        shader->set_uniform("Light.Position",camera->view().conv<float>().multiply_point(Sample::worldLight));
        shader->set_camera_matrices_uniforms(camM);

        materialUBO.update(floorM);
        materialUBO.bind(1);

        if(auto drawer = drawersM->get_drawer_ptr("floor-drawer"); drawer != nullptr){
            drawer->draw();
        }
    }
}

void Sample::draw_lights(){

    if(auto shader = shadersM->get_ptr("others/unicolor"); shader != nullptr){

        if(auto drawer = drawersM->get_drawer_ptr("sphere-drawer"); drawer != nullptr){

            shader->use();
            shader->set_uniform("unicolor", geo::Pt3f{1.f,1.f,0.f});

            model = Mat4d::translate(Mat4d(true), Sample::worldLight.xyz().conv<double>());
            model = Mat4d::scale(model, Vec3d{0.3,0.3,0.3});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);
            drawer->draw();

            model = Mat4d::translate(Mat4d(true), Sample::mobileLightPos1.xyz().conv<double>());
            model = Mat4d::scale(model, Vec3d{0.3,0.3,0.3});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);
            drawer->draw();

            model = Mat4d::translate(Mat4d(true), Sample::mobileLightPos2.xyz().conv<double>());
            model = Mat4d::scale(model, Vec3d{0.3,0.3,0.3});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);
            drawer->draw();

            shader->set_camera_matrices_uniforms(camM);
            drawer->draw();
        }
    }
}

void Sample::draw_skybox(){

    if(!drawSkybox){
        gl::TBO::unbind_textures(0,1);
        return;
    }

    if(auto shader = shadersM->get_ptr("others/skybox"); shader != nullptr){

        model = Mat4d::transform({1.,1.,1.},skyboxRot.conv<double>(),{0.,0.,0.});
        update_matrices();

        shader->use();
        shader->set_camera_matrices_uniforms(camM);

        gl::TBO::bind_textures({texturesM->id("grace")},0);

        if(auto drawer = drawersM->get_drawer_ptr("skybox-drawer"); drawer != nullptr){
            drawer->draw();            
        }
    }
}

void Ch3Diffuse::init(){
    shader = shadersM->get_ptr("ch3/diffuse");
}

void Ch3Diffuse::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();
    shader->set_uniform("LightPosition", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});
    shader->set_uniform("Kd", geo::Pt3f{0.9f, 0.5f, 0.3f});
    shader->set_uniform("Ld", lInfo.Ld);

    draw_nb(shader, drawer);
}


void Ch3Diffuse::update_imgui(){
    Sample::update_imgui();
}

void Ch3Flat::init(){
    shader = shadersM->get_ptr("ch3/flat");
    lightUBO.generate();
    lightUBO.set_data_space_from_shader(shader);
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch3Flat::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    lInfo.Position = camera->view().multiply_point(worldLight.conv<double>()).conv<float>();
    lightUBO.update(lInfo);
    lightUBO.bind(0);

    materialUBO.update(mInfo);
    materialUBO.bind(1);

    draw_nb(shader,drawer);
}


void Ch3Discard::init(){
    shader = shadersM->get_ptr("ch3/discard");
    lightUBO.generate();
    lightUBO.set_data_space_from_shader(shader);
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch3Discard::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    lInfo.Position = camera->view().multiply_point(worldLight.conv<double>()).conv<float>();
    lightUBO.update(lInfo);
    lightUBO.bind(0);

    materialUBO.update(mInfo);
    materialUBO.bind(1);

    shader->set_uniform("discardV", discardV);
    shader->set_uniform("scaleV", scaleV);

    draw_nb(shader,drawer);
}

void Ch3Discard::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("discard value:", &discardV, 0.f, 1.f, "r = %.3f");
    ImGui::SliderFloat("scale value:", &scaleV, 0.f, 100.f, "v = %.2f");
}


void Ch3TwoSide::init(){
    shader = shadersM->get_ptr("ch3/twoside");
    lightUBO.generate();
    lightUBO.set_data_space_from_shader(shader);
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch3TwoSide::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    lInfo.Position = camera->view().multiply_point(worldLight.conv<double>()).conv<float>();
    lightUBO.update(lInfo);
    lightUBO.bind(0);

    materialUBO.update(mInfo);
    materialUBO.bind(1);

    draw_nb(shader,drawer);
}


void Ch3Phong::init(){
    shader = shadersM->get_ptr("ch3/phong");
    lightUBO.generate();
    lightUBO.set_data_space_from_shader(shader);
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch3Phong::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    lInfo.Position = camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>();

    lightUBO.update(lInfo);
    lightUBO.bind(0);

    materialUBO.update(mInfo);
    materialUBO.bind(1);

    draw_nb(shader,drawer);
}

void Ch4PhongDirectionnalLight::init(){
    shader = shadersM->get_ptr("ch4/phong-directional-light");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch4PhongDirectionnalLight::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();
    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", lInfo.La);
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(worldLight.conv<double>()).conv<float>()});

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawer);
}


void Ch4BlinnPhong::init(){
    shader = shadersM->get_ptr("ch4/blinn-phong");
    lightUBO.generate();
    lightUBO.set_data_space_from_shader(shader);
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch4BlinnPhong::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    lInfo.Position = camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>();

    lightUBO.update(lInfo);
    lightUBO.bind(0);

    materialUBO.update(mInfo);
    materialUBO.bind(1);

    draw_nb(shader,drawer);
}


void Ch4Cartoon::init(){
    shader = shadersM->get_ptr("ch4/cartoon");

    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch4Cartoon::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", lInfo.La);
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});
    shader->set_uniform("levels", levels);

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawer);
}

void Ch4Cartoon::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderInt("Levels", &levels, 1, 20);
}


void Ch4PhongMultiLights::init(){
    shader = shadersM->get_ptr("ch4/phong-multi-lights");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch4PhongMultiLights::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    std_v1<Pt4f> lPos ={
        Pt4f{camera->view().multiply_point({5.0,5.0,2.0,1.0}).conv<float>()},
        Pt4f{camera->view().multiply_point({0.0,5.0,2.0,1.0}).conv<float>()},
        Pt4f{camera->view().multiply_point({5.0,0.0,2.0,1.0}).conv<float>()},
        Pt4f{camera->view().multiply_point({5.0,5.0,0.0,1.0}).conv<float>()},
        Pt4f{camera->view().multiply_point({0.0,5.0,0.0,1.0}).conv<float>()},
        };

    std_v1<Vec3f> lL ={
        Vec3f{0.f,0.8f,0.8f},
        Vec3f{8.f,0.8f,0.8f},
        Vec3f{0.f,0.8f,0.0f},
        Vec3f{0.f,0.8f,0.8f},
        Vec3f{0.8f,0.8f,0.0f}
    };

    std_v1<Vec3f> lLa ={
        Vec3f{0.f,0.2f,0.2f},
        Vec3f{0.f,0.2f,0.2f},
        Vec3f{0.f,0.2f,0.2f},
        Vec3f{0.f,0.2f,0.2f},
        Vec3f{0.f,0.2f,0.2f}
    };

    shader->use();
    for(size_t ii = 0; ii < lPos.size(); ++ii){
        std::string lightName = "lights[" + std::to_string(ii) + "].";
        shader->set_uniform((lightName + "L").c_str(), lL[ii]);
        shader->set_uniform((lightName + "La").c_str(), lLa[ii]);
        shader->set_uniform((lightName + "Position").c_str(), lPos[ii]);
    }

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader, drawer);
}


void Ch4PhongPerFragment::init(){
    shader = shadersM->get_ptr("ch4/phong-per-fragment");

    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch4PhongPerFragment::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();
    shader->set_uniform("Light.L",  Vec3f{0.f,0.8f,0.8f});
    shader->set_uniform("Light.La", lInfo.La);
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawer);
}


void Ch4PBR::init(){

    shader = shadersM->get_ptr("ch4/pbr");

    materialsB.generate();
    materialsB.set_data_storage(1*sizeof(MaterialPbr), &mPbrInfo, GL_DYNAMIC_STORAGE_BIT);

    lights.resize(3);
    lights[0].La = Vec3f{45.0f,45.0f,45.0f};
    lights[1].La = Vec3f{15.0f,15.0f,15.0f};
    lights[2].La = Vec3f{30.0f,30.0f,30.0f};

    lightsB.generate();
    lightsB.set_data_storage(lights.size()*sizeof(Light2), lights.data(), GL_DYNAMIC_STORAGE_BIT);
}

void Ch4PBR::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();
    lights[0].Position = camera->view().conv<float>().multiply_point(mobileLightPos1);
    lights[1].Position = camera->view().conv<float>().multiply_point(mobileLightPos2);
    lights[2].Position = camera->view().conv<float>().multiply_point(worldLight);
    lightsB.update_data(lights.data(), lights.size()*sizeof(Light2));
    lightsB.bind(0);

    materialsB.update_data(&mPbrInfo, 1*sizeof(MaterialPbr));
    materialsB.bind(1);

    draw_nb(shader, drawer);
}

void Ch4PBR::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat4("PBR color", mPbrInfo.color.v.data(), 0.0f, 1.00f, "ratio = %.3f");
    ImGui::SliderFloat("PBR rough", &mPbrInfo.rough, 0.0f, 1.00f, "ratio = %.3f");
    ImGui::SliderFloat("PBR metal", &mPbrInfo.metal, 0.0f, 1.00f, "ratio = %.3f");
}


void Ch5DiscardPixels::init(){
    shader = shadersM->get_ptr("ch5/discard-pixels");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch5DiscardPixels::draw(tool::gl::Drawer *){

    Sample::draw();

    shader->use();
    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", lInfo.La);
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});
    shader->set_uniform("ModelViewMatrix",   camM.mv.conv<float>());
    shader->set_uniform("decay_factor",      decayFactor);

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawersM->get_drawer_ptr("cement-moss-cube-drawer"));
}

void Ch5DiscardPixels::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("Decay", &decayFactor, 0.0f, 1.00f, "ratio = %.3f");
}

void Ch5SceneTexture::init(){
    shader = shadersM->get_ptr("ch5/scene-texture");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch5SceneTexture::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawer);
}

void Ch5SceneMutliTexture::init(){
    shader = shadersM->get_ptr("ch5/scene-multi-textures");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch5SceneMutliTexture::draw(tool::gl::Drawer *){

    Sample::draw();

    shader->use();

    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawersM->get_drawer_ptr("brick-moss-cube-drawer"));

}
void Ch5NormalMap::init(){

    shader = shadersM->get_ptr("ch5/normal-map");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch5NormalMap::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    shader->set_uniform("animate", nbAnimations > 0);
    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawer);
}

void Ch5ParallaxMapping::init(){
    shader = shadersM->get_ptr("ch5/parallax-mapping");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch5ParallaxMapping::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();
    shader->set_uniform("showHeightMap", showHeightMap);
    shader->set_uniform("bumpFactor", bumpFactor);
    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});

    materialUBO.update(mInfo);
    materialUBO.bind(0);

    draw_nb(shader,drawersM->get_drawer_ptr("multi-tex-plane-drawer"));
}

void Ch5ParallaxMapping::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::Checkbox("Show heightmap", &showHeightMap);
    ImGui::SliderFloat("Bump factor", &bumpFactor, 0.0f, 1.f, "ratio = %.3f");
}

void Ch5SteepParallaxMapping::init(){
    shader = shadersM->get_ptr("ch5/steep-parallax-mapping");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch5SteepParallaxMapping::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();
    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});
    shader->set_uniform("bumpScale", bumpScale);

    materialUBO.bind(0);
    materialUBO.update(mInfo);

    draw_nb(shader,drawersM->get_drawer_ptr("multi-tex-plane-drawer"));
}

void Ch5SteepParallaxMapping::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("Bumpscale", &bumpScale, 0.001f, 0.20f, "ratio = %.3f");
}

void Ch5ReflectCubeMap::init(){
    shader = shadersM->get_ptr("ch5/reflect-cubemap");
}

void Ch5ReflectCubeMap::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    shader->set_uniform("WorldCameraPosition", camera->position().conv<float>());
    shader->set_uniform("MaterialColor", matColor);
    shader->set_uniform("ReflectFactor", reflectFactor);
    shader->set_model_matrix(model);
    shader->set_camera_matrices_uniforms(camM);

    draw_nb(shader, drawer);
}

void Ch5ReflectCubeMap::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("Reflect factor", &reflectFactor, 0.0f, 1.f, "ratio = %.3f");
    ImGui::SliderFloat4("Mat color", matColor.v.data(), 0.0, 1.0f, "ratio = %.2f");
}

void Ch5RefractCubeMap::init(){
    shader = shadersM->get_ptr("ch5/refract-cubemap");
}

void Ch5RefractCubeMap::draw(tool::gl::Drawer *drawer){

    Sample::draw(drawer);

    shader->use();

    shader->set_uniform("WorldCameraPosition", camera->position().conv<float>());
    shader->set_uniform("Material.Eta", rmInfo.eta);
    shader->set_uniform("Material.ReflectionFactor", rmInfo.reflectionFactor);

    draw_nb(shader, drawer);
}

void Ch5RefractCubeMap::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("refract Eta", &rmInfo.eta, 0.0f, 1.00f, "ratio = %.3f");
    ImGui::SliderFloat("reflect factor", &rmInfo.reflectionFactor, 0.0f, 1.00f, "ratio = %.3f");
}









void Ch6HdrLightingToneMapping::init(){
    update_screen_size();
}

void Ch6HdrLightingToneMapping::update_screen_size(){

    int size = camera->screen()->size();
    texData.resize(size*3);

    // Generate and bind the framebuffer
    hdrFBO.clean();
    hdrFBO.generate();
    hdrFBO.bind();

    // Create the depth buffer
    hdrDepthBuffer.clean();
    hdrDepthBuffer.generate();
    hdrDepthBuffer.bind();
    hdrDepthBuffer.set_data_storage(camera->screen()->width(),camera->screen()->height());

    // Create the  HDR texture object
    hdrRenderTexture.clean();
    hdrRenderTexture.init_hdr_render(camera->screen()->width(),camera->screen()->height(), 4);

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::nearest;
    options.maxLevel = 0;
    hdrRenderTexture.set_texture_options(options);

    // Bind the texture to the FBO
    hdrFBO.attach_color0_texture(hdrRenderTexture);

    // Bind the depth buffer to the FBO
    hdrFBO.attach_depth_buffer(hdrDepthBuffer);    

    // set colors buffers to be drawn
    hdrFBO.set_draw_buffers({
        attachment::none,
        attachment::color0
    });

    gl::FBO::unbind();
}

void Ch6HdrLightingToneMapping::draw(tool::gl::Drawer *drawer){

    if(auto shader = shadersM->get_ptr("ch6/hdr-lighting-tone-mapping"); shader != nullptr){

        shader->use();

        auto intense = Vec3f{1.0f,1.0f,1.0f};
        shader->set_uniform("Lights[0].L", intense );
        shader->set_uniform("Lights[1].L", intense );
        shader->set_uniform("Lights[2].L", intense );

        intense = Vec3f{0.2f,0.2f,0.2f};
        shader->set_uniform("Lights[0].La", intense );
        shader->set_uniform("Lights[1].La", intense );
        shader->set_uniform("Lights[2].La", intense );
        shader->set_uniform("DoToneMap", doToneMap);

        // pass 1
        {
            shader->set_uniform("Pass", 1);

            hdrFBO.bind();

            glClearColor(0.5f,0.5f,0.5f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);

            auto lightPos = Vec4f{0.0f, 4.0f, 2.5f, 1.0f};
            lightPos.x() = -7.0f;
            shader->set_uniform("Lights[0].Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});
            lightPos.x() = 0.0f;
            shader->set_uniform("Lights[1].Position", Pt4f{camera->view().multiply_point(lightPos.conv<double>()).conv<float>()});
            lightPos.x() = 7.0f;
            shader->set_uniform("Lights[2].Position", Pt4f{camera->view().multiply_point(lightPos.conv<double>()).conv<float>()});

            shader->set_uniform("Material.Kd", geo::Vec3f{0.9f, 0.3f, 0.2f});
            shader->set_uniform("Material.Ks", geo::Vec3f{1.0f, 1.0f, 1.0f});
            shader->set_uniform("Material.Ka", geo::Vec3f{0.2f, 0.2f, 0.2f});
            shader->set_uniform("Material.Shininess", 100.0f);

            // The backdrop plane
            // model = Mat4d::transform({2,1.,1.},Vec3d{-90.,0.,180.},{0.0f,0.0f,4.0f});
            model = Mat4d::rotate(Mat4d(true), Vec3d{1,0,0}, 90.);
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // The bottom plane
            // model = Mat4d::transform({2.,1.,1.},Vec3d{0.,0.,0.},{0.0f,-5.0f,0.0f});
            model = Mat4d(true);
            model = Mat4d::translate(model, Vec3d{0,-5,0});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // Top plane
            //model = Mat4d::transform({2.,1.,1.},Vec3d{-180.,0.,0.},{0.0f,5.0f,0.0f});
            model = Mat4d(true);
            model = Mat4d::translate(model, Vec3d{0,5,0});
            model = Mat4d::rotate(model, Vec3d{1,0,0}, 180.);
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // sphere
            shader->set_uniform("Material.Kd", geo::Vec3f{0.4f, 0.9f, 0.4f});

            //model = Mat4d::transform({1.,1.,1.},Vec3d{0.,0.,0.},{3.0f,-3.0f,2.0f});
            model = Mat4d(true);
            model = Mat4d::translate(model, Vec3d{-3,-3,2.0});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("sphere-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // teapot
            shader->set_uniform("Material.Kd", geo::Vec3f{0.4f, 0.4f, 0.9f});

            //model = Mat4d::transform({1.,1.,1.},Vec3d{90.,0.,0.},{-3.0f,-5.0f,1.5f});
            model = Mat4d(true);
            model = Mat4d::translate(model, Vec3d{4,-5,1.5});
            model = Mat4d::rotate(model, Vec3d{1,0,0}, -90.);
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("teapot-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }


        // compute log avg luminance
        {
            const int size = camera->screen()->size();
            hdrRenderTexture.bind();
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, texData.data());

            float sum = 0.0f;
            size_t count = 0;
            for( int i = 0; i < size; i++ ) {
                float lum = geo::dot(
                    geo::Pt3f{texData[i*3+0], texData[i*3+1], texData[i*3+2]},
                    geo::Pt3f{0.2126f, 0.7152f, 0.0722f} );
                if(texData[i*3+0] != 0){
                    count++;
                }
                if(texData[i*3+1] != 0){
                    count++;
                }

                if(texData[i*3+2] != 0){
                    count++;
                }
                sum += logf( lum + 0.00001f );
            }
            shader->set_uniform( "AveLum", expf( sum / size ) );
        }


        // pass 2
        {
            shader->set_uniform("Pass", 2);

            gl::FBO::unbind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDisable(GL_DEPTH_TEST);

            update_matrices_with_identity();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }
    }
}

void Ch6HdrLightingToneMapping::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::Checkbox("do tone mapping", &doToneMap);
}

void Ch6HdrBloom::init(){

    weights.resize(10);
    float sum = 0.f, sigma2 = 25.0f;

    // Compute and sum the weights
    weights[0] = gauss(0,sigma2);
    sum = weights[0];
    for(size_t ii = 1; ii < weights.size(); ii++ ) {
        weights[ii] = gauss(float(ii), sigma2);
        sum += 2 * weights[ii];
    }

    for(size_t ii = 0; ii < weights.size(); ii++ ) {
        weights[ii] /= sum;
    }


    update_screen_size();


}

void Ch6HdrBloom::update_screen_size(){

    int size = camera->screen()->size();
    texData.resize(size*3);

//    gl::FBO::unbind();
    {
        // Generate and bind the framebuffer
        hdrFBO.clean();
        hdrFBO.generate();
        hdrFBO.bind();

        // Create the depth buffer
        hdrDepthBuffer.clean();
        hdrDepthBuffer.generate();
        hdrDepthBuffer.bind();
        hdrDepthBuffer.set_data_storage(camera->screen()->width(),camera->screen()->height());

        // Create the  HDR texture object
        hdrRenderTexture.clean();
        hdrRenderTexture.init_hdr_render(camera->screen()->width(),camera->screen()->height(), 4);

        TextureOptions options;
        options.minFilter = TextureMinFilter::nearest;
        options.magFilter = TextureMagFilter::nearest;
        options.maxLevel = 0;
        hdrRenderTexture.set_texture_options(options);

        // Bind the texture to the FBO
        hdrFBO.attach_color0_texture(hdrRenderTexture);

        // Bind the depth buffer to the FBO
        hdrFBO.attach_depth_buffer(hdrDepthBuffer);        

        // set colors buffers to be drawn
        hdrFBO.set_draw_buffers({attachment::color0});

    }
    gl::FBO::unbind();

    {
        // Generate and bind the framebuffer
        blurFBO.clean();
        blurFBO.generate();
        blurFBO.bind();

        bloomBufWidth  = camera->screen()->width()/8;
        bloomBufHeight = camera->screen()->height()/8;

        // Create two texture objects to ping-pong for the bright-pass filter
        // and the two-pass blur
        blurTex1.clean();
        blurTex1.init_hdr_render(bloomBufWidth,bloomBufHeight, 3);

        blurTex2.clean();
        blurTex2.init_hdr_render(bloomBufWidth,bloomBufHeight, 3);

        // Bind tex1 to the FBO
        blurFBO.attach_color0_texture(blurTex1);

        GLenum drawBufs[] = {GL_COLOR_ATTACHMENT0};
        glNamedFramebufferDrawBuffers(blurFBO.id(), 1, drawBufs);
    }
    gl::FBO::unbind();
}

void Ch6HdrBloom::draw(tool::gl::Drawer *drawer){

    gl::TBO::unbind_textures(0, 3);

    if(auto shader = shadersM->get_ptr("ch6/hdr-bloom"); shader != nullptr){

        shader->use();

        shader->set_uniform("LumThresh", luminanceThreshold);
        shader->set_uniform("Exposure", exposure);
        shader->set_uniform("White", white);
        shader->set_uniform("Gamma", gamma);

        auto intense = Vec3f{1.0f,1.0f,1.0f};
        shader->set_uniform("Lights[0].L", intense );
        shader->set_uniform("Lights[1].L", intense );
        shader->set_uniform("Lights[2].L", intense );

        intense = Vec3f{0.2f,0.2f,0.2f};
        shader->set_uniform("Lights[0].La", intense );
        shader->set_uniform("Lights[1].La", intense );
        shader->set_uniform("Lights[2].La", intense );
        //        shader->set_uniform("DoToneMap", doToneMap);

        shader->set_uniform("Weight[0]", weights);
        shader->set_uniform("Light.L", Vec3f{1.0f,1.0f,1.0f});
        shader->set_uniform("Light.La", Vec3f{0.2f,0.2f,0.2f});
        shader->set_uniform("Light.Position", Pt4f{0.0f,0.0f,0.0f,1.0f});

        // pass 1
        {
            shader->set_uniform("Pass", 1);

            hdrFBO.bind();

            glClearColor(0.5f,0.5f,0.5f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);

            auto lightPos = Vec4f{0.0f, 4.0f, 2.5f, 1.0f};
            lightPos.x() = -7.0f;
            shader->set_uniform("Lights[0].Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});
            lightPos.x() = 0.0f;
            shader->set_uniform("Lights[1].Position", Pt4f{camera->view().multiply_point(lightPos.conv<double>()).conv<float>()});
            lightPos.x() = 7.0f;
            shader->set_uniform("Lights[2].Position", Pt4f{camera->view().multiply_point(lightPos.conv<double>()).conv<float>()});

            shader->set_uniform("Material.Kd", geo::Vec3f{0.9f, 0.3f, 0.2f});
            shader->set_uniform("Material.Ks", geo::Vec3f{1.0f, 1.0f, 1.0f});
            shader->set_uniform("Material.Ka", geo::Vec3f{0.2f, 0.2f, 0.2f});
            shader->set_uniform("Material.Shininess", 100.0f);

            // The backdrop plane
            model = Mat4d::rotate(Mat4d(true), Vec3d{1,0,0}, 90.);
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // The bottom plane
            model = Mat4d::translate(Mat4d(true), Vec3d{0,-5,0});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // Top plane
            model = Mat4d::translate(Mat4d(true), Vec3d{0,5,0});
            model = Mat4d::rotate(model, Vec3d{1,0,0}, 180.);
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // sphere
            shader->set_uniform("Material.Kd", geo::Vec3f{0.4f, 0.9f, 0.4f});

            model = Mat4d::translate(Mat4d(true), Vec3d{-3,-3,2.0});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("sphere-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // teapot
            shader->set_uniform("Material.Kd", geo::Vec3f{0.4f, 0.4f, 0.9f});

            model = Mat4d::translate(Mat4d(true), Vec3d{4,-5,1.5});
            model = Mat4d::rotate(model, Vec3d{1,0,0}, -90.);
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("teapot-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }

        // compute log average luminance
        {
            const int size = camera->screen()->size();
            hdrRenderTexture.bind();
//            gl::TBO::bind_textures({hdrRenderTexture.id(),0,0});
//            texData.resize(size*4);

//            glGetTextureImage(hdrRenderTexture.id(), 0, GL_RGB, GL_FLOAT, texData.size()/4, texData.data());

            glGetTextureImage(hdrRenderTexture.id(), 0, GL_RGB, GL_FLOAT, static_cast<GLsizei>(texData.size()*4), texData.data());
//            hdrRenderTexture.get_hdr_texture_data(texData);

            float sum = 0.0f;
            size_t count = 0;
            for( int i = 0; i < size; i++ ) {
                float lum = geo::dot(
                    geo::Pt3f{texData[i*3+0], texData[i*3+1], texData[i*3+2]},
                    geo::Pt3f{0.2126f, 0.7152f, 0.0722f} );
                if(texData[i*3+0] != 0){
                    count++;
                }
                if(texData[i*3+1] != 0){
                    count++;
                }

                if(texData[i*3+2] != 0){
                    count++;
                }
                sum += logf( lum + 0.00001f );
            }
            shader->set_uniform( "AveLum", expf( sum / size ) );
        }


        // pass 2
        {
            shader->set_uniform("Pass", 2);

            blurFBO.bind();
            blurFBO.attach_color0_texture(blurTex1);

            glViewport(0,0,bloomBufWidth, bloomBufHeight);
            glDisable(GL_DEPTH_TEST);
            glClearColor(0,0,0,0);
            glClear(GL_COLOR_BUFFER_BIT);

            update_matrices_with_identity();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

        }

        // pass 3
        {
            shader->set_uniform("Pass", 3);

            blurFBO.attach_color0_texture(blurTex2);

            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }

        // pass 4
        {
            shader->set_uniform("Pass", 4);

            blurFBO.attach_color0_texture(blurTex1);

            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }

        // pass 5
        {

            shader->set_uniform("Pass", 5);

            // Bind to the default framebuffer, this time we're going to
            // actually draw to the screen!
            gl::FBO::unbind();
            glClear(GL_COLOR_BUFFER_BIT);

            glViewport(0,0,camera->screen()->width(), camera->screen()->height());

            // In this pass, we're reading from tex1 (unit 1) and we want
            // linear sampling to get an extra blur
            linearSampler.bind(1);

            // Render the full-screen quad
            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // Revert to nearest sampling
            nearestSampler.bind(1);
        }
    }
}

void Ch6HdrBloom::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("luminance threshold", &luminanceThreshold, 0.f, 5.f, "ratio = %.3f");
    ImGui::SliderFloat("exposure", &exposure, 0.f, 1.f, "ratio = %.3f");
    ImGui::SliderFloat("white", &white, 0.f, 1.f, "ratio = %.3f");
}


void Ch6GaussianFilter::init(){
    update_screen_size();
}

void Ch6GaussianFilter::update_screen_size(){

    {
        // Generate and bind the framebuffer
        screenFBO.clean();
        screenFBO.generate();
        screenFBO.bind();

        // Create the texture object
        screenRenderTexture.clean();
        screenRenderTexture.init_render(camera->screen()->width(),camera->screen()->height());

        TextureOptions options;
        options.minFilter = TextureMinFilter::nearest;
        options.magFilter = TextureMagFilter::nearest;
        options.maxLevel = 0;
        screenRenderTexture.set_texture_options(options);

        // Create the depth buffer
        screenDepthBuffer.clean();
        screenDepthBuffer.generate();
        screenDepthBuffer.bind();
        screenDepthBuffer.set_data_storage(camera->screen()->width(),camera->screen()->height());

        // Bind the texture to the FBO
        screenFBO.attach_color0_texture(screenRenderTexture);

        // Bind the depth buffer to the FBO
        screenFBO.attach_depth_buffer(screenDepthBuffer);

        // set colors buffers to be drawn
        screenFBO.set_draw_buffers({
            attachment::color0
        });
    }

    {
        // Generate and bind the framebuffer
        intermediateFBO.clean();
        intermediateFBO.generate();
        intermediateFBO.bind();

        // Create the texture object
        intermediateRenderTexture.clean();
        intermediateRenderTexture.init_render(camera->screen()->width(),camera->screen()->height());

        TextureOptions options;
        options.minFilter = TextureMinFilter::nearest;
        options.magFilter = TextureMagFilter::nearest;
        options.maxLevel = 0;
        intermediateRenderTexture.set_texture_options(options);

        // Bind the texture to the FBO
        intermediateFBO.attach_color0_texture(intermediateRenderTexture);

        // set colors buffers to be drawn
        intermediateFBO.set_draw_buffers({
            attachment::color0
        });
    }

    // Unbind the framebuffer, and revert to default framebuffer
    gl::FBO::unbind();
}

void Ch6GaussianFilter::draw(tool::gl::Drawer *drawer){

    // gaussian weights
    std_v1<float> weights;
    weights.resize(5);
    float sum = 0.f;

    // Compute and sum the weights
    weights[0] = gauss(0,sigma2);
    sum = weights[0];
    for( int i = 1; i < 5; i++ ) {
        weights[i] = gauss(float(i), sigma2);
        sum += 2 * weights[i];
    }

    for(size_t ii = 0; ii < weights.size(); ii++ ) {
        weights[ii] /= sum;
    }

    if(auto shader = shadersM->get_ptr("ch6/gaussian-filter"); shader != nullptr){

        shader->use();
        shader->set_uniform("Weight[0]", weights);
        shader->set_uniform("Light.L", Vec3f{1.0f,1.0f,1.0f});
        shader->set_uniform("Light.La", Vec3f{0.2f,0.2f,0.2f});
        shader->set_uniform("Light.Position", Pt4f{0.0f,0.0f,0.0f,1.0f});

        // pass 1 : blinnPhong
        {
            screenFBO.bind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);

            shader->set_uniform("Pass", 1);

            // teapot
            model = Mat4f::transform({1.f,1.f,1.f},Vec3f{90.f,0,0},{0.0f,0.f,0.f}).conv<double>();
            update_matrices();

            shader->set_uniform("Material.Kd", Vec3f{0.9f, 0.9f, 0.9f});
            shader->set_uniform("Material.Ks", Vec3f{0.95f, 0.95f, 0.95f});
            shader->set_uniform("Material.Ka", Vec3f{0.1f, 0.1f, 0.1f});
            shader->set_uniform("Material.Shininess" , 100.0f);
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("teapot-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // plane
            model = Mat4f::transform({50.f,50.f,50.f},Vec3f{0,0,0},{0.0f,-0.75f,0.f}).conv<double>();
            update_matrices();

            shader->set_uniform("Material.Kd", Vec3f{0.4f, 0.4f, 0.4f});
            shader->set_uniform("Material.Ks", Vec3f{0.0f, 0.0f, 0.0f});
            shader->set_uniform("Material.Ka", Vec3f{0.1f, 0.1f, 0.1f});
            shader->set_uniform("Material.Shininess" , 1.0f);
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-10x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // torus
            model = Mat4f::transform({1.f,1.f,1.f},Vec3f{90,0,0},{1.0f,1.f,3.f}).conv<double>();
            update_matrices();

            shader->set_uniform("Material.Kd", Vec3f{0.9f, 0.5f, 0.2f});
            shader->set_uniform("Material.Ks", Vec3f{0.95f, 0.95f, 0.95f});
            shader->set_uniform("Material.Ka", Vec3f{0.1f, 0.1f, 0.1f});
            shader->set_uniform("Material.Shininess" , 100.0f);
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("torus-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }

        // pass 2
        {
            intermediateFBO.bind();
            screenRenderTexture.bind();

            glDisable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT);

            shader->set_uniform("Pass", 2);

            model       = Mat4d(true);
            Mat4d view  = Mat4d(true);
            Mat4d proj  = Mat4d(true);
            update_matrices(view,proj);
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }

        // pass 3
        {
            gl::FBO::unbind();
            intermediateRenderTexture.bind();
            glClear(GL_COLOR_BUFFER_BIT);

            shader->set_uniform("Pass", 3);

            update_matrices_with_identity();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

        }
    }
}

void Ch6GaussianFilter::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("Sigma2", &sigma2, 0.01f, 10.00f, "ratio = %.3f");
}

void Ch6EdgeDetectionFilter::init(){
    update_screen_size();
}

void Ch6EdgeDetectionFilter::update_screen_size(){

    // Generate and bind the framebuffer
    screenFBO.clean();
    screenFBO.generate();
    screenFBO.bind();

    // Create the texture object
    screenRenderTexture.clean();
    screenRenderTexture.init_render(camera->screen()->width(),camera->screen()->height());

    TextureOptions options;
    options.minFilter = TextureMinFilter::nearest;
    options.magFilter = TextureMagFilter::nearest;
    options.maxLevel = 0;
    screenRenderTexture.set_texture_options(options);

    // Create the depth buffer
    screenDepthBuffer.clean();
    screenDepthBuffer.generate();
    screenDepthBuffer.bind();
    screenDepthBuffer.set_data_storage(camera->screen()->width(),camera->screen()->height());

    // Bind the texture to the FBO
    screenFBO.attach_color0_texture(screenRenderTexture);

    // Bind the depth buffer to the FBO
    screenFBO.attach_depth_buffer(screenDepthBuffer);

    // set colors buffers to be drawn
    screenFBO.set_draw_buffers({
        attachment::color0
    });

    // Unbind the framebuffer, and revert to default framebuffer
    gl::FBO::unbind();
}

void Ch6EdgeDetectionFilter::draw(tool::gl::Drawer *drawer){

    if(auto shader = shadersM->get_ptr("ch6/edge-detection-filter"); shader != nullptr){

        shader->use();

        shader->set_uniform("EdgeThreshold", edgeThreshold);
        shader->set_uniform("Light.L", Vec3f{1.0f,1.0f,1.0f});
        shader->set_uniform("Light.La", Vec3f{0.2f,0.2f,0.2f});
        shader->set_uniform("Light.Position", Pt4f{0.0f,0.0f,0.0f,1.0f});

        // pass 1 : blinnphong
        {

            screenFBO.bind();
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            shader->set_uniform("Pass", 1);

            // teapot
            model = Mat4f::transform({1.f,1.f,1.f},Vec3f{90.f,0,0},{0.0f,0.f,0.f}).conv<double>();
            update_matrices();

            shader->set_uniform("Material.Kd", Vec3f{0.9f, 0.9f, 0.9f});
            shader->set_uniform("Material.Ks", Vec3f{0.95f, 0.95f, 0.95f});
            shader->set_uniform("Material.Ka", Vec3f{0.1f, 0.1f, 0.1f});
            shader->set_uniform("Material.Shininess" , 100.0f);
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("teapot-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // plane
            model = Mat4f::transform({50.f,50.f,50.f},Vec3f{0,0,0},{0.0f,-0.75f,0.f}).conv<double>();
            update_matrices();

            shader->set_uniform("Material.Kd", Vec3f{0.4f, 0.4f, 0.4f});
            shader->set_uniform("Material.Ks", Vec3f{0.0f, 0.0f, 0.0f});
            shader->set_uniform("Material.Ka", Vec3f{0.1f, 0.1f, 0.1f});
            shader->set_uniform("Material.Shininess" , 1.0f);

            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-10x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // torus
            model = Mat4f::transform({1.f,1.f,1.f},Vec3f{90,0,0},{1.0f,1.f,3.f}).conv<double>();
            update_matrices();

            shader->set_uniform("Material.Kd", Vec3f{0.9f, 0.5f, 0.2f});
            shader->set_uniform("Material.Ks", Vec3f{0.95f, 0.95f, 0.95f});
            shader->set_uniform("Material.Ka", Vec3f{0.1f, 0.1f, 0.1f});
            shader->set_uniform("Material.Shininess" , 100.0f);
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("torus-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }

        glFlush();

        // pass 2
        {
            gl::FBO::unbind();
            screenRenderTexture.bind();

            glDisable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT);

            shader->set_uniform("Pass", 2);

            update_matrices_with_identity();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }
    }
}

void Ch6EdgeDetectionFilter::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("edge threshold", &edgeThreshold, 0.005f, 0.30f, "ratio = %.3f");
}

void Ch5RenderToTexture::init(){

    // Create the texture object
    renderTexCh5RenderToTexture.init_render(512,512);

    TextureOptions options;
    options.minFilter = TextureMinFilter::linear;
    options.magFilter = TextureMagFilter::linear;
    renderTexCh5RenderToTexture.set_texture_options(options);

    update_screen_size();
}

void Ch5RenderToTexture::update_screen_size(){

    // Generate and bind the framebuffer
    fboCh5RenderToTexture.generate();
    fboCh5RenderToTexture.bind();

    // Create the depth buffer
    depthBufCh5RenterToTexture.generate();
    depthBufCh5RenterToTexture.bind();
    depthBufCh5RenterToTexture.set_data_storage();

    // Bind the texture to the FBO
    fboCh5RenderToTexture.attach_color0_texture(renderTexCh5RenderToTexture);

    // Bind the depth buffer to the FBO
    fboCh5RenderToTexture.attach_depth_buffer(depthBufCh5RenterToTexture);

    // set colors buffers to be drawn
    fboCh5RenderToTexture.set_draw_buffers({
        attachment::color0
    });

    // Unbind the framebuffer, and revert to default framebuffer
    gl::FBO::unbind();
}

void Ch5RenderToTexture::draw(tool::gl::Drawer *drawer){

    if(auto shader = shadersM->get_ptr("ch5/render-to-texture"); shader != nullptr){

        shader->use();

        { // pass 0

            fboCh5RenderToTexture.bind();
            glViewport(0,0,512,512);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            Mat4d view = Mat4f::LookAt(Vec3f{0.0f,0.f,-2.f}, Vec3f{0.0f,0.0f,1.0f}, Vec3f{0.0f,1.0f,0.0f}).conv<double>();
            Mat4d proj = Mat4f::Perspective(60.0f, 1.0f, 0.3f, 10000.0f).conv<double>();
            model = Mat4f::transform({10.f,10.f,10.f}, Vec3f{angle,angle,angle},{0.0f,0.f,3.f}).conv<double>();
            update_matrices(view, proj);

            shader->set_uniform("Light.L", Vec3f{1.0f,1.0f,1.0f});
            shader->set_uniform("Light.La", Vec3f{0.15f,0.15f,0.15f});
            shader->set_uniform("Light.Position", Pt4f{0,0,0,1});
            shader->set_uniform("Material.Ks", Vec3f{0.95f, 0.95f, 0.95f});
            shader->set_uniform("Material.Shininess" , 100.0f);
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("rabbit-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            glFlush();
        }

        { // pass 1

            gl::FBO::unbind();
            renderTexCh5RenderToTexture.bind();
            glViewport(0,0, camera->screen()->width(), camera->screen()->height());

            shader->set_uniform("Light.Position", Vec4f{0.0f,0.0f,0.0f,1.0f} );
            shader->set_uniform("Material.Ks", Vec3f{0.0f, 0.0f, 0.0f});
            shader->set_uniform("Material.Shininess", 1.0f);

            model = Mat4f::transform({1.f,1.f,1.f},Vec3f{0.f,0.f,0.f},{0.f,0.f,5.f}).conv<double>();
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("cube-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }
    }
}

void Ch5RenderToTexture::update(float elapsedSeconds){
    Sample::update(elapsedSeconds);
    angle = (elapsedSeconds * 10.f);
}

void Ch5SamplerObject::init(){

    // sampler objects
    TextureOptions linearOptions;
    linearOptions.magFilter = TextureMagFilter::linear;
    linearOptions.minFilter = TextureMinFilter::linear;

    TextureOptions nearestOptions;
    nearestOptions.magFilter = TextureMagFilter::nearest;
    nearestOptions.minFilter = TextureMinFilter::nearest;

    auto samplers = gl::Sampler::generate({linearOptions, nearestOptions});
    linearSampler   = gl::Sampler(linearOptions);
    nearestSampler  = gl::Sampler(nearestOptions);
}

void Ch5SamplerObject::draw(tool::gl::Drawer *drawer){


    if(auto shader = shadersM->get_ptr("ch5/sampler-objects"); shader != nullptr){

        shader->use();
        shader->set_uniform("Light.L", Vec3f{1.0f,1.0f, 1.0f});
        shader->set_uniform("Light.La", Vec3f{0.2f,0.2f,0.2f});
        shader->set_uniform("Light.Position", Vec4f{0.0f,20.0f,0.0f,1.0f} );
        shader->set_uniform("Material.Ks", Vec3f{0.95f, 0.95f, 0.95f});
        shader->set_uniform("Material.Shininess", 100.0f);

        model = Mat4f::transform({1.f,1.f,1.f},Vec3f{0.f,0.f,0.f},{-5.01f,0.f,0.f}).conv<double>();
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);

        nearestSampler.bind(0);
        if(auto drawer = drawersM->get_drawer_ptr("grid-floor-drawer"); drawer != nullptr){
            drawer->draw(shader);
        }

        model = Mat4f::transform({1.f,1.f,1.f},Vec3f{0.f,0.f,0.f},{5.01f,0.f,0.f}).conv<double>();
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);

        linearSampler.bind(0);
        if(auto drawer = drawersM->get_drawer_ptr("grid-floor-drawer"); drawer != nullptr){
            drawer->draw(shader);
        }

        gl::Sampler::unbind();
    }
}

void Ch5DiffuseImageBasedLighting::draw(tool::gl::Drawer *drawer){

    if(auto shader = shadersM->get_ptr("ch5/diffuse-image-based-lighting"); shader != nullptr){

        shader->use();
        gl::TBO::bind_textures({texturesM->id("grace-diffuse"),texturesM->id("spot_texture")});

        model = Mat4f::transform({1.f,1.f,1.f},Vec3f{0.f,0.f,0.f},{0.0f,0.f,3.f}).conv<double>();
        update_matrices();

        shader->set_uniform("gamma",  gamma);
        shader->set_uniform("CamPos", camera->position().conv<float>());
        shader->set_uniform("ModelMatrix",  model.conv<float>());
        shader->set_camera_matrices_uniforms(camM);

        if(auto drawer = drawersM->get_drawer_ptr("notext-spot-drawer"); drawer != nullptr){
            drawer->draw(shader);
        }
    }
}

void Ch5DiffuseImageBasedLighting::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("reflect factor", &reflectFactor, 0.0f, 10.00f, "ratio = %.3f");
    ImGui::SliderFloat("gamma", &gamma, 0.0f, 10.00f, "ratio = %.3f");
}

void Ch5ProjectTexture::init(){
    shader = shadersM->get_ptr("ch5/projected-texture");
    materialUBO.generate();
    materialUBO.set_data_space_from_shader(shader);
}

void Ch5ProjectTexture::draw(tool::gl::Drawer *drawer){

    materialUBO.bind(1);

    Vec3f projPos{2.0f,5.0f,5.0f};
    Vec3f projAt{-2.0f,-4.0f,0.0f};
    Vec3f projUp{0.0f,1.0f,0.0f};
    Mat4f projView = Mat4f::LookAt(projPos, projAt, projUp);
    Mat4f projProj = Mat4f::Perspective(30.0f, 1.0f, 0.2f, 1000.0f);
    Mat4f bias     = Mat4f::transform({0.5f,0.5f,0.5f},Vec3f{0.f,0.f,0.f},{0.5f,0.5f,0.5f});

    auto mv2 = bias.conv<double>() * projView.conv<double>();
    auto mvp2 =  mv2 * projProj.conv<double>();

    auto projectorMat = mvp2.conv<float>();//bias * projProj * projView;
    shader->set_uniform("ProjectorMatrix", projectorMat);

    model = Mat4d::transform({0.5,0.5,0.5},Vec3d{0.,0.,0},{0.,-2.,0.});
    update_matrices();

    shader->use();
    shader->set_uniform("Light.L",  Vec3f{0.8f,0.8f,0.8f});
    shader->set_uniform("Light.La", Vec3f{0.2f,0.2f,0.2f});
    shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos1.conv<double>()).conv<float>()});

    mInfo.Ka = {0.5f, 0.5f, 0.5f};
    mInfo.Kd = {0.5f, 0.5f, 0.5f};
    mInfo.Ks = {0.8f, 0.8f, 0.8f};
    mInfo.Shininess = 10.0f;
    materialUBO.update(mInfo);

    shader->set_model_matrix(model);
    shader->set_camera_matrices_uniforms(camM);

    texturesM->get_tbo("flower-projected")->bind();
    if(auto drawer = drawersM->get_drawer_ptr("notext-plane-10x10-drawer"); drawer != nullptr){
        drawer->draw(shader);
    }

    model = Mat4d::transform({5.,5.,5.},Vec3f{rx,ry,rz}.conv<double>(),{-5.,-2.,15.});
    update_matrices();

    shader->set_uniform("ModelMatrix",       model.conv<float>());
    shader->set_camera_matrices_uniforms(camM);

    if(auto drawer = drawersM->get_drawer_ptr("torus-drawer"); drawer != nullptr){
        drawer->draw(shader);
    }

}






void Ch6Deferred::init(){

    const std_v1<geo::Pt3f> colors = {
        Vec3f{1.f,0.f,0.f},
        Vec3f{0.f,1.f,0.f},
        Vec3f{0.f,0.f,1.f},
        Vec3f{0.f,1.f,0.f},
        Vec3f{0.f,1.f,1.f},
        Vec3f{1.f,0.f,1.f},
        Vec3f{1.f,1.f,0.f},
        Vec3f{1.f,1.f,1.f}
    };

    for(size_t ii = 0; ii < 25; ++ii){
        lightsColors.emplace_back(colors[ii%8]);
    }

    update_screen_size();
}

void Ch6Deferred::update_screen_size(){

    const auto width = camera->screen()->width();
    const auto height = camera->screen()->height();

    // Generate and bind the framebuffer
    deferredFBO.clean();
    deferredFBO.generate();
    deferredFBO.bind();

    // Create the depth buffer
    depthBuf.clean();
    depthBuf.generate();
    depthBuf.bind();
    depthBuf.set_data_storage(width, height);

    // Create the textures for position, normal and color
    posTex.clean();
    posTex.init_position(width, height);
    normTex.clean();
    normTex.init_position(width, height);
    diffuseColorTex.clean();
    diffuseColorTex.init_color(width, height);
    ambiantColorTex.clean();
    ambiantColorTex.init_color(width, height);
    specularColorTex.clean();
    specularColorTex.init_color(width, height);

    // Attach the textures to the framebuffer
    deferredFBO.attach_depth_buffer(depthBuf);
    deferredFBO.attach_colors_textures({
        &posTex,&normTex,
        &diffuseColorTex,&ambiantColorTex,&specularColorTex
    });

    // set colors buffers to be drawn
    deferredFBO.set_draw_buffers({
        attachment::none,
        attachment::color0,
        attachment::color1,
        attachment::color2,
        attachment::color3,
        attachment::color4
    });

    gl::FBO::unbind();
}

void Ch6Deferred::draw(tool::gl::Drawer *drawer){

    gl::TBO::unbind_textures(0, 3);


    if(auto shader = shadersM->get_ptr("ch6/deferred"); shader != nullptr){

        shader->use();

        // pass 1
        {
            shader->set_uniform("Pass", 1);

            deferredFBO.bind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);


            shader->set_uniform("Material.Ks", geo::Vec3f{0.0f, 0.0f, 0.0f});
            shader->set_uniform("Material.Ka", geo::Vec3f{0.1f, 0.1f, 0.1f});
            shader->set_uniform("Material.Shininess", 1.0f);

            shader->set_uniform("LightCount", 25);
            int count = 0;
            for(int ii = 0; ii < 5; ++ii){
                for(int jj = 0; jj < 5; ++jj){

                    std::string lightName = "Light[" + std::to_string(count) + "].";
                    shader->set_uniform((lightName + "L").c_str(),  Vec3f{0.8f,0.8f,0.8f});
                    shader->set_uniform((lightName + "La").c_str(), lightsColors[count]);

                    shader->set_uniform("Material.Kd", lightsColors[count]);

                    auto lightP = Pt4f{-10.f+ii*4,2,-10.f+jj*4, 1.f};
                    shader->set_uniform((lightName + "Position").c_str(), Pt4f{camera->view().multiply_point(lightP.conv<double>()).conv<float>()});
                    ++count;

                    model = Mat4d::transform({0.1,0.1,0.1},Vec3d{0.,0.,0.},lightP.xyz().conv<double>());
                    update_matrices();
                    shader->set_camera_matrices_uniforms(camM);

                    if(auto drawer = drawersM->get_drawer_ptr("sphere-drawer"); drawer != nullptr){
                        drawer->draw(shader);
                    }
                }
            }



            // teapot
            shader->set_uniform("Material.Kd", geo::Vec3f{0.5f, 0.5f, 0.5f});
            shader->set_uniform("Material.Ks", geo::Vec3f{1.0f, 1.0f, 1.0f});
            shader->set_uniform("Material.Ka", geo::Vec3f{0.5f, 0.5f, 0.5f});
            shader->set_uniform("Material.Shininess", 100.0f);

            for(int ii = 0; ii < 10; ++ii){
                for(int jj = 0; jj < 10; ++jj){
                    model = Mat4d::transform({0.3,0.3,0.3},Vec3d{90.,0.,0.},{-15.f+ii*3,0,-15.f+jj*3});
                    update_matrices();
                    shader->set_camera_matrices_uniforms(camM);

                    if(auto drawer = drawersM->get_drawer_ptr("teapot-drawer"); drawer != nullptr){
                        drawer->draw(shader);
                    }
                }
            }

            // plane
            shader->set_uniform("Material.Kd", geo::Vec3f{0.1f, 0.9f, 0.2f});
            shader->set_uniform("Material.Ks", geo::Vec3f{1.0f, 1.0f, 1.0f});
            shader->set_uniform("Material.Ka", geo::Vec3f{0.2f, 0.2f, 0.2f});
            shader->set_uniform("Material.Shininess", 1.0f);
            model = Mat4d::transform({5.,5.,5.},Vec3d{0.,0.,0.},{0.0f,-0.75f,0.0f});
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-10x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // torus
            shader->set_uniform("Material.Kd", geo::Vec3f{0.1f, 0.3f, 0.9f});
            shader->set_uniform("Material.Ks", geo::Vec3f{0.0f, 0.0f, 0.0f});
            shader->set_uniform("Material.Ka", geo::Vec3f{0.2f, 0.2f, 0.2f});
            shader->set_uniform("Material.Shininess", 1.0f);
            model = Mat4f::transform({1.f,1.f,1.f},Vec3f{-90,0,0},{1.0f,1.0f,3.0f}).conv<double>();
            update_matrices();

            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("torus-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

//            glFinish(); // ?
        }

        // pass 2
        {
            shader->set_uniform("Pass", 2);

            gl::FBO::unbind();
            gl::TBO::bind_textures({posTex.id(), normTex.id(),
                diffuseColorTex.id(),ambiantColorTex.id(),specularColorTex.id()
            });

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDisable(GL_DEPTH_TEST);
            draw_screen_quad(shader);
        }
    }
}




void Ch6SSAO::init(){

    std::mt19937 generator;
    std::uniform_real_distribution<float> distr01(0.0f, 1.0f);

    std::random_device rd;
    generator.seed(rd());

    auto uniformCircle = [&](){
        geo::Vec3f result;
        float x = distr01(generator);
        result.x() = std::cos(tool::two_PI<float> * x);
        result.y() = std::sin(tool::two_PI<float> * x);
        return result;
    };

    auto uniformHemisphere = [&](){
        geo::Vec3f result;
        float x1 = distr01(generator);
        float x2 = distr01(generator);
        float s = sqrt(1.0f - x1 * x1);
        result.x() = std::cos(tool::two_PI<float> * x2) *s;
        result.y() = std::sin(tool::two_PI<float> * x2) *s;
        result.z() = x1;
        return result;
    };

    int size = 4;
    std::vector<GLfloat> randDirections(3 * size * size);
    for (int i = 0; i < size * size; i++) {
        geo::Vec3f v = uniformCircle();
        randDirections[i * 3 + 0] = v.x();
        randDirections[i * 3 + 1] = v.y();
        randDirections[i * 3 + 2] = v.z();
        std::cout << i << " " << v << "\n";
    }

    gl::TextureOptions options;
    options.minFilter   = TextureMinFilter::nearest;
    options.magFilter   = TextureMagFilter::nearest;
    options.maxLevel    = 0;
    randRotationTex.load_data(randDirections.data(), size, size, 3, true, options);

    int kernSize = 64;
    kern.resize(kernSize);
    for (int i = 0; i < kernSize; i++) {
        geo::Vec3f v = uniformHemisphere();
        float scale = ((float)(i * i)) / (kernSize * kernSize);
        // v *= glm::mix(0.1f, 1.0f, scale);
        //  x * (1.0 - a) + y * a       
        v *= 0.1f * (1.0 - scale) + 1.0f * scale;
        kern[i] = {v.x(), v.y(), v.z()};
    }

//    std_v1<geo::Pt3f> colors;
//    colors.resize(kernSize);
//    std::fill(colors.begin(), colors.end(), geo::Pt3f{1,0,0});
//    tool::io::save_cloud("./kernel.obj", kern.data(), colors.data(), kern.size());

    update_screen_size();
}

void Ch6SSAO::update_screen_size(){

    const auto width  = camera->screen()->width();
    const auto height = camera->screen()->height();

    // Generate and bind the framebuffer
    deferredFBO.clean();
    deferredFBO.generate();
    deferredFBO.bind();

    // Create the depth buffer
    depthBuf.clean();
    depthBuf.generate();
    depthBuf.bind();
    depthBuf.set_data_storage(width, height);

    // Create the textures for position, normal and color
    posTex.clean();
    posTex.init_position(width, height);
    normTex.clean();
    normTex.init_position(width, height);
    colorTex.clean();
    colorTex.init_color(width, height);
    aoTex[0].clean();
    aoTex[0].init_ao(width, height);
    aoTex[1].clean();
    aoTex[1].init_ao(width, height);

    // Attach the textures to the framebuffer
    deferredFBO.attach_depth_buffer(depthBuf);
    deferredFBO.attach_colors_textures({
        &posTex,
        &normTex,
        &colorTex
    });

    // set colors buffers to be drawn
    deferredFBO.set_draw_buffers({
        attachment::none,
        attachment::color0,
        attachment::color1,
        attachment::color2,
        attachment::none
    });

    // Generate and bind the framebuffer
    ssaoFBO.clean();
    ssaoFBO.generate();
    ssaoFBO.bind();

    // Attach the textures to the framebuffer
    ssaoFBO.attach_colors_textures({
        &aoTex[0]
    });

    // set colors buffers to be drawn
    ssaoFBO.set_draw_buffers({
        attachment::none,
        attachment::none,
        attachment::none,
        attachment::none,
        attachment::color0
    });

    gl::FBO::unbind();
}

void Ch6SSAO::draw(tool::gl::Drawer *drawer){


    if(auto shader = shadersM->get_ptr("ch6/ssao"); shader != nullptr){

        shader->use();

        shader->set_uniform("randScale", geo::Vec2f{800.f/factorScale, 600.f/factorScale});
        shader->set_uniform("SampleKernel[0]", kern);
        shader->set_uniform("ProjectionMatrix", camera->projection().conv<float>());
        shader->set_uniform("Radius", radius);

        // pass 1 : Render to G-Buffers
        shader->set_uniform("Pass", 1);
        deferredFBO.bind();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        {
            shader->set_uniform("Light.L", geo::Vec3f{0.3f, 0.3f, 0.3f});
            shader->set_uniform("Light.La", geo::Vec3f{0.5f, 0.5f, 0.5f});
            shader->set_uniform("Light.Position", Pt4f{3.0f, 3.0f, 1.5f, 1.0f});
//            shader->set_uniform("Light.Position", Pt4f{camera->view().multiply_point(mobileLightPos.conv<double>()).conv<float>()});


            // floor
            gl::TBO::bind_textures({texturesM->id("hardwood_diffuse")}, 5);

            shader->set_uniform("Material.UseTex", true);

            model = Mat4d(true);
            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // walls
            gl::TBO::bind_textures({texturesM->id("brick")}, 5);

            model = Mat4d(true);
            model = Mat4d::translate(model, {0,0,-2});
            model = Mat4d::rotate(model, {1,0,0},90);

            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            model = Mat4d(true);
            model = Mat4d::translate(model, {-2,0,0});
            model = Mat4d::rotate(model, {0,1,0},90);
            model = Mat4d::rotate(model, {1,0,0},90);

            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("notext-plane-20x10-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }

            // dragon
            shader->set_uniform("Material.UseTex", false);
            shader->set_uniform("Material.Kd", geo::Vec3f{0.9f, 0.5f, 0.2f});

//            model = Mat4d::transform(Vec3d{2,2,2}, Vec3d{rx,ry,rz}, Vec3d{x,y,z});

            model = Mat4d(true);
            model = Mat4d::rotate(model, Vec3d{0,1,0}, 135.);
            model = Mat4d::scale(model, Vec3d{2,2,2});
            model = Mat4d::translate(model, Vec3d{0,0.282958,0});

            update_matrices();
            shader->set_camera_matrices_uniforms(camM);

            if(auto drawer = drawersM->get_drawer_ptr("dragon-drawer"); drawer != nullptr){
                drawer->draw(shader);
            }
        }

        // pass 2 : SSAO
        shader->set_uniform("Pass", 2);
        ssaoFBO.bind();
        ssaoFBO.attach_color0_texture(aoTex[0]);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);
        {
            gl::TBO::bind_textures({
               posTex.id(),
               normTex.id(),
               colorTex.id()//,
               //aoTex[0].id(),
               //randRotationTex.id()
               },
               0
            );

            gl::TBO::bind_textures({randRotationTex.id()}, 4);
            draw_screen_quad(shader);
        }

        // pass 3 : Blur
        // Read from aoTex[0], write to aoTex[1]
        shader->set_uniform("Pass", 3);
        ssaoFBO.attach_color0_texture(aoTex[1]);
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);
        {
            gl::TBO::bind_textures({aoTex[0].id() }, 3);
            draw_screen_quad(shader);
        }

        // pass 4 : Lighting
        // Read from aoTex[1] (blurred)
        shader->set_uniform("Pass", 4);
        gl::FBO::unbind();
        glClear(GL_COLOR_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);
        {
            gl::TBO::bind_textures({aoTex[1].id() }, 3);

            draw_screen_quad(shader);
        }
    }
}

void Ch6SSAO::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("radius", &radius, 0.01f, 10.f, "ratio = %.3f");
    ImGui::SliderFloat("factor scale", &factorScale, 0.5f, 16.f, "ratio = %.3f");
}








void Ch6OIT::init(){


    shader = shadersM->get_ptr("ch6/oit");

    rotSpeed = tool::PI<float> / 8.0f;
    angle = 210.f;
    tPrev = 0.f;

    glEnable(GL_DEPTH_TEST);

    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    pass1Index = glGetSubroutineIndex( shader->id(), GL_FRAGMENT_SHADER, "pass1");
    pass2Index = glGetSubroutineIndex( shader->id(), GL_FRAGMENT_SHADER, "pass2");

    update_screen_size();
}

void Ch6OIT::update_screen_size(){

    const auto width  = camera->screen()->width();
    const auto height = camera->screen()->height();

    GLuint maxNodes = 20 * width * height;
    GLint nodeSize  = 5 * sizeof(GLfloat) + sizeof(GLuint); // The size of a linked list node

    // Our atomic counter
    counterBuffer.clean();
    counterBuffer.generate();
    counterBuffer.set_data_storage(sizeof(GLuint), GL_DYNAMIC_STORAGE_BIT);

    // The buffer for the head pointers, as an image texture
    headPtrTexture.clean();
    headPtrTexture.init_image_32ui(width, height, 1);

    // The buffer of linked lists
    linkedListBuffer.clean();
    linkedListBuffer.generate();
    linkedListBuffer.set_data_storage(maxNodes * nodeSize);
    shader->set_uniform("MaxNodes", maxNodes);

    std::vector<GLuint> headPtrClearBuf(width*height, 0xffffffff);
    clearBuffer.clean();
    clearBuffer.generate();
    clearBuffer.set_data_storage(headPtrClearBuf.size() * sizeof(GLuint), gl::UintData{headPtrClearBuf.data()});

    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
}

void Ch6OIT::draw(tool::gl::Drawer *drawer){

    const auto width  = camera->screen()->width();
    const auto height = camera->screen()->height();

    headPtrTexture.bind_image(0, 0, GL_FALSE, 0, GL_READ_WRITE);
    linkedListBuffer.bind_to_index(0);
    counterBuffer.bind_to_index(0);

    // clear buffers
    {        
        // updates a subset of a buffer object's data store
        GLuint zero = 0;
        counterBuffer.update_data(&zero, sizeof(GLuint));

        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, clearBuffer.id());
        headPtrTexture.update_image_32ui(nullptr, width, height, 0, 0);
    }

    // pass 1
    glEnable(GL_DEPTH_TEST);
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDepthMask( GL_FALSE );

    glUseProgram(shader->id());
    glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &pass1Index);
    {

        auto cube = drawersM->get_drawer_ptr("cube-drawer");
        auto sphere = drawersM->get_drawer_ptr("sphere-drawer");

        // draw scene
        shader->set_uniform("LightPosition", geo::Vec4f(0,0,0,1));
        shader->set_uniform("LightIntensity", geo::Vec3f(0.9f,0.9f,0.9f));
        shader->set_uniform("Kd", geo::Vec4f(0.2f, 0.2f, 0.9f, 0.55f));

        double size = 0.45;
        for( int i = 0; i <= 6; i++ ){
            for( int j = 0; j <= 6; j++ ){
                for( int k = 0; k <= 6; k++ ) {
                    if( (i + j + k) % 2 == 0 ) {

                        model = Mat4d::translate(Mat4d(true), Vec3d{i-3.0, j-3.0, k-3.0});
                        model = Mat4d::scale(model, Vec3d{size,size,size});

                        update_matrices();
                        shader->set_camera_matrices_uniforms(camM);

                        sphere->draw();
                    }
                }
            }
        }

        shader->set_uniform("Kd", geo::Vec4f(0.9f, 0.2f, 0.2f, 0.4f));
        size = 2.0;
        double pos = 1.75;
        model = Mat4d::translate(Mat4d(true), Vec3d{-pos, -pos, pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        model = Mat4d::translate(Mat4d(true), Vec3d{-pos, -pos, -pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        model = Mat4d::translate(Mat4d(true), Vec3d{-pos, pos, pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        model = Mat4d::translate(Mat4d(true), Vec3d{-pos, pos, -pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        model = Mat4d::translate(Mat4d(true), Vec3d{pos, pos, pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        model = Mat4d::translate(Mat4d(true), Vec3d{pos, pos, -pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        model = Mat4d::translate(Mat4d(true), Vec3d{pos, -pos, pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        model = Mat4d::translate(Mat4d(true), Vec3d{pos, -pos, -pos});
        model = Mat4d::scale(model, Vec3d{size,size,size});
        update_matrices();
        shader->set_camera_matrices_uniforms(camM);
        cube->draw();
        glFinish();
    }

    //  glFlush ensures that previous OpenGL commands must complete in finite time
    glFlush();

    // pass 2
    glMemoryBarrier( GL_SHADER_STORAGE_BARRIER_BIT );
    glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &pass2Index);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    {
        draw_screen_quad(shader);
    }

    glDepthMask( GL_TRUE );
}


void Ch7BezCurve::init(){

    std_v1<geo::Pt2f> patch ={
        {-1.0f, -1.0f},
        {-0.5f, 1.0f},
        {0.5f, -1.0f},
        {1.0f, 1.0f}
    };

    bezPoints = std::make_unique<gl::CloudPointsDrawer>();
    bezPoints->init(&patch);

    prog = shadersM->get_ptr("ch7/bezcurve");    
    prog->set_uniform("NumStrips", 1);
    prog->set_uniform("LineColor", geo::Vec4f(1.0f,0.0f,0.0f,1.0f));

    solidProg = shadersM->get_ptr("ch7/solid");
    solidProg->set_uniform("Color", geo::Vec4f(0.5f,1.0f,1.0f,1.0f));

    int maxVerts;
    glGetIntegerv(GL_MAX_PATCH_VERTICES, &maxVerts);
}

void Ch7BezCurve::draw(tool::gl::Drawer *drawer){

    glEnable(GL_DEPTH_TEST);
    glPointSize(10.0f);

    // Set the number of vertices per patch.  IMPORTANT!!
    glPatchParameteri( GL_PATCH_VERTICES, 4);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    model = geo::Mat4d(true);
    update_matrices();

    // Draw the curve
    prog->use();
    prog->set_uniform("NumSegments", numSegments);
    prog->set_camera_matrices_uniforms(camM);
    bezPoints->draw_patches();

    // Draw the control points
    solidProg->use();
    solidProg->set_camera_matrices_uniforms(camM);
    bezPoints->draw();

    glFinish();
}

void Ch7BezCurve::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderInt("bezcurve num segments", &numSegments, 1, 200, "ratio = %.3f");
}

void Ch7ShadeWire::init(){

    shader = shadersM->get_ptr("ch7/shadewire");    
    shader->set_uniform("Line.Color", geo::Vec4f(0.05f,0.0f,0.05f,1.0f));
    shader->set_uniform("Material.Kd", geo::Vec3f(0.7f, 0.7f, 0.7f));
    shader->set_uniform("Material.Ka", geo::Vec3f(0.2f, 0.2f, 0.2f));
    shader->set_uniform("Light.Intensity", geo::Vec3f(1.0f, 1.0f, 1.0f));
    shader->set_uniform("Material.Ks", geo::Vec3f(0.8f, 0.8f, 0.8f));
    shader->set_uniform("Material.Shininess", 100.0f);

    update_screen_size();
}

void Ch7ShadeWire::update_screen_size(){

    const auto width  = camera->screen()->width();
    const auto height = camera->screen()->height();

    float w2 = width / 2.0f;
    float h2 = height / 2.0f;
    Mat4f viewport = Mat4f{
        w2,0.0f,0.0f,0.0f,
        0.0f,h2,0.0f,0.0f,
        0.0f,0.0f,1.0f,0.0f,
        w2+0, h2+0, 0.0f, 1.0f
    };
    shader->set_uniform("ViewportMatrix", viewport);
}

void Ch7ShadeWire::draw(tool::gl::Drawer *drawer){

    shader->use();
    shader->set_uniform("Line.Width", lineWidth);
    shader->set_uniform("Light.Position", mobileLightPos1);

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    model = Mat4d(true);
    update_matrices();
    shader->set_camera_matrices_uniforms(camM);
    shader->set_uniform("ProjectionMatrix", camera->projection().conv<float>());    

    auto ogre = drawersM->get_drawer_ptr("ogre-drawer");
    ogre->draw();

    glFinish();
}

void Ch7ShadeWire::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("shadewire line width", &lineWidth, 0.01f, 10.f, "ratio = %.3f");
}

void Ch7ScenePointSprite::init(){

    shader = shadersM->get_ptr("ch7/pointsprite");
    pointsSprites = std::make_unique<gl::CloudPointsDrawer>();
}

void Ch7ScenePointSprite::draw(tool::gl::Drawer *drawer){

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<float> dist(0.f, 10000.f);
    std_v1<geo::Pt3f> locations;
    locations.reserve(numSprites);
    const auto max = dist.max();
    for(int ii = 0; ii < numSprites; ++ii){
        locations.emplace_back(geo::Pt3f{
            (dist(e2)/max * 2.f) - 1.0f,
            (dist(e2)/max * 2.f) - 1.0f,
            (dist(e2)/max * 2.f) - 1.0f
        });
    }
    pointsSprites->init(&locations);

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shader->use();
    shader->set_uniform("Size2", sizeSprite);
    gl::TBO::bind_textures({texturesM->id("flower")});

    model = Mat4d(true);
    update_matrices();
    shader->set_camera_matrices_uniforms(camM);
    shader->set_uniform("ProjectionMatrix", camera->projection().conv<float>());

    pointsSprites->draw();
}

void Ch7ScenePointSprite::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderInt("num sprites", &numSprites, 1, 1000, "ratio = %.3f");
    ImGui::SliderFloat("size sprites", &sizeSprite, 0.01f, 10.f, "ratio = %.3f");
}

void Ch7Silhouette::init(){

    shader = shadersM->get_ptr("ch7/silhouette");

    shader->set_uniform("EdgeWidth", 0.015f);
    shader->set_uniform("PctExtend", 0.25f);
    shader->set_uniform("LineColor", geo::Pt4f(0.05f,0.0f,0.05f,1.0f));
    shader->set_uniform("Material.Kd", geo::Pt3f(0.7f, 0.5f, 0.2f));
    shader->set_uniform("Material.Ka", geo::Pt3f(0.2f, 0.2f, 0.2f));
    shader->set_uniform("Light.Intensity", geo::Pt3f(1.0f, 1.0f, 1.0f));

}

void Ch7Silhouette::draw(tool::gl::Drawer *drawer){

    shader->use();
    shader->set_uniform("Light.Position", mobileLightPos1);// geo::Pt4f(0.0f,0.0f,0.0f,1.0f));

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    model = Mat4d(true);
    update_matrices();
    shader->set_camera_matrices_uniforms(camM);

    // auto ogre = drawersM->get_drawer_ptr("ogre-drawer");
    // TODO: convert mesh to agency mode
//    ogre->draw_adjacency();

    glFinish();
}

void Ch8ShadowMap::init(){

    solidP  = shadersM->get_ptr("ch8/solid");
    shadowP = shadersM->get_ptr("ch8/shadowmap");

    frustumD     = dynamic_cast<gl::FrustumDrawer*>(drawersM->get_drawer_ptr("frustum-drawer"));
    lightFrustum = dynamic_cast<gl::Frustum*>(frustumD->object());

    // Set up the framebuffer object
    {
        // Generate and bind the framebuffer
        shadowFBO.clean();
        shadowFBO.generate();
        shadowFBO.bind();

        // Create the depth buffer
        shadowTexture.clean();
        shadowTexture.debug_generate();

        glTextureStorage2D (
            shadowTexture.id(),    // GLuint texture
            1,                      // GLsizei levels
            GL_DEPTH_COMPONENT24,   // GLenum internalformat
            shadowMapWidth,         // GLsizei width
            shadowMapHeight         // GLsizei height
        );

        TextureOptions options;
        options.magFilter = TextureMagFilter::nearest;
        options.minFilter = TextureMinFilter::nearest;
        options.wrapS     = TextureWrapMode::clamp_to_border;
        options.wrapT     = TextureWrapMode::clamp_to_border;
        options.borderColor = {1.0f, 0.0f,0.0f,0.0f };
        shadowTexture.set_texture_options(options);
        glTextureParameteri(shadowTexture.id(), GL_TEXTURE_COMPARE_MODE,  GL_COMPARE_REF_TO_TEXTURE);
        glTextureParameteri(shadowTexture.id(), GL_TEXTURE_COMPARE_FUNC, GL_LESS);

        shadowFBO.attach_depth_texture(shadowTexture);

        // set colors buffers to be drawn
        shadowFBO.set_draw_buffers({attachment::none});

        gl::FBO::unbind();
    }

    // ##########

    pass1Index = glGetSubroutineIndex( shadowP->id(), GL_FRAGMENT_SHADER, "recordDepth");
    pass2Index = glGetSubroutineIndex( shadowP->id(), GL_FRAGMENT_SHADER, "shadeWithShadow");

    shadowBias = Mat4f{
        0.5f,0.0f,0.0f,0.0f,
        0.0f,0.5f,0.0f,0.0f,
        0.0f,0.0f,0.5f,0.0f,
        0.5f,0.5f,0.5f,1.0f
    };

    shadowP->set_uniform("Light.Intensity", Vec3f{0.85f,0.85f,0.85f});
    shadowP->set_uniform("ShadowMap", gl::Sampler2DShadow{0});
}

void Ch8ShadowMap::draw(tool::gl::Drawer *drawer){

    glEnable(GL_DEPTH_TEST);

    float c = 1.65f;
    lightPos = Vec3f(0.0f,c * 5.25f, c * 7.5f);  // World coords
    lightFrustum->orient( lightPos, {0,0,0}, Vec3f{0.0f,1.0f,0.0f});
    lightFrustum->set_perspective(60.0f, 1.0f, 1.0f, 25.0f);
    lightPV = lightFrustum->view_matrix()*lightFrustum->projection_matrix() * shadowBias;

    shadowP->use();
    {
        shadowFBO.bind();
        gl::TBO::bind_textures({shadowTexture.id()});
        glClear(GL_DEPTH_BUFFER_BIT);

        // Pass 1 (shadow map generation)
        viewP = lightFrustum->view_matrix();
        projP = lightFrustum->projection_matrix();
        glViewport(0,0,shadowMapWidth,shadowMapHeight);

        glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &pass1Index);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT); // specify whether front- or back-facing facets can be culled
        glEnable(GL_POLYGON_OFFSET_FILL); // set the scale and units used to calculate depth values
        glPolygonOffset(2.5f,10.0f);
        draw_scene();
        glCullFace(GL_BACK);
        glFlush();
    }

//    spitOutDepthBuffer(); // This is just used to get an image of the depth buffer

    {
        // Pass 2 (render)
        viewP = camera->view().conv<float>();
        projP = camera->projection().conv<float>();
        shadowP->set_uniform("Light.Position", camera->view().conv<float>().multiply_point(Pt4f(lightFrustum->origin(),1.0f)));

        gl::FBO::unbind();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glViewport(0,0,camera->screen()->width(), camera->screen()->height());
        glUniformSubroutinesuiv( GL_FRAGMENT_SHADER, 1, &pass2Index);

        draw_scene();
    }

    {
        // Draw the light's frustum
        solidP->use();
        solidP->set_uniform("Color", Vec4f{1.0f,0.0f,0.0f,1.0f});
        solidP->set_uniform("MVP",((lightFrustum->inverse_view_matrix()*camera->view().conv<float>())*camera->projection().conv<float>()));
        frustumD->draw();
    }

    {
        solidP->use();
        solidP->set_uniform("Color", Vec4f{1.0f,0.0f,0.0f,1.0f});
        model = Mat4d::translate(Mat4d(true), lightFrustum->origin().conv<double>());
        model = Mat4d::scale(model, {0.2f,0.2f,0.2f});
        update_matrices();
        solidP->set_camera_matrices_uniforms(camM);
        drawersM->get_drawer_ptr("cube-drawer")->draw();

        solidP->set_uniform("Color", Vec4f{1.0f,0.0f,1.0f,1.0f});
        auto p = Pt3f(Vec3f{x1,y1,z1});
        model = Mat4d::translate(Mat4d(true), p.conv<double>());
        model = Mat4d::scale(model, {0.2f,0.2f,0.2f});
        update_matrices();
        solidP->set_camera_matrices_uniforms(camM);
        drawersM->get_drawer_ptr("cube-drawer")->draw();
    }
}


void Ch8ShadowMap::spit_out_depth_buffer() {
    int size = shadowMapWidth * shadowMapHeight;
    float * buffer = new float[size];
    unsigned char * imgBuffer = new unsigned char[size * 4];

    glGetTexImage(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT,GL_FLOAT,buffer);

    for( int i = 0; i < shadowMapHeight; i++ )
        for( int j = 0; j < shadowMapWidth; j++ )
        {
            int imgIdx = 4 * ((i*shadowMapWidth) + j);
            int bufIdx = ((shadowMapHeight - i - 1) * shadowMapWidth) + j;

            // This is just to make a more visible image.  Scale so that
            // the range (minVal, 1.0) maps to (0.0, 1.0).  This probably should
            // be tweaked for different light configurations.
            float minVal = 0.88f;
            float scale = (buffer[bufIdx] - minVal) / (1.0f - minVal);
            unsigned char val = (unsigned char)(scale * 255);
            imgBuffer[imgIdx] = val;
            imgBuffer[imgIdx+1] = val;
            imgBuffer[imgIdx+2] = val;
            imgBuffer[imgIdx+3] = 0xff;
        }

    Texture t;
    std_v1<unsigned char> data(shadowMapHeight*shadowMapWidth*4);
    std::copy(imgBuffer, imgBuffer + shadowMapHeight*shadowMapWidth*4 , data.begin());
    t.copy_2d_data(shadowMapWidth, shadowMapHeight, 4, data);
    t.write_2d_image_file_data("./depth.png");

    delete [] buffer;
    delete [] imgBuffer;

//    exit(1);
}

void Ch8ShadowMap::draw_scene(){



    Vec3f color ={0.7f,0.5f,0.3f};
    shadowP->set_uniform("Material.Ka", color * 0.05f);
    shadowP->set_uniform("Material.Kd", color);
    shadowP->set_uniform("Material.Ks", Vec3f{0.9f,0.9f,0.9f});
    shadowP->set_uniform("Material.Shininess", 150.0f);
    model = Mat4d::rotate(Mat4d(true), Vec3d{1,0,0}, -90.);
    shadowP->set_uniform("ShadowMatrix", model.conv<float>()*lightFrustum->view_matrix()*lightFrustum->projection_matrix() * shadowBias);
    update_matrices(model, viewP.conv<double>(), projP.conv<double>());
    shadowP->set_camera_matrices_uniforms(camM);
    drawersM->get_drawer_ptr("teapot-drawer")->draw();

    shadowP->set_uniform("Material.Ka", color * 0.05f);
    shadowP->set_uniform("Material.Kd", color);
    shadowP->set_uniform("Material.Ks", Vec3f{0.9f,0.9f,0.9f});
    shadowP->set_uniform("Material.Shininess", 150.0f);
    model = Mat4d::translate(Mat4d(true), Vec3d{0.0f,2.0f,5.0f});
    model = Mat4d::rotate(model, Vec3d{1,0,0}, -45.);    
    shadowP->set_uniform("ShadowMatrix", (model.conv<float>() * lightPV));
    update_matrices(model, viewP.conv<double>(), projP.conv<double>());    
    shadowP->set_camera_matrices_uniforms(camM);
    drawersM->get_drawer_ptr("torus-drawer")->draw();

    shadowP->set_uniform("Material.Kd", Vec3f{0.25f, 0.25f, 0.25f});
    shadowP->set_uniform("Material.Ks", Vec3f{0.0f, 0.0f, 0.0f});
    shadowP->set_uniform("Material.Ka", Vec3f{0.05f, 0.05f, 0.05f});
    shadowP->set_uniform("Material.Shininess", 1.0f);
    model = Mat4d(true);
    shadowP->set_uniform("ShadowMatrix", (model.conv<float>() * lightPV));
    update_matrices(model, viewP.conv<double>(), projP.conv<double>());
    shadowP->set_camera_matrices_uniforms(camM);
    drawersM->get_drawer_ptr("notext-plane-40x40-drawer")->draw();

    model = Mat4d::translate(Mat4d(true), Vec3d{-5.0f,5.0f,0.0f});
    model = Mat4d::rotate(model, Vec3d{0,0,1}, -90.);
    shadowP->set_uniform("ShadowMatrix", (model.conv<float>() * lightPV));
    update_matrices(model, viewP.conv<double>(), projP.conv<double>());
    shadowP->set_camera_matrices_uniforms(camM);
    drawersM->get_drawer_ptr("notext-plane-40x40-drawer")->draw();

    model = Mat4d::translate(Mat4d(true), Vec3d{0.0f,5.0f,-5.0f});
    model = Mat4d::rotate(model, Vec3d{1,0,0}, 090.);
    shadowP->set_uniform("ShadowMatrix", (model.conv<float>() * lightPV));
    update_matrices(model, viewP.conv<double>(), projP.conv<double>());
    shadowP->set_camera_matrices_uniforms(camM);
    drawersM->get_drawer_ptr("notext-plane-40x40-drawer")->draw();
}

void Ch8ShadowMap2::init(){

    // configure depth map FBO
    glGenFramebuffers(1, &depthMapFBO);
    // create depth texture
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    // attach depth texture as FBO's depth buffer
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // retrieve shaders
    shadowMapping       = shadersM->get_ptr("learn/3_1_1_shadow_mapping");
    shadowMappingDepth  = shadersM->get_ptr("learn/3_1_1_shadow_mapping_depth");
    debugQuad           = shadersM->get_ptr("learn/3_1_1_debug_quad");

//    shadowMapping->set_uniform("diffuseTexture", gl::Sampler2D{0});
//    shadowMapping->set_uniform("shadowMap", gl::Sampler2D{1});
//    debugQuad->set_uniform("depthMap", gl::Sampler2D{0});
}

void Ch8ShadowMap2::draw(tool::gl::Drawer *drawer){

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 1. render depth of scene to texture (from light's perspective)
    // --------------------------------------------------------------
//    geo::Mat4f lightProjection, lightView;
    lightPos = {x,y,z};//mobileLightPos1.xyz();// {x,y,z};
    decltype (lightPos) lookAtPos = {x1,y1,z1};
    auto lightProjection  = Mat4f::Perspective(deg_2_rad(fov), 1600.f/600.f, nearPlane, farPlane);
    auto lightView        = Mat4f::LookAt(lightPos, lookAtPos, Pt3f{0.0, 1.0, 0.0});
    auto lightSpaceMatrix = lightView * lightProjection;// lightProjection;

    // render scene from light's point of view
    shadowMappingDepth->use();
    shadowMappingDepth->set_uniform("lightSpaceMatrix", lightSpaceMatrix);

    // 1. first render to depth map
    glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);
        render_scene(shadowMappingDepth);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

//    // reset viewport
//    glViewport(0, 0, camera->screen()->width(), camera->screen()->height());
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    // 2. then render scene as normal with shadow mapping (using depth map)
    glViewport(0, 0, camera->screen()->width(), camera->screen()->height());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shadowMapping->use();
    shadowMapping->set_uniform("projection", camera->projection().conv<float>());
    shadowMapping->set_uniform("view", camera->view().conv<float>());

    // set light uniforms
    shadowMapping->set_uniform("viewPos", camera->position().conv<float>());
    shadowMapping->set_uniform("lightPos", lightPos);
    shadowMapping->set_uniform("lightSpaceMatrix", lightSpaceMatrix);
    gl::TBO::bind_textures({texturesM->get_tbo("brick")->id(), depthMap});
    render_scene(shadowMapping);

    // render Depth map to quad for visual debugging
    // ---------------------------------------------
//    debugQuad->use();
//    debugQuad->set_uniform("near_plane", near_plane);
//    debugQuad->set_uniform("far_plane", far_plane);
//    gl::TBO::bind_textures({depthMap});
//    if(auto drawer = drawersM->get_drawer_ptr("screen-quad-drawer"); drawer != nullptr){
//        drawer->draw(debugQuad);
//    }
    auto shaderSolid = shadersM->get_ptr("ch8/solid");
    shaderSolid->use();
    model = Mat4d::transform({0.3,0.3,0.3},{0,0,0},lightPos.conv<double>());
    update_matrices();
    shaderSolid->set_camera_matrices_uniforms(camM);
    shaderSolid->set_uniform("Color", Pt4f{1,0,0,1});
    drawersM->get_drawer_ptr("sphere-drawer")->draw();

    model = Mat4d::transform({0.3,0.3,0.3},{0,0,0},lookAtPos.conv<double>());
    update_matrices();
    shaderSolid->set_camera_matrices_uniforms(camM);
    shaderSolid->set_uniform("Color", Pt4f{0,1,0,1});
    drawersM->get_drawer_ptr("sphere-drawer")->draw();

}


void Ch8ShadowMap2::render_scene(gl::ShaderProgram *shader){

    // floor
    gl::TBO::bind_textures({texturesM->get_tbo("brick")->id()});
    model = Mat4d::identity();
    shader->set_uniform("model", model.conv<float>());
    drawersM->get_drawer_ptr("notext-plane-40x40-drawer")->draw();
    // cubes

    gl::TBO::bind_textures({texturesM->get_tbo("hardwood_diffuse")->id()});
    for(int ii = 0; ii < 10; ++ii){
        for(int jj = 0; jj < 10; ++jj){

            model = Mat4d::transform({0.5,0.5,0.5},{ii*45.0,0.0,0.0},{3.0*ii, 1.0, 3.0*jj});

//            model = Mat4d::identity();
//            model = Mat4d::scale(model, {0.5f,0.5f,0.5f});
//            model = Mat4d::rotate(model, {45.f,0.f,0.f});
//            model = Mat4d::translate(model, {3.f*ii, 1.5, 3.f*jj});
            shader->set_uniform("model", model.conv<float>());
            drawersM->get_drawer_ptr("cube-drawer")->draw();

        }
    }    
}

void Ch8ShadowMap2::update_imgui(){
    Sample::update_imgui();
    ImGui::Text("Current:");
    ImGui::SliderFloat("near_plane", &nearPlane, 1.f, 10.4f, "ratio = %.3f");
    ImGui::SliderFloat("far_plane", &farPlane, 3.f, 200.4f, "ratio = %.3f");
    ImGui::SliderFloat("fov", &fov, 15.f, 360.0f, "ratio = %.3f");
}


