
#pragma once

// glm
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

// opengl-utility
#include "opengl/buffer/framebuffer_object.hpp"
#include "opengl/buffer/uniform_buffer_object.hpp"
#include "opengl/buffer/shader_storage_buffer_object.hpp"
#include "opengl/buffer/pixel_buffer_object.hpp"
#include "opengl/buffer/atomic_buffer_object.hpp"
#include "opengl/gl_material.hpp"
#include "opengl/sampler.hpp"

// local
#include "engine/managers.hpp"


namespace tool::graphics {

// std140
struct Light2{
    geo::Vec4f Position; // Light position in cam coords
    geo::Vec3f La;
    float padding1;
    geo::Vec3f Ld;
    float padding2;
    geo::Vec3f Ls;
    float padding3;
};

// std140
struct MaterialPbr{

    MaterialPbr(geo::Pt4f c, float r, float m) : color(c), rough(r), metal(m){
    }

    geo::Pt4f color;                        // 0
    float rough;                            // 12
    float metal;                            // 16
    float padding1;                         // 20
    float padding2;                         // 24
    // float padding3;                      // 28
    // float padding4;                      // 32
};





struct Sample{

    geo::Vec3f from_glm(const glm::vec3 &v) const{
        return {v.x, v.y, v.z};
    }

    geo::Vec4f from_glm(const glm::vec4 &v) const{
        return {v.x, v.y, v.z, v.w};
    }

    geo::Mat4f from_glm(const glm::mat4 &m) const{
        return {
            m[0][0], m[1][0], m[2][0], m[3][0],
            m[0][1], m[1][1], m[2][1], m[3][1],
            m[0][2], m[1][2], m[2][2], m[3][2],
            m[0][3], m[1][3], m[2][3], m[3][3],
        };
    }

    float gauss(float x, float sigma2 ) const{
        double coeff = 1.0 / (tool::two_PI<double>* sigma2);
        double expon = -(x*x) / (2.0 * sigma2);
        return (float) (coeff*exp(expon));
    }

    Sample(Camera *cam) :
        // camera
        camera(cam),
        // managers
        shadersM(&Managers::shaders),
        texturesM(&Managers::textures),
        modelsM(&Managers::models),
        drawersM(&Managers::drawers)
    {

    }

    virtual void init(){}
    virtual void update_screen_size(){}
    virtual void update(float elapsedSeconds);    
    virtual void draw(gl::Drawer *drawer = nullptr);
    virtual void parent_update_imgui();
    virtual void update_imgui(){}


    void draw_nb(gl::ShaderProgram *shader, gl::Drawer *drawer);

    void update_matrices();
    void update_matrices_m(const geo::Mat4d &model = geo::Mat4d(true));
    void update_matrices_p(const geo::Mat4d &proj);
    void update_matrices_vp(const geo::Mat4d &view, const geo::Mat4d &proj);
    void update_matrices_mvp(const geo::Mat4d &model, const geo::Mat4d &view, const geo::Mat4d &proj);

    void draw_screen_quad(gl::ShaderProgram *shader);
    void draw_floor();
    void draw_lights();
    void draw_skybox();

    void draw_scene1(gl::ShaderProgram *shader);

public:

    // ## TODO: remove
    static inline float alpha = 0.f;
    static inline geo::Pt3f xyz = {};
    static inline float x1 = 0.f;
    static inline float y1 = 0.f;
    static inline float z1 = 1.f;
    static inline float rx = 0.f;
    static inline float ry = 0.f;
    static inline float rz = 0.f;
    // ##


protected:

    float elapsedSeconds = 0.f;

    // gui parameters
    static inline bool drawFloor  = true;
    static inline bool drawSkybox = true;
    static inline bool drawLights = true;
    static inline bool moveLight = true;
    static inline geo::Pt3f modelPos = {0.f,0.f, 2.f};
    static inline geo::Pt3f modelRot = {};
    static inline geo::Pt3f skyboxRot = {};
    static inline float scale = 1.f;
    static inline geo::Pt3<int> nb = {1,1,1};
    static inline geo::Pt4f worldLight{0.f,10.f,0.f,1.0f};
    static inline geo::Pt4f mobileLightPos1 = geo::Pt4f(5.0f, 5.0f, 5.0f, 1.0f);
    static inline geo::Pt4f mobileLightPos2 = geo::Pt4f(4.0f, 4.0f, 4.0f, 1.0f);
    static inline int idAnimation = 0;
    static inline int nbAnimations = 0;
    static inline bool stopAnimation = false;
    static inline float timeAnimation = 0.f;
    static inline float durationAnimation = 10.f;

    static inline float camFov = 60.f;

    static inline gl::TextureName skyboxT;

    static inline graphics::LightInfo lInfo = {
        {},
        {0.4f, 0.4f, 0.4f},
        {1.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f}
    };

    static inline graphics::MaterialInfo mInfo = {
        {0.5f, 0.5f, 0.5f},
        {0.5f, 0.5f, 0.5f},
        {0.8f, 0.8f, 0.8f},
        10.0f
    };

    static inline graphics::MaterialPbr mPbrInfo = {
        {1.f, 0.71f, 0.29f,1.f},
        0.50f,
        0.f
    };

    static inline graphics::RefractMaterialInfo rmInfo = {
        0.1f, 0.94f
    };

//    auto gold = MaterialPbr(Pt4f(1, 0.71f, 0.29f,1.f),           0.50f,0.f);
//    auto copper = MaterialPbr(Pt4f(0.95f, 0.64f, 0.54f,1.f),       0.50f,0.f);
//    auto aluminium = MaterialPbr(Pt4f(0.91f, 0.92f, 0.92f,1.f),       0.50f,0.f);
//    auto titanium = MaterialPbr(Pt4f(0.542f, 0.497f, 0.449f,1.f),    0.50f,0.f);
//    auto silver = MaterialPbr(Pt4f(0.95f, 0.93f, 0.88f,1.f),       0.50f,0.f);

    // camera / transform
    Camera *camera = nullptr;
    CameraMatrices camM;

    // managers
    ShadersManager  *shadersM   = nullptr;
    TexturesManager *texturesM  = nullptr;
    ModelsManager   *modelsM    = nullptr;
    DrawersManager  *drawersM   = nullptr;

    // UBO
    gl::LightUBO lightUBO;
    gl::MaterialUBO materialUBO;

    // shaders
    gl::ShaderProgram *shader = nullptr;
};

// ############################################## CH3
struct Ch3Diffuse : public Sample{
    Ch3Diffuse(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer) final override;
};
struct Ch3Flat : public Sample{
    Ch3Flat(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch3Discard : public Sample{
    Ch3Discard(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float discardV = 0.2f;
    float scaleV = 15.f;
};
struct Ch3TwoSide : public Sample{
    Ch3TwoSide(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch3Phong : public Sample{
    Ch3Phong(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
// ############################################## CH4
struct Ch4PhongDirectionnalLight : public Sample{
    Ch4PhongDirectionnalLight(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch4PhongMultiLights : public Sample{
    Ch4PhongMultiLights(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch4PhongPerFragment : public Sample{
    Ch4PhongPerFragment(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch4BlinnPhong : public Sample{
    Ch4BlinnPhong(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch4Cartoon : public Sample{
    Ch4Cartoon(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    int levels = 5;
};
struct Ch4PBR : public Sample{
    Ch4PBR(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    std_v1<Light2> lights;
    gl::UBO lightsB;
    gl::UBO materialsB;
};
// ############################################## CH5
struct Ch5DiscardPixels : public Sample{
    Ch5DiscardPixels(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float decayFactor = 0.f;
};
struct Ch5SceneTexture : public Sample{
    Ch5SceneTexture(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch5SceneMutliTexture : public Sample{
    Ch5SceneMutliTexture(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch5NormalMap : public Sample{
    Ch5NormalMap(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
};
struct Ch5ParallaxMapping : public Sample{
    Ch5ParallaxMapping(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    bool showHeightMap = false;
    float bumpFactor = 0.015f;
};
struct Ch5SteepParallaxMapping : public Sample{
    Ch5SteepParallaxMapping(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float bumpScale = 0.03f;
};
struct Ch5ReflectCubeMap : public Sample{
    Ch5ReflectCubeMap(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float reflectFactor = 0.1f;
    geo::Pt4f matColor = {0.5f, 0.5f, 0.5f, 1.0f};
};
struct Ch5RefractCubeMap : public Sample{
    Ch5RefractCubeMap(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float reflectFactor = 0.1f;
    float eta = 0.94f;
};
struct Ch5ProjectTexture : public Sample{
    Ch5ProjectTexture(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float fov = 30.f;
    float aspectRatio = 1.f;
    float zNear = 0.2f;
    float zFar = 1000.f;
    geo::Pt3f projPos = {0.0f,3.0f,0.0f};
    geo::Pt3f projRot = {-120.0f,0.0f,0.0f};
    TextureOptions projOptions;
    gl::ShaderProgram *solidP = nullptr;      
};
struct Ch5DiffuseImageBasedLighting : public Sample{
    Ch5DiffuseImageBasedLighting(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float reflectFactor = 0.1f;
    float gamma = 2.2f;
};

struct Ch5SamplerObject : public Sample{
    Ch5SamplerObject(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:

    gl::Sampler sampler1;
    TextureOptions options1;

    static inline std::vector<std::string> magFiltersStr = {
        "nearest",
        "linear"
    };

    static inline std::vector<std::string> minFiltersStr = {
        "nearest",
        "linear",
        "nearest_mipmap_nearest",
        "linear_mimmap_nearest",
        "nearest_mipmap_linear",
        "linear_mipmap_linear",
    };

    static inline std::vector<std::string> wrapModeStr = {
        "clamp_to_edge",
        "clamp_to_border",
        "mirrored_repeat",
        "repeat",
        "mirror_clamp_to_edge"
    };
};

struct Ch5RenderToTexture: public Sample{
    Ch5RenderToTexture(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update(float elapsedSeconds)final override;
    void update_imgui() final override;
public:
    double angle = 0.;
private:
    // render to texture GBO
    gl::FBO fboCh5RenderToTexture;
    gl::RBO depthBufCh5RenterToTexture;
    gl::Texture2D renderTexCh5RenderToTexture;

    geo::Pt3f projModelPos = {0.16f, 0.32f, 3.f};
    geo::Pt3f projModelRot = {205.f, -205.f, 274.f};
    float projScale = 2.f;
};

struct Ch6EdgeDetectionFilter: public Sample{
    Ch6EdgeDetectionFilter(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    bool enable = true;
    float edgeThreshold = 0.05f;
    // screen FBO
    gl::FBO screenFBO;
    gl::RBO screenDepthBuffer;
    gl::Texture2D screenRenderTexture;
};

struct Ch6GaussianFilter : public Sample{
    Ch6GaussianFilter(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    bool enable = true;
    float sigma2 = 4.0f;
    std_v1<float> weights;
    // screen FBO
    gl::FBO screenFBO;
    gl::RBO screenDepthBuffer;
    gl::Texture2D screenRenderTexture;
    // intermediate FBO
    gl::FBO intermediateFBO;
    gl::Texture2D intermediateRenderTexture;    
};

struct Ch6HdrLightingToneMapping : public Sample{
    Ch6HdrLightingToneMapping(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    bool doToneMap = true;
    // hdr FBO
    gl::FBO hdrFBO;
    gl::RBO hdrDepthBuffer;
    gl::Texture2D hdrRenderTexture;
    // texture data
    std::vector<GLfloat> texData;
};

struct Ch6HdrBloom : public Sample{
    Ch6HdrBloom(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float luminanceThreshold = 1.7f;
    float exposure = 0.35f;
    float white = 0.928f;
    float sigma = 25.0f;
    float gamma = 2.2f;
    // hdr FBO
    gl::FBO hdrFBO;
    gl::RBO hdrDepthBuffer;
    gl::Texture2D hdrRenderTexture;
    // blur FBO
    gl::FBO blurFBO;
    gl::Texture2D blurTex1;
    gl::Texture2D blurTex2;
    // sampler objects
    gl::Sampler linearSampler;
    gl::Sampler nearestSampler;
    // others
    std_v1<float> weights;
    int bloomBufWidth, bloomBufHeight;
    // texture data
    std::vector<GLfloat> texData;
};























struct Ch6Deferred : public Sample{
    Ch6Deferred(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
private:
    gl::FBO deferredFBO;
    gl::RBO depthBuf;
    gl::GeometryTexture2D posTex;
    gl::GeometryTexture2D normTex;
    gl::GeometryTexture2D diffuseColorTex;
    gl::GeometryTexture2D ambiantColorTex;
    gl::GeometryTexture2D specularColorTex;
    std_v1<geo::Pt3f> lightsColors;
};

struct Ch6SSAO : public Sample{
    Ch6SSAO(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float radius = 0.55f;
    float factorScale = 4.f;
    gl::FBO deferredFBO;
    gl::RBO depthBuf;
    gl::FBO ssaoFBO;
    gl::GeometryTexture2D posTex;
    gl::GeometryTexture2D normTex;
    gl::GeometryTexture2D colorTex;
    gl::GeometryTexture2D aoTex[2];
    gl::Texture2D randRotationTex;
    std::vector<geo::Vec3f> kern;
};

struct Ch6OIT : public Sample{
    struct ListNode {
        geo::Col4f color;
        GLfloat depth;
        GLuint next;
    };
    Ch6OIT(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
public:
    float angle, tPrev, rotSpeed;
private:
    gl::SSBO linkedListBuffer;
    gl::ABO counterBuffer;
    gl::PBO clearBuffer;
    gl::Texture2D headPtrTexture;
    GLuint pass1Index, pass2Index;
    gl::ShaderProgram *shader = nullptr;
};

struct Ch7BezCurve : public Sample{
    Ch7BezCurve(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    int numSegments = 50;
    GLuint vaoHandle;
    gl::ShaderProgram *prog = nullptr;
    gl::ShaderProgram *solidProg = nullptr;
    std::unique_ptr<gl::CloudPointsDrawer> bezPoints = nullptr;
};

struct Ch7ShadeWire : public Sample{
    Ch7ShadeWire(Camera *cam) : Sample(cam){}
    void init() final override;
    void update_screen_size() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float lineWidth = 0.75f;
    gl::ShaderProgram *shader = nullptr;
};

struct Ch7ScenePointSprite : public Sample{
    Ch7ScenePointSprite(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void update_imgui() final override;
private:
    float sizeSprite = 0.15f;
    int numSprites = 50;
    GLuint sprites;
    gl::ShaderProgram *shader = nullptr;
    std::unique_ptr<gl::CloudPointsDrawer> pointsSprites = nullptr;
};

struct Ch7Silhouette : public Sample{
    Ch7Silhouette(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
private:
    gl::ShaderProgram *shader = nullptr;
};

struct Ch8ShadowMap : public Sample{
    Ch8ShadowMap(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
private:
    void spit_out_depth_buffer();
    void draw_scene();
    gl::FBO shadowFBO;
    gl::Texture2D shadowTexture;
    GLuint pass1Index, pass2Index;
    int shadowMapWidth = 1024, shadowMapHeight = 1024;
    geo::Mat4f lightPV, shadowBias;
    geo::Mat4f viewP, projP;
    gl::ShaderProgram *shadowP = nullptr;
    gl::ShaderProgram *solidP = nullptr;
    gl::FrustumDrawer *frustumD = nullptr;
    gl::Frustum *lightFrustum = nullptr;
    geo::Pt3f lightPos;
};

struct Ch8ShadowMap2 : public Sample{
    Ch8ShadowMap2(Camera *cam) : Sample(cam){}
    void init() final override;
    void draw(gl::Drawer *drawer = nullptr) final override;
    void render_scene(gl::ShaderProgram *shader);
    void update_imgui() final override;
private:
    unsigned int depthMapFBO;
    unsigned int depthMap;
    const unsigned int SHADOW_WIDTH = 512, SHADOW_HEIGHT = 512;
    gl::ShaderProgram *shadowMapping = nullptr;
    gl::ShaderProgram *shadowMappingDepth = nullptr;
    gl::ShaderProgram *debugQuad = nullptr;
    geo::Pt3f lightPos{-2.0f, 4.0f, -1.0f};
public:
    float nearPlane = 1.0f;
    float farPlane  = 10.5f;
    float fov       = 60.f;
};
}


