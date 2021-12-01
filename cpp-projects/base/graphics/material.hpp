

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
#include <vector>

// base
#include "geometry/point3.hpp"
#include "utility/vector.hpp"

// local
#include "texture.hpp"

namespace tool::graphics {

enum class BlendMode : std::int8_t{
    default_blend,  // SourceColor*SourceAlpha + DestColor*(1-SourceAlpha)
    additive_blend, //  SourceColor*1 + DestColor*1
};

enum class ShadingModel : std::int8_t{
    flat, gouraud, phong, phong_blinn, toon, oren_nayar, minnarert, coook_torrance, no_shadding, fresnel
};

struct Material { // http://assimp.sourceforge.net/lib_html/materials.html

    enum class Property : std::int8_t{
        name,twosided,shading_model,enable_wireframe,blend_func,opacity, bumpscaling, shininess, reflectivity,
        shininess_strength, refacti, color_diffuse, color_ambient, color_specular, color_emissive, color_transparent,
        color_reflective, global_background_image,
        text_blend, text_mapping, text_operation, text_uvw_source,
        text_mapping_mode_u, text_mapping_mode_v,
        text_texmap_axis, text_flags,
        SizeEnum
    };

    //  TODO: add default values
    bool backfaceCulling = true;
    bool wireframe = false;

    float opacity;
    float shininess;
    float shininessStrength;
    float refraction;
    float reflectivity;

    geo::Pt3f ambiantColor;
    geo::Pt3f diffuseColor;
    geo::Pt3f specularColor;
    geo::Pt3f emissiveColor;
    geo::Pt3f transparentColor;
    geo::Pt3f reflectiveColor;

    ShadingModel shadingModel;
    BlendMode blendMode;

    std::string name;

    std::unordered_map<TextureType, std_v1<Texture2D*>> textures;
    std::unordered_map<TextureType, std_v1<TextureInfo>> texturesInfo;
};


struct MaterialInfo {

    // [size: 48]
    geo::Vec3f Ka;      // Ambient reflectivity         [offset: 0]
    geo::Vec3f Kd;      // Diffuse reflectivity         [offset: 16]
    geo::Vec3f Ks;      // Specular reflectivity        [offset: 32]
    float Shininess;    // Specular shininess factor    [offset: 44]
};

struct RefractMaterialInfo {
    float eta;              // ration of indices of refraction
    float reflectionFactor; // percentage of reflected light
};

}

