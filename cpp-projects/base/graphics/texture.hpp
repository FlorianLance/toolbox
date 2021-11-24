

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
#include <cstdint>
#include <tuple>

// local
#include "utility/array_utility.hpp"
#include "utility/vector_utility.hpp"
#include "utility/tuple_array.hpp"
#include "geometry/point2.hpp"
#include "geometry/point3.hpp"
#include "geometry/point4.hpp"

namespace tool::graphics{

using namespace std::literals::string_view_literals;

enum class TextureType : std::int8_t {
    diffuse,        // The texture is combined with the result of the diffuse lighting equation.
    ambiant,        // The texture is combined with the result of the ambient lighting equation.
    specular,       // The texture is combined with the result of the specular lighting equation.
    normal,         // The texture is a (tangent space) normal-map.
    height,         // The texture is a height map. By convention, higher gray-scale values stand for higher elevations from the base height.
    light_map,      // Lightmap texture (aka Ambient Occlusion). The texture contains a scaling value for the final color value of a pixel. Its intensity is not affected by incoming light.
    opacity,        // The texture defines per-pixel opacity.
    emissive,       // The texture is added to the result of the lighting calculation. It isn't influenced by incoming light.
    shininess,      // The texture defines the glossiness of the material. The glossiness is in fact the exponent of the specular (phong) lighting equation.
    reflection,     // Contains the color of a perfect mirror reflection. Rarely used, almost never for real-time applications.
    displacement,   // Displacement texture.
    SizeEnum
};

enum class TextureOperation : std::int8_t{
    multiply,   // T = T1 * T2
    add,        // T = T1 + T2
    substract,  // T = T1 - T2
    divide,     // T = T1 / T2
    smooth_add, // T = (T1 + T2) - (T1 * T2)
    singed_add, // T = T1 + (T2-0.5)
    SizeEnum
};

enum class TextureMapping : std::int8_t{
    UV,         // mapping coordinates are taken from an UV channel
    sphere,     // spherical mapping
    cylinder,   // cylindrical mapping
    box,        // cubic mapping
    plane,      // planar mapping
    other,      // undefined
    SizeEnum
};

enum class TextureMapMode : std::int8_t{
    wrap,       // texture coordinate u|v is translated to u%1|v%1
    clamp,      // texture coordinates outside [0...1] are clamped to the nearest valid value.
    decal,      // if the texture coordinates for a pixel are outside [0...1] the texture is not applied to that pixel
    mirro,      // a texture coordinate u|v becomes u%1|v%1 if (u-(u%1))%2 is zero and 1-(u%1)|1-(v%1) otherwise
    SizeEnum
};

enum class TextureWrapMode : std::int8_t{
    clamp_to_edge,        // Causes s coordinates to be clamped to the range [12N,1−12N],
                          // where N is the size of the texture in the direction of clamping.
    clamp_to_border,      // Evaluates s coordinates in a similar manner to GL_CLAMP_TO_EDGE.
                          // However, in cases where clamping would have occurred in GL_CLAMP_TO_EDGE mode,
                          // the fetched texel data is substituted with the values specified by GL_TEXTURE_BORDER_COLOR.
    mirrored_repeat,      // Causes the s coordinate to be set to the fractional part of the texture coordinate if the integer part of s is even;
                          // if the integer part of s is odd, then the s texture coordinate is set to 1−frac(s), where frac(s) represents the fractional part of s.
    repeat,               // Causes the integer part of the s coordinate to be ignored; the GL uses only the fractional part, thereby creating a repeating pattern.
    mirror_clamp_to_edge, // Causes the scoordinate to be repeated as for GL_MIRRORED_REPEAT for one repetition of the texture, at which point the coordinate
    SizeEnum              // to be clamped as in GL_CLAMP_TO_EDGE. Initially, GL_TEXTURE_WRAP_S is set to GL_REPEAT.
};

enum class TextureMode : std::int8_t{
    texture_1d,             // Images in this texture all are 1-dimensional. They have width, but no height or depth.
    texture_2d,             // Images in this texture all are 2-dimensional. They have width and height, but no depth.
    texture_3d,             // Images in this texture all are 3-dimensional. They have width, height, and depth.
    rectangle,              // The image in this texture (only one image. No mipmapping) is 2-dimensional. Texture coordinates used for these textures are not normalized.
    buffer,                 // The image in this texture (only one image. No mipmapping) is 1-dimensional. The storage for this data comes from a Buffer Object.
    cubemap,                // There are exactly 6 distinct sets of 2D images, all of the same size. They act as 6 faces of a cube.
    texture_1d_array,       // Images in this texture all are 1-dimensional. However, it contains multiple sets of 1-dimensional images, all within one texture. The array length is part of the texture's size.
    texture_2d_array,       // Images in this texture all are 2-dimensional. However, it contains multiple sets of 2-dimensional images, all within one texture. The array length is part of the texture's size.
    cubemap_array,          // Images in this texture are all cube maps. It contains multiple sets of cube maps, all within one texture. The array length * 6 (number of cube faces) is part of the texture size.
    texture_2d_multisample,      // The image in this texture (only one image. No mipmapping) is 2-dimensional. Each pixel in these images contains multiple samples instead of just one value.
    texture_2d_multisample_array,// Combines 2D array and 2D multisample types. No mipmapping.
    SizeEnum
};

// The texture minifying function is used whenever the level-of-detail function used when sampling from the texture determines that the texture should be minified.
// There are six defined minifying functions. Two of them use either the nearest texture elements or a weighted average of multiple texture elements to compute the texture value. The other four use mipmaps.
// A mipmap is an ordered set of arrays representing the same image at progressively lower resolutions. If the texture has dimensions 2n×2m,
// there are max(n,m)+1 mipmaps. The first mipmap is the original texture, with dimensions 2n×2m. Each subsequent mipmap has dimensions 2k−1×2l−1, where 2k×2l are the dimensions of the previous mipmap,
// until either k=0 or l=0. At that point, subsequent mipmaps have dimension 1×2l−1 or 2k−1×1 until the final mipmap, which has dimension 1×1.
// To define the mipmaps, call glTexImage1D, glTexImage2D, glTexImage3D, glCopyTexImage1D, or glCopyTexImage2D with the level argument indicating the order of the mipmaps. Level 0 is the original texture;
// level max(n,m) is the final 1×1
// As more texture elements are sampled in the minification process, fewer aliasing artifacts will be apparent.
// While the GL_NEAREST and GL_LINEAR minification functions can be faster than the other four, they sample only one or multiple texture elements to determine the texture value of
// the pixel being rendered and can produce moire patterns or ragged transitions. The initial value of GL_TEXTURE_MIN_FILTER is GL_NEAREST_MIPMAP_LINEAR.
enum class TextureMinFilter : std::int8_t{
    nearest,                // Returns the value of the texture element that is nearest (in Manhattan distance) to the specified texture coordinates.
    linear,                 // Returns the weighted average of the four texture elements that are closest to the specified texture coordinates.
                            // These can include items wrapped or repeated from other parts of a texture, depending on the values of GL_TEXTURE_WRAP_S and GL_TEXTURE_WRAP_T, and on the exact mapping.
    nearest_mipmap_nearest, // Chooses the mipmap that most closely matches the size of the pixel being textured and uses the GL_NEAREST criterion
                            // (the texture element closest to the specified texture coordinates) to produce a texture value.
    linear_mimmap_nearest,  // Chooses the mipmap that most closely matches the size of the pixel being textured and uses the GL_LINEAR criterion
                            // (a weighted average of the four texture elements that are closest to the specified texture coordinates) to produce a texture value.
    nearest_mipmap_linear,  // Chooses the two mipmaps that most closely match the size of the pixel being textured and uses the GL_NEAREST criterion
                            // (the texture element closest to the specified texture coordinates ) to produce a texture value from each mipmap. The final texture value is a weighted average of those two values.
    linear_mipmap_linear,   // Chooses the two mipmaps that most closely match the size of the pixel being textured and uses the GL_LINEAR criterion
                            // (a weighted average of the texture elements that are closest to the specified texture coordinates) to produce a texture value from each mipmap. The final texture value is a weighted average of those two values.
    SizeEnum
};

// The texture magnification function is used whenever the level-of-detail function used when sampling from the texture determines that the texture should be magnified.
// It sets the texture magnification function to either GL_NEAREST or GL_LINEAR (see below).
// GL_NEAREST is generally faster than GL_LINEAR, but it can produce textured images with sharper edges because the transition between texture elements is not as smooth.
// The initial value of GL_TEXTURE_MAG_FILTER is GL_LINEAR.
enum class TextureMagFilter : std::int8_t{
    nearest,// Returns the value of the texture element that is nearest (in Manhattan distance) to the specified texture coordinates.
    linear, // Returns the weighted average of the texture elements that are closest to the specified texture coordinates.
            // These can include items wrapped or repeated from other parts of a texture, depending on the values of GL_TEXTURE_WRAP_S and GL_TEXTURE_WRAP_T, and on the exact mapping.
    SizeEnum
};

enum class TextureMaxLevel : std::int8_t{
    SizeEnum
};


using TTexType = std::tuple<
             TextureType,                 std::string_view>;
static constexpr TupleArray<TextureType::SizeEnum, TTexType> textureTypes{{
    TTexType{TextureType::diffuse,        "diffuse"sv     },
    TTexType{TextureType::ambiant,        "ambiant"sv     },
    TTexType{TextureType::specular,       "specular"sv    },
    TTexType{TextureType::normal,         "normal"sv      },
    TTexType{TextureType::height,         "height"sv      },
    TTexType{TextureType::light_map,      "lightMap"sv    },
    TTexType{TextureType::opacity,        "opacity"sv     },
    TTexType{TextureType::emissive,       "emissive"sv    },
    TTexType{TextureType::shininess,      "shininess"sv   },
    TTexType{TextureType::reflection,     "reflection"sv  },
    TTexType{TextureType::displacement,   "displacement"sv},
}};


[[maybe_unused]] static std::string_view to_string(TextureType t) {
    return textureTypes.at<0,1>(t);
}

struct TextureOptions{
    TextureType type                  = TextureType::diffuse;
    TextureOperation operation        = TextureOperation::add;
    TextureMapping mapping            = TextureMapping::UV;
    geo::Pt3<TextureMapMode> mapMode{TextureMapMode::wrap,TextureMapMode::wrap,TextureMapMode::wrap};

    // mixed
    TextureWrapMode wrapS       = TextureWrapMode::repeat; // Sets the wrap parameter for texture coordinate s
    TextureWrapMode wrapT       = TextureWrapMode::repeat; // Sets the wrap parameter for texture coordinate t
    TextureWrapMode wrapR       = TextureWrapMode::repeat; // Sets the wrap parameter for texture coordinate r
    TextureMinFilter minFilter  = TextureMinFilter::nearest_mipmap_linear;
    TextureMagFilter magFilter  = TextureMagFilter::linear;

    geo::Pt4f borderColor{0.0f,0.0f,0.0f,0.0f};

    int maxLevel = 1000; // Sets the index of the highest defined mipmap level. This is an integer value. The initial value is 1000.
    int minLOD = -1000; // Sets the minimum level-of-detail parameter. This floating-point value limits the selection of highest resolution mipmap (lowest mipmap level). The initial value is -1000.
    int maxLOD = 1000; // Sets the maximum level-of-detail parameter. This floating-point value limits the selection of the lowest resolution mipmap (highest mipmap level). The initial value is 1000.
};


struct TextureAlias{
    std::string v;
};

struct TexturePath{
    std::string v;
};

struct Texture{

    constexpr int nb_channels()const{return m_nbChannels;}

    void copy_2d_data(int width, int height, int nbChannels, unsigned char *data);
    void copy_2d_data(int width, int height, int nbChannels, const std_v1<unsigned char> &data);
    void copy_2d_data(int width, int height, const std_v1<geo::Pt3<unsigned char>> &data);
    void copy_2d_data(int width, int height, int nbChannels, const std_v1<float> &data);
    void copy_2d_data(int width, int height, const std_v1<geo::Pt3<float>> &data);

    bool load_2d_image_file_data(const std::string &pathTexture, bool flip, int targetNbChannels = 3);
    bool write_2d_image_file_data(const std::string &pathTexture);
    unsigned char *data(){return m_data.data();}

    constexpr bool is_hdr() const {return m_hdr;}

protected:

    bool m_hdr = false;
    std_v1<unsigned char> m_data;
    int m_nbChannels = 0;
    std_v1<int> m_sizes;
};

struct Texture2D : public Texture{

    std::string path;

    Texture2D() = default;
    explicit Texture2D(const std::string &pathTexture, bool flip = true, int targetNbChannels = 4);

    constexpr geo::Pt2<int> size() const { return geo::Pt2<int>{m_sizes[0], m_sizes[1]};}

private :

    bool m_flipped = false;

};

struct CubeMap{
    // posx.png","negx.png","posy.png","negy.png","posz.png","negz.png
    std_a1<Texture2D,6> textures;

    CubeMap() = default;

    bool load_2d_images_files(const std_a1<TexturePath, 6> &pathTextures, bool flip, int targetNbChannels = 3);
    bool load_2d_images_files(const std::string &basePath, std_a1<std::string, 6> &extensions, bool flip, int targetNbChannels = 3);

};

struct TextureInfo {
    Texture2D *texture = nullptr;
    TextureOptions options;
};

}
