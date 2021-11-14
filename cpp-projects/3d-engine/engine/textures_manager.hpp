
/*******************************************************************************
** Toolbox-3d-engine                                                          **
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

// opengl-utlity
#include "opengl/gl_texture.hpp"

namespace tool::graphics {

struct TextureLoadingInfo{
    std::string alias;
    std::string fileName;
    bool flip = true;
    int targetNbChannels = 4;
};

class TexturesManager{

public:

    using Alias = std::string;
    using Path  = std::string;

    bool load_textures_from_directory(const Path &directoryPath, std_v1<TextureLoadingInfo> infos);
    bool load_cube_map(const Path &basePath, const std_a1<std::string,6> &extensions, const Alias &alias, bool flip = true);

    std::weak_ptr<Texture2D> get_texture(const Alias &textureAlias);
    Texture2D* get_texture_ptr(const Alias &textureAlias);
    TextureInfo get_texture_info(const Alias &textureAlias, TextureOptions options = {});

    bool generate_texture2d_tbo(const Alias &tboAlias, const Alias &textureAlias, TextureOptions options = {});
    bool generate_projected_texture2d_tbo(const Alias &tboAlias, const Alias &textureAlias);
    bool generate_cubemap_tbo(const Alias &tboAlias, const Alias &cubemapAlias);

    gl::TBO *texture_tbo(const Alias &tboAlias);
    gl::TextureName texture_id(const Alias &tboAlias);

    gl::TBO *cube_map_tbo(const Alias &tboAlias);
    gl::TextureName cube_map_id(const Alias &tboAlias);

//    constexpr size_t textures_count() const noexcept {return texturesAliases.size();}
//    constexpr size_t cube_maps_count() const noexcept {return cubeMapsAliases.size();}
//    constexpr size_t tbo_count() const noexcept {return tboAliases.size();}

//    gl::TextureName get_cube_maps_tbo(){

//    }

private:

    std::vector<Alias> texturesAliases;
    std::vector<Alias> cubeMapsAliases;
    std::vector<Alias> tboAliases;

    std::unordered_map<Path,  Alias> aliasPerPath;

    std::unordered_map<Alias, std::shared_ptr<Texture2D>> textures;
    std::unordered_map<Alias, std::shared_ptr<CubeMap>> cubeMaps;
    std::unordered_map<Alias, gl::TBO> texturesTBOs;
    std::unordered_map<Alias, gl::TBO> cubeMapsTBOs;
};

}
