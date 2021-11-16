
#include "shaders_manager.hpp"

// base
#include "utility/logger.hpp"

using namespace tool::graphics;
using namespace tool::gl;

bool ShadersManager::add_shader( const std::string &alias, const std::vector<std::string> &paths){

    if(shaders.count(alias) != 0){
        Logger::error(std::format("[ShadersM] Shader alias {} already exists.\n", alias));
        return false;
    }

    auto shader = std::make_shared<gl::ShaderProgram>();
    if(!shader->load_from_files(paths)){
        Logger::error("[ShadersM] Cannot generate ShaderProgram from paths:\n");
        for(const auto &path : paths){
            Logger::error(std::format(" -> {}\n", path));
        }
        return false;
    }

    shaders[alias] = shader;
    return true;
}

bool ShadersManager::add_shader(const std::string &alias, ShaderProgram&& shader){

    if(shaders.count(alias) != 0){
        Logger::error(std::format("[ShadersM] Shader alias {} already exists.\n", alias));
        return false;
    }

    shaders[alias] = std::make_shared<ShaderProgram>(std::move(shader));
    return true;
}

ShaderProgram *ShadersManager::reload_shader(ShaderProgram *shader){

    std::string alias;
    for(const auto &s : shaders){
        if(s.second.get() == shader){
            alias = s.first;
            break;
        }
    }

    auto reloadedShader = std::make_shared<gl::ShaderProgram>();
    if(!reloadedShader->load_from_files(shader->shaders_file_paths())){
        Logger::error(std::format("[ShadersM] Cannot reload ShaderProgram with alias {}:\n", alias));
        return nullptr;
    }

    shaders[alias]->clean();
    shaders[alias] = reloadedShader;

    return reloadedShader.get();
}

std::weak_ptr<ShaderProgram> ShadersManager::get(std::string alias){
    if(shaders.count(alias) != 0){
        return shaders[alias];
    }
    return {};
}

ShaderProgram *ShadersManager::get_ptr(std::string alias){
    if(shaders.count(alias) != 0){
        return shaders[alias].get();
    }
    return nullptr;
}

void ShadersManager::unbind(){
    ShaderProgram::unbind();
}
