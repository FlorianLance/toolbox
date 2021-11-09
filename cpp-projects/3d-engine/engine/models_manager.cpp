
#include "models_manager.hpp"

// std
#include <future>
#include <set>

// base
#include "files/assimp_loader.hpp"
#include "utility/logger.hpp"

using namespace tool;
using namespace tool::graphics;


bool ModelsManager::add_models(std::vector<std::pair<Alias, Path> > aliasPaths, bool forceReload){

    // preprocess aliases/paths
    std::vector<std::pair<Alias, Path>> newV;
    newV.reserve(aliasPaths.size());

    std::set<Alias> aliases;
    std::set<Path> paths;
    for(auto &ap : aliasPaths){

        if(aliases.count(ap.first) != 0){ // check for duplicates aliases
            continue;
        }
        if(paths.count(ap.second) != 0){ // check for duplicates paths
            continue;
        }
        aliases.insert(ap.first);
        aliases.insert(ap.second);

        if(!forceReload){
            if(aliasPerPath.count(ap.second) != 0){
                Logger::error(std::format("[ModelsM] Model with alias {} and path {} already loaded with alias {}, loading canceled.\n",
                        ap.first, ap.second, aliasPerPath[ap.second]));
                continue;
            }
        }

        newV.emplace_back(std::move(ap));
    }

    // do asynch loading
    std::unordered_map<Alias, std::future<std::shared_ptr<Model<float>>>> m;
    for(const auto &ap : newV){
        aliasPerPath[ap.first] = ap.second;
        m[ap.first] = std::async(std::launch::async, files::AiLoader::load_model, ap.second, false);
    }

    // add results to map
    for(auto &futureModel : m){
        models[futureModel.first] = futureModel.second.get();
    }

    return true;
}

bool ModelsManager::add_model(std::string alias, const std::string &path){

    if(aliasPerPath.count(path) != 0){
        Logger::error(std::format(
            "[ModelsM] Model with alias {} and path {} already loaded with alias {}, loading canceled.\n",
            alias, path, aliasPerPath[path])
        );
        return false;
    }

    if(models.count(alias) != 0){
        Logger::error(std::format(
            "[ModelsM] Model alias {} already used, it will be replaced by newly loaded model.\n",
            alias)
        );

    }
    if(auto model = files::AiLoader::load_model(path); model != nullptr){
        models[alias]  = model;
        aliasPerPath[path] = alias;
        return true;
    }

    return false;

}

std::weak_ptr<tool::graphics::Model<float> > ModelsManager::get_model(std::string alias){
    if(models.count(alias) != 0){
        return models[alias];
    }
    return {};
}

tool::graphics::Model<float> *ModelsManager::get_model_ptr(std::string alias){
    if(models.count(alias) != 0){
        return models[alias].get();
    }
    return nullptr;
}
