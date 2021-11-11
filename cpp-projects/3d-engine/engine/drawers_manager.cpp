
#include "drawers_manager.hpp"


using namespace tool;
using namespace tool::graphics;


bool DrawersManager::add_drawer(const Alias &alias, std::shared_ptr<gl::Drawer> drawer){

    if(drawers.count(alias) != 0){
        std::cerr << "[ModelsM] Drawer alias already used: " << alias << "\n";
        return false;
    }

    drawers[alias] = std::make_tuple(aliases.size(), drawer);
    aliases.push_back(alias);

    return true;
}

std::weak_ptr<tool::gl::Drawer> DrawersManager::get_drawer(const Alias &alias) const{
    if(drawers.count(alias) != 0){
        return std::get<1>(drawers.at(alias));
    }
    return {};
}

tool::gl::Drawer *DrawersManager::get_drawer_ptr(const Alias &alias) const{
    if(drawers.count(alias) != 0){
        return std::get<1>(drawers.at(alias)).get();
    }
    return nullptr;
}

gl::Drawer *DrawersManager::get_drawer_ptr(size_t id) const{
    if(id < aliases.size()){
        return get_drawer_ptr(aliases[id]);
    }
    return nullptr;
}

DrawersManager::Alias DrawersManager::get_alias(size_t id) const noexcept{
    if(id < aliases.size()){
        return aliases[id];
    }
    return  "";
}

size_t DrawersManager::get_id(const DrawersManager::Alias &alias) const{
    if(drawers.count(alias) != 0){
        return std::get<0>(drawers.at(alias));
    }
    return 0;
}
