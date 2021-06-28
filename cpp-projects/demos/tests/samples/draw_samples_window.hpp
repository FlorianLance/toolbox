
#pragma once

// imgui
#include "imgui/extra/implot/implot.h"

// local
#include "windows/base_sfml_gl_window.hpp"
#include "engine/managers.hpp"
#include "samples.hpp"

namespace tool::graphics {

class DrawSampleWindow : public BaseSfmlGlWindow{

public:

    DrawSampleWindow(std::string title, geo::Pt2<unsigned int> size, std::optional<sf::ContextSettings> context = std::nullopt) :
          BaseSfmlGlWindow(title, size, context){}

private:

    // gl
    bool initialize_gl() override;
    void update_gl() override;
    void post_update() override;
    // imgui
    void update_imgui() override;
    // # window
    void resize_windows() override;

    // managers
    bool init_textures();
    bool init_shaders();
    bool init_models();
    bool init_drawers();
    bool init_samples();

protected:

    // imgui
    ImPlotContext *imPlotContext = nullptr;

    // samples
    int currentDemo = 0;
    std::vector<std::string> demosName;
    std::map<std::string, std::unique_ptr<Sample>> samples;
};
}
