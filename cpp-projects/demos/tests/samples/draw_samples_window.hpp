
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

    DrawSampleWindow(std::string title, geo::Pt2<unsigned int> size, std::optional<sf::ContextSettings> context = std::nullopt);

private:

    void update() override;

    // gl
    bool initialize_gl() override;
    void draw_gl() override;    
    // imgui
    void draw_imgui() override;
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

    // drawers
    size_t currentDrawer = 0;
    std::vector<std::string> drawersName;

    // samples
    size_t currentSample = 23;
    std::vector<std::string> samplesName;
    std::vector<std::tuple<std::string, std::unique_ptr<Sample>>> samples;

    bool m_showDemoWindow = false;
    bool m_showMetricsWindow = false;
};
}
