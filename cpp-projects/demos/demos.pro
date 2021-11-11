
# /*******************************************************************************
# ** Toolbox-demos                                                              **
# ** MIT License                                                                **
# ** Copyright (c) [2018] [Florian Lance]                                       **
# **                                                                            **
# ** Permission is hereby granted, free of charge, to any person obtaining a    **
# ** copy of this software and associated documentation files (the "Software"), **
# ** to deal in the Software without restriction, including without limitation  **
# ** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
# ** and/or sell copies of the Software, and to permit persons to whom the      **
# ** Software is furnished to do so, subject to the following conditions:       **
# **                                                                            **
# ** The above copyright notice and this permission notice shall be included in **
# ** all copies or substantial portions of the Software.                        **
# **                                                                            **
# ** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
# ** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
# ** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
# ** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
# ** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
# ** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
# ** DEALINGS IN THE SOFTWARE.                                                  **
# **                                                                            **
# ********************************************************************************/

####################################### repo
TOOLBOX_REPOSITORY_DIR      = $$PWD"/../.."

####################################### PRI
# defines compiling options
include(../tb-settings.pri)
# defines projects paths and variables
include(../tb-projects.pri)
# defines thirdparty includes and libs
include(../tb-thirdparty.pri)

####################################### TARGET
equals(CFG, "debug"){
    TARGET = demosd
}
equals(CFG, "release"){
    TARGET = demos
}

####################################### TEMPLATE

equals(DEMOS_TARGET, "app"){
    TEMPLATE = app
    CONFIG += console
}

####################################### BUILD FILES
OBJECTS_DIR = $$DEMOS_OBJ
DESTDIR     = $$DEMOS_DEST

####################################### CONFIG
CONFIG -= qt

####################################### INCLUDES
INCLUDEPATH += \
    # base
    $$BASE_INCLUDES\
    # opengl-utility
    $$OPENGL_UTILITY_INCLUDES\
    # 3d-engine
    $$3D_ENGINE_INCLUDES\
    # thirdparty
    $$GLEW_INCLUDES\
    $$GLFW_INCLUDES\
    $$SFML_INCLUDES\
    $$ASSIMP_INCLUDES\
    $$GLM_INCLUDES\    
    $$BIOPAC_INCLUDES\
    $$SIGNALS_INCLUDES\

####################################### LIBRAIRIES

LIBS +=\
    # base
    $$BASE_LIB\
    # opengl-utility
    $$OPENGL_UTILITY_LIB \
    # 3d-engine
    $$3D_ENGINE_LIB \
    # thirdparty
    $$WINDOWS_LIBS \
    $$GLEW_LIBS \
    $$GLFW_LIBS \
    $$SFML_LIBS \
    $$ASSIMP_LIBS\    
    $$GLM_LIBS\
    $$BIOPAC_LIBS\

####################################### PROJECT FILES

HEADERS += \
    ## samples    
    tests/samples/draw_samples_window.hpp \
    tests/samples/samples.hpp \
    tests/samples/shaders_tests.hpp \
    ## biopac
    tests/biopac/biopac_control_window.hpp \
    tests/biopac/biopac.hpp \

SOURCES += \
    imgui_demo.cpp \
    main.cpp \        
    ## samples
    tests/samples/draw_samples_window.cpp \
    tests/samples/samples.cpp \
    tests/samples/shaders_tests.cpp \
    ## biopac
    tests/biopac/biopac.cpp \
    tests/biopac/biopac_control_window.cpp \

DISTFILES += \
    ../../resources/shaders/samples/ch2/1.fs \
    ../../resources/shaders/samples/ch2/1.vs \
    ../../resources/shaders/samples/ch3/diffuse.fs \
    ../../resources/shaders/samples/ch3/diffuse.vs \
    ../../resources/shaders/samples/ch3/discard.fs \
    ../../resources/shaders/samples/ch3/discard.vs \
    ../../resources/shaders/samples/ch3/flat.fs \
    ../../resources/shaders/samples/ch3/flat.vs \
    ../../resources/shaders/samples/ch3/phong.fs \
    ../../resources/shaders/samples/ch3/phong.vs \
    ../../resources/shaders/samples/ch3/twoside.fs \
    ../../resources/shaders/samples/ch3/twoside.vs \
    ../../resources/shaders/samples/ch4/blinn-phong.fs \
    ../../resources/shaders/samples/ch4/blinn-phong.vs \
    ../../resources/shaders/samples/ch4/cartoon.fs \
    ../../resources/shaders/samples/ch4/cartoon.vs \
    ../../resources/shaders/samples/ch4/pbr.fs \
    ../../resources/shaders/samples/ch4/pbr.vs \
    ../../resources/shaders/samples/ch4/phong-directional-light.fs \
    ../../resources/shaders/samples/ch4/phong-directional-light.vs \
    ../../resources/shaders/samples/ch4/phong-multi-lights.fs \
    ../../resources/shaders/samples/ch4/phong-multi-lights.vs \
    ../../resources/shaders/samples/ch4/phong-per-fragment.fs \
    ../../resources/shaders/samples/ch4/phong-per-fragment.vs \
    ../../resources/shaders/samples/ch5/diffuse-image-based-lighting.fs \
    ../../resources/shaders/samples/ch5/diffuse-image-based-lighting.vs \
    ../../resources/shaders/samples/ch5/discard-pixels.fs \
    ../../resources/shaders/samples/ch5/discard-pixels.vs \
    ../../resources/shaders/samples/ch5/normal-map.fs \
    ../../resources/shaders/samples/ch5/normal-map.vs \
    ../../resources/shaders/samples/ch5/parallax-mapping.fs \
    ../../resources/shaders/samples/ch5/parallax-mapping.vs \
    ../../resources/shaders/samples/ch5/projected-texture.fs \
    ../../resources/shaders/samples/ch5/projected-texture.vs \
    ../../resources/shaders/samples/ch5/reflect-cubemap.fs \
    ../../resources/shaders/samples/ch5/reflect-cubemap.vs \
    ../../resources/shaders/samples/ch5/refract-cubemap.fs \
    ../../resources/shaders/samples/ch5/refract-cubemap.vs \
    ../../resources/shaders/samples/ch5/render-to-texture.fs \
    ../../resources/shaders/samples/ch5/render-to-texture.vs \
    ../../resources/shaders/samples/ch5/sampler-objects.fs \
    ../../resources/shaders/samples/ch5/sampler-objects.vs \
    ../../resources/shaders/samples/ch5/scene-multi-textures.fs \
    ../../resources/shaders/samples/ch5/scene-multi-textures.vs \
    ../../resources/shaders/samples/ch5/scene-texture.fs \
    ../../resources/shaders/samples/ch5/scene-texture.vs \
    ../../resources/shaders/samples/ch5/steep-parallax-mapping.fs \
    ../../resources/shaders/samples/ch5/steep-parallax-mapping.vs \
    ../../resources/shaders/samples/ch6/deferred.fs \
    ../../resources/shaders/samples/ch6/deferred.vs \
    ../../resources/shaders/samples/ch6/edge-detection-filter.fs \
    ../../resources/shaders/samples/ch6/edge-detection-filter.vs \
    ../../resources/shaders/samples/ch6/gaussian-filter.fs \
    ../../resources/shaders/samples/ch6/gaussian-filter.vs \
    ../../resources/shaders/samples/ch6/hdr-bloom.fs \
    ../../resources/shaders/samples/ch6/hdr-bloom.vs \
    ../../resources/shaders/samples/ch6/hdr-lighting-tone-mapping.fs \
    ../../resources/shaders/samples/ch6/hdr-lighting-tone-mapping.vs \
    ../../resources/shaders/samples/ch6/oit.fs \
    ../../resources/shaders/samples/ch6/oit.vs \
    ../../resources/shaders/samples/ch6/ssao.fs \
    ../../resources/shaders/samples/ch6/ssao.vs \
    ../../resources/shaders/samples/ch7/bezcurve.fs \
    ../../resources/shaders/samples/ch7/bezcurve.tcs \
    ../../resources/shaders/samples/ch7/bezcurve.tes \
    ../../resources/shaders/samples/ch7/bezcurve.vs \
    ../../resources/shaders/samples/ch7/pointsprite.fs \
    ../../resources/shaders/samples/ch7/pointsprite.gs \
    ../../resources/shaders/samples/ch7/pointsprite.vs \
    ../../resources/shaders/samples/ch7/shadewire.fs \
    ../../resources/shaders/samples/ch7/shadewire.gs \
    ../../resources/shaders/samples/ch7/shadewire.vs \
    ../../resources/shaders/samples/ch7/silhouette.fs \
    ../../resources/shaders/samples/ch7/silhouette.gs \
    ../../resources/shaders/samples/ch7/silhouette.vs \
    ../../resources/shaders/samples/ch7/solid.fs \
    ../../resources/shaders/samples/ch7/solid.vs \
    ../../resources/shaders/samples/ch8/shadowmap.fs \
    ../../resources/shaders/samples/ch8/shadowmap.vs \
    ../../resources/shaders/samples/ch8/shadowmap2.fs \
    ../../resources/shaders/samples/ch8/shadowmap2.vs \
    ../../resources/shaders/samples/ch8/shadowvolume-comp.fs \
    ../../resources/shaders/samples/ch8/shadowvolume-comp.vs \
    ../../resources/shaders/samples/ch8/shadowvolume-render.fs \
    ../../resources/shaders/samples/ch8/shadowvolume-render.vs \
    ../../resources/shaders/samples/ch8/shadowvolume-vol.fs \
    ../../resources/shaders/samples/ch8/shadowvolume-vol.gs \
    ../../resources/shaders/samples/ch8/shadowvolume-vol.vs \
    ../../resources/shaders/samples/ch8/solid.fs \
    ../../resources/shaders/samples/ch8/solid.vs \
    ../../resources/shaders/samples/learn/3_1_1_debug_quad.fs \
    ../../resources/shaders/samples/learn/3_1_1_debug_quad.vs \
    ../../resources/shaders/samples/learn/3_1_1_shadow_mapping.fs \
    ../../resources/shaders/samples/learn/3_1_1_shadow_mapping.vs \
    ../../resources/shaders/samples/learn/3_1_1_shadow_mapping_depth.fs \
    ../../resources/shaders/samples/learn/3_1_1_shadow_mapping_depth.vs \
    ../../resources/shaders/samples/others/model.fs \
    ../../resources/shaders/samples/others/model.vs \
    ../../resources/shaders/samples/others/screen-quad.fs \
    ../../resources/shaders/samples/others/screen-quad.vs \
    ../../resources/shaders/samples/others/skybox.fs \
    ../../resources/shaders/samples/others/skybox.vs \
    ../../resources/shaders/samples/others/unicolor.fs \
    ../../resources/shaders/samples/others/unicolor.vs \
    ../../resources/shaders/samples/others/model.fs \
    ../../resources/shaders/samples/others/model.vs \
    ../../resources/shaders/samples/others/skybox.fs \
    ../../resources/shaders/samples/others/skybox.vs \
    ../../resources/shaders/samples/others/unicolor.fs \
    ../../resources/shaders/samples/others/unicolor.vs \
    ../_build/bin/demos/resources/shaders/samples/ch2/1.fs \
    ../_build/bin/demos/resources/shaders/samples/ch2/1.vs \
    ../_build/bin/demos/resources/shaders/samples/ch3/diffuse.fs \
    ../_build/bin/demos/resources/shaders/samples/ch3/diffuse.vs \
    ../_build/bin/demos/resources/shaders/samples/ch3/discard.fs \
    ../_build/bin/demos/resources/shaders/samples/ch3/discard.vs \
    ../_build/bin/demos/resources/shaders/samples/ch3/flat.fs \
    ../_build/bin/demos/resources/shaders/samples/ch3/flat.vs \
    ../_build/bin/demos/resources/shaders/samples/ch3/phong.fs \
    ../_build/bin/demos/resources/shaders/samples/ch3/phong.vs \
    ../_build/bin/demos/resources/shaders/samples/ch3/twoside.fs \
    ../_build/bin/demos/resources/shaders/samples/ch3/twoside.vs \
    ../_build/bin/demos/resources/shaders/samples/ch4/blinn-phong.fs \
    ../_build/bin/demos/resources/shaders/samples/ch4/blinn-phong.vs \
    ../_build/bin/demos/resources/shaders/samples/ch4/cartoon.fs \
    ../_build/bin/demos/resources/shaders/samples/ch4/cartoon.vs \
    ../_build/bin/demos/resources/shaders/samples/ch4/pbr.fs \
    ../_build/bin/demos/resources/shaders/samples/ch4/pbr.vs \
    ../_build/bin/demos/resources/shaders/samples/ch4/phong-directional-light.fs \
    ../_build/bin/demos/resources/shaders/samples/ch4/phong-directional-light.vs \
    ../_build/bin/demos/resources/shaders/samples/ch4/phong-multi-lights.fs \
    ../_build/bin/demos/resources/shaders/samples/ch4/phong-multi-lights.vs \
    ../_build/bin/demos/resources/shaders/samples/ch4/phong-per-fragment.fs \
    ../_build/bin/demos/resources/shaders/samples/ch4/phong-per-fragment.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/diffuse-image-based-lighting.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/diffuse-image-based-lighting.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/discard-pixels.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/discard-pixels.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/normal-map.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/normal-map.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/parallax-mapping.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/parallax-mapping.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/projected-texture.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/projected-texture.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/reflect-cubemap.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/reflect-cubemap.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/refract-cubemap.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/refract-cubemap.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/render-to-texture.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/render-to-texture.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/sampler-objects.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/sampler-objects.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/scene-multi-textures.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/scene-multi-textures.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/scene-texture.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/scene-texture.vs \
    ../_build/bin/demos/resources/shaders/samples/ch5/steep-parallax-mapping.fs \
    ../_build/bin/demos/resources/shaders/samples/ch5/steep-parallax-mapping.vs \
    ../_build/bin/demos/resources/shaders/samples/ch6/deferred.fs \
    ../_build/bin/demos/resources/shaders/samples/ch6/deferred.vs \
    ../_build/bin/demos/resources/shaders/samples/ch6/edge-detection-filter.fs \
    ../_build/bin/demos/resources/shaders/samples/ch6/edge-detection-filter.vs \
    ../_build/bin/demos/resources/shaders/samples/ch6/gaussian-filter.fs \
    ../_build/bin/demos/resources/shaders/samples/ch6/gaussian-filter.vs \
    ../_build/bin/demos/resources/shaders/samples/ch6/hdr-bloom.fs \
    ../_build/bin/demos/resources/shaders/samples/ch6/hdr-bloom.vs \
    ../_build/bin/demos/resources/shaders/samples/ch6/hdr-lighting-tone-mapping.fs \
    ../_build/bin/demos/resources/shaders/samples/ch6/hdr-lighting-tone-mapping.vs \
    ../_build/bin/demos/resources/shaders/samples/ch6/oit.fs \
    ../_build/bin/demos/resources/shaders/samples/ch6/oit.vs \
    ../_build/bin/demos/resources/shaders/samples/ch6/ssao.fs \
    ../_build/bin/demos/resources/shaders/samples/ch6/ssao.vs \
    ../_build/bin/demos/resources/shaders/samples/ch7/bezcurve.fs \
    ../_build/bin/demos/resources/shaders/samples/ch7/bezcurve.tcs \
    ../_build/bin/demos/resources/shaders/samples/ch7/bezcurve.tes \
    ../_build/bin/demos/resources/shaders/samples/ch7/bezcurve.vs \
    ../_build/bin/demos/resources/shaders/samples/ch7/pointsprite.fs \
    ../_build/bin/demos/resources/shaders/samples/ch7/pointsprite.gs \
    ../_build/bin/demos/resources/shaders/samples/ch7/pointsprite.vs \
    ../_build/bin/demos/resources/shaders/samples/ch7/shadewire.fs \
    ../_build/bin/demos/resources/shaders/samples/ch7/shadewire.gs \
    ../_build/bin/demos/resources/shaders/samples/ch7/shadewire.vs \
    ../_build/bin/demos/resources/shaders/samples/ch7/silhouette.fs \
    ../_build/bin/demos/resources/shaders/samples/ch7/silhouette.gs \
    ../_build/bin/demos/resources/shaders/samples/ch7/silhouette.vs \
    ../_build/bin/demos/resources/shaders/samples/ch7/solid.fs \
    ../_build/bin/demos/resources/shaders/samples/ch7/solid.vs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowmap.fs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowmap.vs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowvolume-comp.fs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowvolume-comp.vs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowvolume-render.fs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowvolume-render.vs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowvolume-vol.fs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowvolume-vol.gs \
    ../_build/bin/demos/resources/shaders/samples/ch8/shadowvolume-vol.vs \
    ../_build/bin/demos/resources/shaders/samples/ch8/solid.fs \
    ../_build/bin/demos/resources/shaders/samples/ch8/solid.vs \
    ../_build/bin/demos/resources/shaders/samples/learn/3_1_1_debug_quad.fs \
    ../_build/bin/demos/resources/shaders/samples/learn/3_1_1_debug_quad.vs \
    ../_build/bin/demos/resources/shaders/samples/learn/3_1_1_shadow_mapping.fs \
    ../_build/bin/demos/resources/shaders/samples/learn/3_1_1_shadow_mapping.vs \
    ../_build/bin/demos/resources/shaders/samples/learn/3_1_1_shadow_mapping_depth.fs \
    ../_build/bin/demos/resources/shaders/samples/learn/3_1_1_shadow_mapping_depth.vs \
    ../_build/bin/demos/resources/shaders/samples/others/model.fs \
    ../_build/bin/demos/resources/shaders/samples/others/model.vs \
    ../_build/bin/demos/resources/shaders/samples/others/screen-quad.fs \
    ../_build/bin/demos/resources/shaders/samples/others/screen-quad.vs \
    ../_build/bin/demos/resources/shaders/samples/others/skybox.fs \
    ../_build/bin/demos/resources/shaders/samples/others/skybox.vs \
    ../_build/bin/demos/resources/shaders/samples/others/unicolor.fs \
    ../_build/bin/demos/resources/shaders/samples/others/unicolor.vs \
    CREDITS.md \
    LICENSE \




