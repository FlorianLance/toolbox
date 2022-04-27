
# /*******************************************************************************
# ** Toolbox-3d-engine                                                          **
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
    TARGET = 3d-engined
}
equals(CFG, "release"){
    TARGET = 3d-engine
}

####################################### TEMPLATE
equals(3D_ENGINE_TARGET, "lib"){
    TEMPLATE = lib
    CONFIG += staticlib
    CONFIG -= console
}
equals(3D_ENGINE_TARGET, "app"){
    TEMPLATE = app
    CONFIG += console
}

####################################### BUILD FILES
OBJECTS_DIR = $$3D_ENGINE_OBJ
DESTDIR     = $$3D_ENGINE_DEST

####################################### CONFIG
CONFIG -= qt

####################################### INCLUDES
INCLUDEPATH += \
    # base
    $$BASE_INCLUDES\
    # opengl-utility
    $$OPENGL_UTILITY_INCLUDES\
    # local
    $$3D_ENGINE_INCLUDES"/imgui"\
    # thirdparty
    $$GLEW_INCLUDES\
    $$SFML_INCLUDES\
    $$ASSIMP_INCLUDES\
    $$SIGNALS_INCLUDES\
    $$SIGSLOT_INCLUDES \
    $$KINECT4_INCLUDES \

####################################### LIBRAIRIES

LIBS +=\
    # base
    $$BASE_LIB\
    # opengl-utility
    $$OPENGL_UTILITY_LIB \    
    # thirdparty
    $$WINDOWS_LIBS \
    $$GLEW_LIBS \
    $$SFML_LIBS \
    $$ASSIMP_LIBS\
    $$KINECT4_LIBS \

####################################### PROJECT FILES

HEADERS += \
    # ui
    imgui-utility/imgui_k4_drawers.hpp \
    imgui-utility/imgui_logs.hpp \
    imgui-utility/imgui_texture.hpp \
    immediate_ui.hpp \
    # resources
    resources.hpp \
    # engine
    engine/drawers_manager.hpp \
    engine/managers.hpp \
    engine/models_manager.hpp \
    engine/shaders_manager.hpp \
    engine/textures_manager.hpp \
    # windows
    windows/base_sfml_gl_window.hpp \
    # imgui
    imgui/imgui.h \
    imgui/imconfig.h \
    imgui/imgui_internal.h \
    imgui/imstb_rectpack.h \
    imgui/imstb_textedit.h \
    imgui/imstb_truetype.h \
    imgui/misc/cpp/imgui_stdlib.h \
    # imgui-extra
    imgui/extra/imgui_markdown.h \
    imgui/extra/implot/implot.h \
    imgui/extra/implot/implot_internal.h \
    imgui/extra/imgui-node-editor/crude_json.h \
    imgui/extra/imgui-node-editor/imgui_bezier_math.h \
    imgui/extra/imgui-node-editor/imgui_bezier_math.inl \
    imgui/extra/imgui-node-editor/imgui_canvas.h \
    imgui/extra/imgui-node-editor/imgui_extra_math.h \
    imgui/extra/imgui-node-editor/imgui_extra_math.inl \
    imgui/extra/imgui-node-editor/imgui_node_editor.h \
    imgui/extra/imgui-node-editor/imgui_node_editor_internal.h \
    imgui/extra/imgui-node-editor/imgui_node_editor_internal.inl \
    imgui/extra/ImGuiFileDialog.h \
    imgui/extra/ImGuiFileDialogConfig.h \
    # imgui-sfml
    imgui-sfml/imgui-SFML.h \
    imgui-sfml/imgui-SFML_export.h \
    # imgui-utility
    imgui-utility/imgui_fbo.hpp \
    imgui-utility/imgui_std.hpp \


SOURCES += \
    # main
    imgui-utility/imgui_k4_drawers.cpp \
    imgui-utility/imgui_logs.cpp \
    imgui-utility/imgui_texture.cpp \
    main.cpp \
    # resources
    resources/resources.cpp \
    # engine
    engine/drawers_manager.cpp \
    engine/models_manager.cpp \
    engine/shaders_manager.cpp \
    engine/textures_manager.cpp \
    # windows
    windows/base_sfml_gl_window.cpp \
    # imgui
    imgui/imgui.cpp \
    imgui/imgui_draw.cpp \
    imgui/imgui_widgets.cpp \
    imgui/imgui_tables.cpp \
    imgui/misc/cpp/imgui_stdlib.cpp \
    # imgui-extra
    imgui/extra/implot/implot.cpp \
    imgui/extra/implot/implot_items.cpp \
    imgui/extra/imgui-node-editor/crude_json.cpp \
    imgui/extra/imgui-node-editor/imgui_canvas.cpp \
    imgui/extra/imgui-node-editor/imgui_node_editor.cpp \
    imgui/extra/imgui-node-editor/imgui_node_editor_api.cpp \    
    imgui/extra/ImGuiFileDialog.cpp \
    # imgui-sfml
    imgui-sfml/imgui-SFML.cpp \
    # imgui-utility
    imgui-utility/imgui_fbo.cpp \

#DISTFILES += \



