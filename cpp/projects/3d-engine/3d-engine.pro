
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

####################################### TARGET
TARGET = 3d-engine

####################################### PRI
# defines projects settings
include(../../projects.pri)
# defines compiling options
include(../settings.pri)
# generate paths
include(../paths.pri)
# defines thirdparty includes and libs
include(../thirdparty.pri)

####################################### TEMPLATE

equals(3D_ENGINE_T, "lib"){
    TEMPLATE = lib
    CONFIG += staticlib
    CONFIG -= console
}
equals(3D_ENGINE_T, "app"){
    TEMPLATE = app
    CONFIG += console
}

OBJECTS_DIR = $$3D_ENGINE_OBJ
DESTDIR     = $$3D_ENGINE_DEST

####################################### CONFIG
CONFIG -= qt

####################################### INCLUDES
INCLUDEPATH += \
    # thirdparty
    $$GLEW_INCLUDES\
    $$SFML_INCLUDES\
    $$ASSIMP_INCLUDES\
    # base
    $$BASE_INCLUDES\
    # opengl-utility
    $$OPENGL_UTILITY_INCLUDES\
    # local
    $$3D_ENGINE_INCLUDES"/imgui"\


####################################### LIBRAIRIES

LIBS +=\
    # thirdparty
    $$WINDOWS_LIBS \
    $$GLEW_LIBS \
    $$SFML_LIBS \
    $$ASSIMP_LIBS\
    # base
    $$BASE_LIB\
    # opengl-utility
    $$OPENGL_UTILITY_LIB \


####################################### PROJECT FILES

HEADERS += \
    # ui
    imgui/misc/cpp/imgui_stdlib.h \
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
    imgui/extra/imgui_markdown.h \
    imgui/extra/implot/implot.h \
    imgui/extra/implot/implot_internal.h \
    # imgui-extra
    imgui/extra/imgui-node-editor/crude_json.h \
    imgui/extra/imgui-node-editor/imgui_bezier_math.h \
    imgui/extra/imgui-node-editor/imgui_bezier_math.inl \
    imgui/extra/imgui-node-editor/imgui_canvas.h \
    imgui/extra/imgui-node-editor/imgui_extra_math.h \
    imgui/extra/imgui-node-editor/imgui_extra_math.inl \
    imgui/extra/imgui-node-editor/imgui_node_editor.h \
    imgui/extra/imgui-node-editor/imgui_node_editor_internal.h \
    imgui/extra/imgui-node-editor/imgui_node_editor_internal.inl \
    # imgui-sfml
    imgui-sfml/imgui-SFML.h \
    imgui-sfml/imgui-SFML_export.h \
    # imgui-utility
    imgui/imgui_utility.hpp \

SOURCES += \
    # main
    imgui/misc/cpp/imgui_stdlib.cpp \
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
    # imgui-extra
    imgui/extra/implot/implot.cpp \
    imgui/extra/implot/implot_items.cpp \
    imgui/extra/imgui-node-editor/crude_json.cpp \
    imgui/extra/imgui-node-editor/imgui_canvas.cpp \
    imgui/extra/imgui-node-editor/imgui_node_editor.cpp \
    imgui/extra/imgui-node-editor/imgui_node_editor_api.cpp \
    # imgui-sfml
    imgui-sfml/imgui-SFML.cpp \


#DISTFILES += \



