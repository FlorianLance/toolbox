
# /*******************************************************************************
# ** tool-test                                                                  **
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
    TARGET = tool-testd
}
equals(CFG, "release"){
    TARGET = tool-test
}

####################################### TEMPLATE
equals(TOOl_TEST_TARGET, "app"){
    TEMPLATE = app
    CONFIG += console
}

####################################### BUILD FILES
OBJECTS_DIR = $$TOOL_TEST_OBJ
DESTDIR     = $$TOOL_TEST_DEST

####################################### CONFIG
CONFIG -= qt

####################################### INCLUDES
INCLUDEPATH += \
    # base
    $$BASE_INCLUDES\
    # opengl-utility
    $$OPENGL_UTILITY_INCLUDES\
    # thirdparty
    $$GLEW_INCLUDES\
    $$GLFW_INCLUDES\
    $$SFML_INCLUDES\
    $$ASSIMP_INCLUDES\
    $$GLM_INCLUDES\
    $$CATCH_INCLUDES\
    $$KINECT4_INCLUDES\
    $$TURBOJPG_INCLUDES\
    $$FASTPFOR_INCLUDES\
    $$OPEN3D_INCLUDES \
    $$AUDIOFILE_INCLUDES \
    $$LIBSOUNDIO_INCLUDES \
    $$QOI_INCLUDES\
    $$SIGNALS_INCLUDES\
    $$TURBOPFOR_INCLUDES \

####################################### LIBRAIRIES

LIBS +=\
    # base
    $$BASE_LIB\
    # opengl-utility
    $$OPENGL_UTILITY_LIB \
    # thirdparty
    $$GLEW_LIBS \
    $$GLFW_LIBS \
    $$SFML_LIBS \
    $$ASSIMP_LIBS\
    $$GLM_LIBS\
    $$KINECT4_LIBS \
    $$TURBOJPG_LIBS\
    $$FASTPFOR_LIBS\
    $$OPEN3D_LIBS \
    $$LIBSOUNDIO_LIBS \
    $$TURBOPFOR_LIBS \

####################################### PROJECT FILES

SOURCES += \
    camera_tests.cpp \
    main.cpp \       
    geometry_tests.cpp \

HEADERS += \

DISTFILES += \
    CREDITS.md \
    LICENSE \




