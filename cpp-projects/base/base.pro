
# /*******************************************************************************
# ** Toolbox-base                                                               **
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
    TARGET = based
}
equals(CFG, "release"){
    TARGET = base
}

####################################### TEMPLATE
equals(BASE_TARGET, "lib"){
    TEMPLATE = lib
    CONFIG += staticlib
    CONFIG -= console
}
equals(BASE_TARGET, "app"){
    TEMPLATE = app
    CONFIG += console
}

####################################### BUILD FILES
OBJECTS_DIR = $$BASE_OBJ
DESTDIR     = $$BASE_DEST

####################################### CONFIG
CONFIG -= qt

####################################### INCLUDES
INCLUDEPATH += \
    # signals
    $$SIGNALS_INCLUDES \
    # assimp
    $$ASSIMP_INCLUDES \
    # catch
    $$CATCH_INCLUDES \
    # turbojpg
    $$TURBOJPG_INCLUDES \
    # fastfor
    $$FASTPFOR_INCLUDES \
    # opencv
    $$OPENCV_INCLUDES \
    # kinect2
    $$KINECT2_INCLUDES \
    # kinect4
    $$KINECT4_INCLUDES \
    # boost
    $$BOOST_INCLUDES \

####################################### LIBRAIRIES
LIBS += \
    # assimp
    $$ASSIMP_LIBS \
    # turbojpg
    $$TURBOJPG_LIBS \
    # fastfor
    $$FASTPFOR_LIBS \
    # opencv
    $$OPENCV_LIBS \
    # kinect2
    $$KINECT2_LIBS \
    # kinect4
    $$KINECT4_LIBS \
    # boost
    $$BOOST_LIBS \

####################################### PROJECT FILES

HEADERS += \
    # files    
    camera/kinect2_settings_files.hpp \
    files/assimp_loader.hpp \
    # geometry
    ## shapes
    geometry/geometry.hpp \
    geometry/transform.hpp \
    geometry/shapes/aabb3.hpp \
    geometry/shapes/circle.hpp \
    geometry/shapes/obb3.hpp \
    geometry/shapes/plane3.hpp \
    geometry/shapes/line2.hpp \
    geometry/shapes/line3.hpp \
    geometry/shapes/rectangle.hpp \
    geometry/shapes/sphere.hpp \
    geometry/shapes/ray3.hpp \
    geometry/geometry2.hpp \
    geometry/geometry3.hpp \
    geometry/interval.hpp \
    geometry/matrix.hpp \
    geometry/matrix2.hpp \
    geometry/matrix3.hpp \
    geometry/matrix4.hpp \
    geometry/mesh.hpp \
    geometry/point.hpp \
    geometry/point2.hpp \
    geometry/point3.hpp \
    geometry/point4.hpp \
    geometry/raycast.hpp \
    geometry/triangle3.hpp \
    geometry/dummy.hpp \
    geometry/aabb2.hpp \
    geometry/octree.hpp \
    geometry/maching_cube.hpp \
    geometry/voxel.hpp \
    geometry/quaternion.hpp \
    # graphics
    graphics/screen.hpp \
    graphics/light.hpp \
    graphics/material.hpp \
    graphics/texture.hpp \
    graphics/model.hpp \
    graphics/camera.hpp \
    # network
    # utility
    network/network_interface.hpp \
    utility/benchmark.hpp \
    utility/joypad_utility.hpp \
    utility/logger.hpp \
    utility/mouse_utility.hpp \
    utility/thread_utility.hpp \
    utility/tuple_array.hpp \
    utility/utility.hpp \
    utility/view.hpp \
    utility/time.hpp \
    utility/files.hpp \
    utility/vector_utility.hpp \
    utility/types_utility.hpp \
    utility/array_utility.hpp \
    utility/constants_utility.hpp \
    utility/maths_utility.hpp \
    utility/string_utility.hpp \
    utility/keyboard_utility.hpp \    
    # exvr
    exvr/ex_component.hpp \
    # algorithms
    algorithms/marching_cube.hpp \
    # tests
    tests/marching_cube_test.hpp \
    # camera
    camera/kinect2_data_types.hpp \
    camera/kinect2_network_types.hpp \
    camera/kinect4_data_types.hpp \
    camera/k4a/k4astaticimageproperties.h \
    camera/kinect4.hpp \    
    camera/kinect2.hpp \
    camera/kinect2_manager.hpp \
    # data
    data/integers_encoder.hpp \
    # thirdparty
    ## stb
    thirdparty/stb/stb_image.h \
    thirdparty/stb/stb_image_resize.h \
    thirdparty/stb/stb_image_write.h \

SOURCES += \
    base_main.cpp \
    # files        
    camera/kinect2_settings_files.cpp \
    files/assimp_loader.cpp \
    # graphics
    graphics/texture.cpp \
    # utility    
    utility/benchmark.cpp \
    utility/logger.cpp \
    # camera
    camera/kinect2_data_types.cpp \
    camera/kinect4.cpp \
    camera/kinect2_manager.cpp \
    camera/kinect2.cpp \
    # network
    network/network_interface.cpp \
    # data
    data/integers_encoder.cpp \
    # thirdparty
    ## stb
    thirdparty/stb/stb_image.cpp \
    thirdparty/stb/stb_image_resize.cpp \
    thirdparty/stb/stb_image_write.cpp \


#DISTFILES += \

