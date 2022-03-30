
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
    # libsoundio
    $$LIBSOUNDIO_INCLUDES \
    # libusb
    $$LIBUSB_INCLUDES \
    # eigen
    $$EIGEN_INCLUDES\
    # open3D
    $$OPEN3D_INCLUDES \
    # turbopfor
    $$TURBOPFOR_INCLUDES \

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
    # libsoundio
    $$LIBSOUNDIO_LIBS \
    # libusb
    $$LIBUSB_LIBS \
    # eigen
    $$EIGEN_LIBS\
    # open3D
    $$OPEN3D_LIBS \
    # turbopfor
    $$TURBOPFOR_LIBS \

####################################### PROJECT FILES

HEADERS += \
    # exvr    
    exvr/ex_element.hpp \
    exvr/ex_experiment.hpp \
    exvr/ex_resource.hpp \
    # files
    files/cloud_io.hpp \
    files/assimp_loader.hpp \
    # geometry
    ## shapes    
    geometry/cloud.hpp \
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
    geometry/shapes/triangle3.hpp \
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
    # input
    input/joypad.hpp \
    input/mouse.hpp \
    input/keyboard.hpp \
    # network
    network/network_utility.hpp \
    network/tcp_reader.hpp \
    network/tcp_sender.hpp \
    network/tcp_server.hpp \
    network/udp_header.hpp \
    network/udp_reader.hpp \
    network/udp_sender.hpp \
    # utility    
    utility/array.hpp \
    utility/benchmark.hpp \
    utility/constants.hpp \
    utility/export.hpp \
    utility/format.hpp \
    utility/io.hpp \
    utility/math.hpp \
    utility/paths.hpp \
    utility/string.hpp \
    utility/thread.hpp \
    utility/types.hpp \
    utility/vector.hpp \
    utility/logger.hpp \
    utility/tuple_array.hpp \
    utility/utility.hpp \
    utility/view.hpp \
    utility/time.hpp \
    # exvr
    exvr/ex_component.hpp \
    # algorithms
    algorithms/marching_cube.hpp \
    # tests
    tests/marching_cube_test.hpp \
    # camera
    camera/frame.hpp \
    ## kinect2
    camera/kinect2/k2_config_files.hpp \
    camera/kinect2/k2_device.hpp \
    camera/kinect2/k2_manager.hpp \
    camera/kinect2/k2_network.hpp \
    camera/kinect2/k2_types.hpp \
    ## kinect4
    camera/kinect4/k4_data.hpp \
    camera/kinect4/k4_device.hpp \
    camera/kinect4/k4_frame.hpp \
    camera/kinect4/k4_frame_compressor.hpp \
    camera/kinect4/k4_frame_uncompressor.hpp \
    camera/kinect4/k4_network.hpp \
    camera/kinect4/k4_types.hpp \
    camera/kinect4/k4_volumetric_cloud_video_manager.hpp \
    camera/kinect4/k4_volumetric_cloud_video_resource.hpp \
    camera/kinect4/k4_volumetric_full_video_manager.hpp \
    camera/kinect4/k4_volumetric_full_video_resource.hpp \
    camera/kinect4/k4_volumetric_video_resource.hpp \
    camera/kinect4/k4a/k4aaudiochanneldatagraph.h \
    camera/kinect4/k4a/k4aaudiomanager.h \
    camera/kinect4/k4a/k4aaudiowindow.h \
    camera/kinect4/k4a/k4adevicecorrelator.h \
    camera/kinect4/k4a/k4amicrophone.h \
    camera/kinect4/k4a/k4amicrophonelistener.h \
    camera/kinect4/k4a/k4asoundio_util.h \
    camera/kinect4/k4a/k4astaticimageproperties.h \
    # data
    data/FastDifferentialCoding/fastdelta.h \
    data/integers_encoder.hpp \
    # thirdparty
    ## stb
    thirdparty/stb/stb_image.h \
    thirdparty/stb/stb_image_resize.h \
    thirdparty/stb/stb_image_write.h \

SOURCES += \
    # main
    base_main.cpp \
    # exvr    
    exvr/ex_component.cpp \
    exvr/ex_element.cpp \
    exvr/ex_experiment.cpp \
    exvr/ex_resource.cpp \
    # files
    files/assimp_loader.cpp \
    files/cloud_io.cpp \
    # graphics
    graphics/texture.cpp \
    # utility    
    utility/benchmark.cpp \
    utility/logger.cpp \
    utility/paths.cpp \
    # camera
    camera/frame.cpp \
    ## kinect2
    camera/kinect2/k2_config_files.cpp \
    camera/kinect2/k2_device.cpp \
    camera/kinect2/k2_manager.cpp \
    camera/kinect2/k2_types.cpp \
    ## kinect4
    camera/kinect4/k4a/k4aaudiochanneldatagraph.cpp \
    camera/kinect4/k4a/k4aaudiomanager.cpp \
    camera/kinect4/k4a/k4aaudiowindow.cpp \
    camera/kinect4/k4a/platform/windows/k4adevicecorrelator.cpp \
    camera/kinect4/k4a/k4amicrophone.cpp \
    camera/kinect4/k4a/k4amicrophonelistener.cpp \
    camera/kinect4/k4_data.cpp \
    camera/kinect4/k4_device.cpp \
    camera/kinect4/k4_frame_compressor.cpp \
    camera/kinect4/k4_frame_uncompressor.cpp \
    camera/kinect4/k4_network.cpp \
    camera/kinect4/k4_volumetric_cloud_video_manager.cpp \
    camera/kinect4/k4_volumetric_cloud_video_resource.cpp \
    camera/kinect4/k4_volumetric_full_video_manager.cpp \
    camera/kinect4/k4_volumetric_full_video_resource.cpp \
    camera/kinect4/k4_volumetric_video_resource.cpp \
    # network
    network/network_utility.cpp \
    network/tcp_reader.cpp \
    network/tcp_sender.cpp \
    network/tcp_server.cpp \
    network/udp_header.cpp \
    network/udp_reader.cpp \
    network/udp_sender.cpp \
    # data
    data/FastDifferentialCoding/fastdelta.c \
    data/integers_encoder.cpp \
    # thirdparty
    ## stb
    thirdparty/stb/stb_image.cpp \
    thirdparty/stb/stb_image_resize.cpp \
    thirdparty/stb/stb_image_write.cpp \


#DISTFILES += \

