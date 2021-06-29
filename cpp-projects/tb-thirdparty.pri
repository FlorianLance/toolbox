
########################################################## DIRECTORIES
############################ third-party headers only libraries
GSL_INCLUDES                         = $$TOOLBOX_CPP_THIRDPARTY_DIR"/gsl"
MAGIC_ENUM_INCLUDES                  = $$TOOLBOX_CPP_THIRDPARTY_DIR"/magic_enum/include"
SIGNALS_INCLUDES                     = $$TOOLBOX_CPP_THIRDPARTY_DIR"/signals"
TINYPLY_INCLUDES                     = $$TOOLBOX_CPP_THIRDPARTY_DIR"/tinyply"
IMGUI_INCLUDES                       = $$TOOLBOX_CPP_THIRDPARTY_DIR"/imgui-master"
CATCH_INCLUDES                       = $$TOOLBOX_CPP_THIRDPARTY_DIR"/catch"
############################ third-party others
TURBOJPG_DIR                         = $$TOOLBOX_CPP_THIRDPARTY_DIR"/libjpeg-turbo-2.0.3"
FASTPFOR_DIR                         = $$TOOLBOX_CPP_THIRDPARTY_DIR"/FastPFor"
KINECT2_DIR                          = $$TOOLBOX_CPP_THIRDPARTY_DIR"/kinect2"
KINECT4_DIR                          = $$TOOLBOX_CPP_THIRDPARTY_DIR"/kinect4"
BOOST_DIR                            = $$TOOLBOX_CPP_THIRDPARTY_DIR"/boost_1_68_0"
ASSIMP_DIR                           = $$TOOLBOX_CPP_THIRDPARTY_DIR"/assimp"
OPENCV_DIR                           = $$TOOLBOX_CPP_THIRDPARTY_DIR"/opencv-3.4.7"
GLEW_DIR                             = $$TOOLBOX_CPP_THIRDPARTY_DIR"/glew-2.1.0"
GLFW_DIR                             = $$TOOLBOX_CPP_THIRDPARTY_DIR"/glfw-3.3.2"
SFML_DIR                             = $$TOOLBOX_CPP_THIRDPARTY_DIR"/SFML"
GLM_DIR                              = $$TOOLBOX_CPP_THIRDPARTY_DIR"/glm"
BIOPAC_DIR                           = $$TOOLBOX_CPP_THIRDPARTY_DIR"/biopac"

########################################################## INCLUDES

############################ OPENCV
OPENCV_INCLUDES = \
    $$OPENCV_DIR \

############################ BOOST
BOOST_INCLUDES =\
    $$BOOST_DIR\

############################ ASSIMP
ASSIMP_INCLUDES = \
    $$ASSIMP_DIR"/include" \

############################ KINECT2
KINECT2_INCLUDES = \
        $$KINECT2_DIR"/inc"

############################ KINECT4
KINECT4_INCLUDES = \
    $$KINECT4_DIR"/include"

############################ GLM
GLM_INCLUDES = \
    $$GLM_DIR"/include" \

############################ BIOPAC
BIOPAC_INCLUDES = \
    $$BIOPAC_DIR"/include" \

############################ GLFW
GLFW_INCLUDES =\
    $$GLFW_DIR"/include"\

############################ GLEW
GLEW_INCLUDES =\
    $$GLEW_DIR"/include"\

############################ SFML
SFML_INCLUDES =\
    $$SFML_DIR"/include"\

############################ TURBOJPG
TURBOJPG_INCLUDES =\
    $$TURBOJPG_DIR"/"

############################ FASTPFOR
FASTPFOR_INCLUDES =\
    $$FASTPFOR_DIR"/"

########################################################## WINDOWS LIBS
WINDOWS_LIBS = \
    -luser32\
    -lopengl32\
    -lgdi32\
    -lshell32\
    -lws2_32\

equals(COMPILER, "vs"){

########################################################## DEBUG LIBS
    equals(CFG, "debug"){

############################ OPENCV

############################ BOOST
        BOOST_LIBS =\
            -L$$BOOST_DIR"/lib64-msvc-14.1" \
            -llibboost_date_time-vc141-mt-gd-x64-1_68 \
            -llibboost_system-vc141-mt-gd-x64-1_68 \
            -llibboost_chrono-vc141-mt-gd-x64-1_68 \
            -llibboost_python37-vc141-mt-gd-x64-1_68 \
            -llibboost_system-vc141-mt-gd-x64-1_68 \
            -llibboost_filesystem-vc141-mt-gd-x64-1_68 \
            -llibboost_iostreams-vc141-mt-gd-x64-1_68 \

############################ ASSIMP

############################ KINECT2

############################ KINECT4

############################ GLM
        GLM_LIBS = \
            -L$$GLM_DIR"/lib/Debug" \
            -lglm_static \

############################ BIOPAC

############################ GLFW
        GLFW_LIBS =\
            -L$$GLFW_DIR"/lib/Debug" \
            -lglfw3dll \

############################ GLEW

############################ SFML

############################ TURBOJPG


############################ FASTPFOR
    }

########################################################## RELEASE LIBS
    equals(CFG, "release"){

############################ OPENCV
        OPENCV_LIBS = \
            -L$$OPENCV_DIR"/lib" \
            -lopencv_world347 \

############################ BOOST
        BOOST_LIBS =\
            -L$$BOOST_DIR"/lib64-msvc-14.1" \
            -llibboost_date_time-vc141-mt-x64-1_68 \
            -llibboost_system-vc141-mt-x64-1_68 \
            -llibboost_chrono-vc141-mt-x64-1_68 \
            -llibboost_python37-vc141-mt-x64-1_68 \
            -llibboost_system-vc141-mt-x64-1_68 \
            -llibboost_filesystem-vc141-mt-x64-1_68 \
            -llibboost_iostreams-vc141-mt-x64-1_68 \

############################ ASSIMP
        ASSIMP_LIBS = \
            -L$$ASSIMP_DIR"/lib/x64" \
            -lassimp-vc141-mt \

############################ KINECT2
        KINECT2_LIBS =\
            -L$$KINECT2_DIR"/Lib/x64" \
            -lKinect20 \

############################ KINECT4
        KINECT4_LIBS =\
            -L$$KINECT4_DIR"/lib" \
            -lk4a \
            -lk4arecord \

############################ GLM
        GLM_LIBS = \
            -L$$GLM_DIR"/lib/Release" \
            -lglm_static \

############################ BIOPAC
        BIOPAC_LIBS = \
            -L$$BIOPAC_DIR"/lib/x64" \
            -lmpdev \

############################ GLFW
        GLFW_LIBS =\
            -L$$GLFW_DIR"/lib/Release" \
            -lglfw3dll \

############################ GLEW
        GLEW_LIBS =\
            -L$$GLEW_DIR"/lib/Release/x64" \
            -lglew32 \

############################ SFML
        SFML_LIBS =\
            -L$$SFML_DIR"/lib" \
            -lsfml-graphics \
            -lsfml-system \
            -lsfml-window \

############################ TURBOJPG
        TURBOJPG_LIBS =\
            -L$$TURBOJPG_DIR"/build/Release" \
            -lturbojpeg \

############################ FASTPFOR
        FASTPFOR_LIBS =\
            -L$$FASTPFOR_DIR"/msvc/x64/Release" \
            -lFastPFor \
    }
}
