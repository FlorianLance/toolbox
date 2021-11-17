
# /*******************************************************************************
# ** exvr                                                                       **
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



# lib
## base
#BASE_TARGET          = "app" # debug
BASE_TARGET          = "lib"
## opengl-utility
#OPENGL_UTILITY_TARGET = "app" # debug
OPENGL_UTILITY_TARGET = "lib"
## 3d-engine
#3D_ENGINE_TARGET     = "app" # debug
3D_ENGINE_TARGET     = "lib"
## nodes
#NODES_TARGET         = "app" # debug
NODES_TARGET         = "lib"
## qt-utility
#QT_UTILITY_TARGET    = "app" # debug
QT_UTILITY_TARGET    = "lib"

# app
## tool-test
TOOl_TEST_TARGET     = "app"
## demos
DEMOS_TARGET         = "app"

# projects list
TOOLBOX_CPP_PROJECTS +=\
    BASE:base:based \
    OPENGL_UTILITY:opengl-utility:opengl-utilityd \
    3D_ENGINE:3d-engine:3d-engined \
    DEMOS:demos:demosd \
    QT_UTILITY:qt-utility:qt-utilityd \
    NODES:nodes:nodesd \
    TOOL_TEST:tool-test:tool-testd \

# define functions
defineTest(generate_variables) {

    # include
    eval($${2}_INCLUDES = $${1}/$${3})
    eval(export($${2}_INCLUDES))

    # objects files
    eval($${2}_OBJ = $${1}/_build/temp/obj/$${CFG}/$${3})
    eval(export($${2}_OBJ))

    # moc
    eval($${2}_MOC = $${1}/_build/temp/moc/$${CFG}/$${3})
    eval(export($${2}_MOC))

    # rcc
    eval($${2}_RCC = $${1}/_build/temp/rcc/$${CFG}/$${3})
    eval(export($${2}_RCC))

    # ui generated files
    eval($${2}_UI = $${1}/_build/temp/ui/$${CFG}/$${3})
    eval(export($${2}_UI))

    # destination
    eval($${2}_DEST = $${1}/_build/bin/$${3})
    eval(export($${2}_DEST))

    # lib
    equals(CFG, "debug"){
        eval($${2}_LIB = "-L"$${1}/_build/bin/$${3} "-l"$${4})
    }
    equals(CFG, "release"){
        eval($${2}_LIB = "-L"$${1}/_build/bin/$${3} "-l"$${3})
    }
    eval(export($${2}_LIB))
}

# sub dir
TOOLBOX_CPP_PROJECTS_DIR    = $$TOOLBOX_REPOSITORY_DIR"/cpp-projects"
TOOLBOX_CPP_THIRDPARTY_DIR  = $$TOOLBOX_CPP_PROJECTS_DIR"/_thirdparty"
TOOLBOX_CPP_RESOURCES_DIR   = $$TOOLBOX_CPP_PROJECTS_DIR"/_resources"

# generate projects variables
for(project_dir, TOOLBOX_CPP_PROJECTS):{
    generate_variables($$TOOLBOX_CPP_PROJECTS_DIR, $$section(project_dir, :, 0, 0), $$section(project_dir, :, 1, 1), $$section(project_dir, :, 2, 2))
}
