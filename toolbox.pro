

# /*******************************************************************************
# ** Toolbox                                                                    **
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

PROJECT_NAME = toolbox

TEMPLATE = subdirs


SUBDIRS = base opengl-utility 3d-engine qt-utility nodes tool-test demos

# where to find the sub projects
base.subdir             = cpp-projects/base
opengl-utility.subdir   = cpp-projects/opengl-utility
3d-engine.subdir        = cpp-projects/3d-engine
qt-utility.subdir       = cpp-projects/qt-utility
nodes.subdir            = cpp-projects/nodes
tool-test.subdir        = cpp-projects/tool-test
demos.subdir            = cpp-projects/demos

# dependencies
opengl-utility.depends = base
3d-engine.depends      = opengl-utility
qt-utility.depends     = opengl-utility
nodes.depends          = base
tool-test.depends      = opengl-utility
demos.depends          = 3d-engine

