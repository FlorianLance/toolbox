
# /*******************************************************************************
# ** Toolbox-node                                                               **
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
    TARGET = nodesd
}
equals(CFG, "release"){
    TARGET = nodes
}

####################################### TEMPLATE
equals(NODES_TARGET, "lib"){
    TEMPLATE = lib
    CONFIG += staticlib
    CONFIG -= console
}
equals(NODES_TARGET, "app"){
    TEMPLATE = app
    CONFIG += console
}

####################################### BUILD FILES
OBJECTS_DIR = $$NODES_OBJ
MOC_DIR     = $$NODES_MOC
RCC_DIR     = $$NODES_RCC
UI_DIR      = $$NODES_UI
DESTDIR     = $$NODES_DEST

####################################### CONFIG

CONFIG += qt
QT += core gui opengl widgets printsupport

####################################### INCLUDES

INCLUDEPATH += \
    # base
    $$BASE_INCLUDES \
    # local
    nodes\
    nodes/internal\   
    # sigslot
    $$SIGSLOT_INCLUDES \

####################################### LIBRAIRIES

LIBS +=  \
    # base
    $$BASE_LIB\

####################################### PROJECT FILES

SOURCES += \
    nodes/Connection.cpp \
    nodes/ConnectionBlurEffect.cpp \
    nodes/ConnectionGeometry.cpp \
    nodes/ConnectionGraphicsObject.cpp \
    nodes/ConnectionPainter.cpp \
    nodes/ConnectionState.cpp \
    nodes/ConnectionStyle.cpp \
    nodes/DataModelRegistry.cpp \
    nodes/FlowScene.cpp \
    nodes/FlowView.cpp \
    nodes/FlowViewStyle.cpp \
    nodes/Node.cpp \
    nodes/NodeConnectionInteraction.cpp \
    nodes/NodeDataModel.cpp \
    nodes/NodeGeometry.cpp \
    nodes/NodeGraphicsObject.cpp \
    nodes/NodePainter.cpp \
    nodes/NodeState.cpp \
    nodes/NodeStyle.cpp \
    nodes/Properties.cpp \
    nodes/StyleCollection.cpp \
    sample_main.cpp \
    samples/calculator/Converters.cpp \
    samples/calculator/MathOperationDataModel.cpp \
    samples/calculator/ModuloModel.cpp \
    samples/calculator/NumberDisplayDataModel.cpp \
    samples/calculator/NumberSourceDataModel.cpp \
    samples/connection_colors/connections_colors_models.cpp \
    samples/example2/TextDisplayDataModel.cpp \
    samples/example2/TextSourceDataModel.cpp \
    samples/images/ImageLoaderModel.cpp \
    samples/images/ImageShowModel.cpp \
    samples/styles/styles_models.cpp


HEADERS += \
    nodes/ConnectionBlurEffect.hpp \
    nodes/ConnectionPainter.hpp \
    nodes/NodeConnectionInteraction.hpp \
    nodes/Properties.hpp \
    nodes/StyleCollection.hpp \
    nodes/Compiler.hpp \
    nodes/Connection.hpp \
    nodes/ConnectionGeometry.hpp \
    nodes/ConnectionGraphicsObject.hpp \
    nodes/ConnectionState.hpp \
    nodes/ConnectionStyle.hpp \
    nodes/DataModelRegistry.hpp \
    nodes/Export.hpp \
    nodes/FlowScene.hpp \
    nodes/FlowView.hpp \
    nodes/FlowViewStyle.hpp \
    nodes/Node.hpp \
    nodes/NodeData.hpp \
    nodes/NodeDataModel.hpp \
    nodes/NodeGeometry.hpp \
    nodes/NodeGraphicsObject.hpp \
    nodes/NodePainterDelegate.hpp \
    nodes/NodeState.hpp \
    nodes/NodeStyle.hpp \
    nodes/OperatingSystem.hpp \
    nodes/PortType.hpp \
    nodes/QUuidStdHash.hpp \
    nodes/Serializable.hpp \
    nodes/Style.hpp \
    nodes/TypeConverter.hpp \
    nodes/memory.hpp \
    samples/calculator/AdditionModel.hpp \
    samples/calculator/Converters.hpp \
    samples/calculator/DecimalData.hpp \
    samples/calculator/DivisionModel.hpp \
    samples/calculator/IntegerData.hpp \
    samples/calculator/MathOperationDataModel.hpp \
    samples/calculator/ModuloModel.hpp \
    samples/calculator/MultiplicationModel.hpp \
    samples/calculator/NumberDisplayDataModel.hpp \
    samples/calculator/NumberSourceDataModel.hpp \
    samples/calculator/SubtractionModel.hpp \
    samples/calculator/calculator.hpp \
    samples/connection_colors/connection_colors.hpp \
    samples/connection_colors/connections_colors_models.hpp \
    samples/example2/TextData.hpp \
    samples/example2/TextDisplayDataModel.hpp \
    samples/example2/TextSourceDataModel.hpp \
    samples/example2/example2.hpp \
    samples/images/ImageLoaderModel.hpp \
    samples/images/ImageShowModel.hpp \
    samples/images/PixmapData.hpp \
    samples/images/images.hpp \
    samples/styles/styles.hpp \
    samples/styles/styles_models.hpp


RESOURCES += \
    resources/nodes_resource.qrc \

DISTFILES += \
    LICENSE \


