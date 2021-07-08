
# /*******************************************************************************
# ** Toolbox-qt-utility                                                         **
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
    TARGET = qt-utilityd
}
equals(CFG, "release"){
    TARGET = qt-utility
}

####################################### TEMPLATE
equals(QT_UTILITY_TARGET, "lib"){
    TEMPLATE = lib
    CONFIG += staticlib
    CONFIG -= console
}
equals(QT_UTILITY_TARGET, "app"){
    TEMPLATE = app
    CONFIG += console
}

####################################### BUILD FILES
OBJECTS_DIR = $$QT_UTILITY_OBJ
MOC_DIR     = $$QT_UTILITY_MOC
RCC_DIR     = $$QT_UTILITY_RCC
UI_DIR      = $$QT_UTILITY_UI
DESTDIR     = $$QT_UTILITY_DEST

####################################### CONFIG
CONFIG += qt
QT += core gui opengl widgets printsupport network
DEFINES += QWT_DLL

####################################### INCLUDES
INCLUDEPATH += \    
    # local
    widgets \
    ex_widgets \
    ex_widgets/items \
    ex_widgets/base \
    ex_widgets/generation \
    data \
    # base
    $$BASE_INCLUDES \
    # opengl-utility
    $$OPENGL_UTILITY_INCLUDES \
    # thirdparty
    $$SIGNALS_INCLUDES \
    $$SFML_INCLUDES \
    $$GLEW_INCLUDES \
    $$GLM_INCLUDES \
    $$QWT_INCLUDES \

####################################### LIBRAIRIES
LIBS +=  \
    # base
    $$BASE_LIB\
    # opengl-utility
    $$OPENGL_UTILITY_LIB \
    # third party
    $$WINDOWS_LIBS \
    $$SFML_LIBS \
    $$GLEW_LIBS \
    $$QWT_LIBS \
    $$GLM_LIBS\

####################################### PROJECT FILES

SOURCES += \
    data/argument.cpp \
    data/id_key.cpp \
    ex_widgets/items/ex_checkbox_w.cpp \
    ex_widgets/items/ex_color_frame_w.cpp \
    ex_widgets/items/ex_combo_box_index_w.cpp \
    ex_widgets/items/ex_combo_box_text_w.cpp \
    ex_widgets/items/ex_curve_w.cpp \
    ex_widgets/items/ex_curve_x_w.cpp \
    ex_widgets/items/ex_double_spin_box_w.cpp \
    ex_widgets/items/ex_float_spin_box_w.cpp \
    ex_widgets/items/ex_label_w.cpp \
    ex_widgets/items/ex_line_edit_w.cpp \
    ex_widgets/items/ex_list_labels_w.cpp \
    ex_widgets/items/ex_pushbutton_w.cpp \
    ex_widgets/items/ex_radio_button_w.cpp \
    ex_widgets/items/ex_select_color_w.cpp \
    ex_widgets/items/ex_slider_w.cpp \
    ex_widgets/items/ex_spin_box_w.cpp \
    ex_widgets/items/ex_text_edit_w.cpp \
    ex_widgets/items/ex_transformation_w.cpp \
    ex_widgets/items/ex_vector2d_w.cpp \
    ex_widgets/items/ex_vector3d_w.cpp \
    qt_logger.cpp \
    qt_str.cpp \
    qt_ui.cpp \
    qt_utility_main.cpp \
    widgets/grabber_parameters_widget.cpp \
    widgets/image_viewer.cpp \
    widgets/list_widget.cpp \
    widgets/rich_text_edit.cpp \
    widgets/sfmlqt_gl_widget.cpp \
    widgets/curve_widget.cpp  \

HEADERS += \
    data/argument.hpp \
    data/ex_widgets_types.hpp \
    data/id_key.hpp \
    data/unity_types.hpp \
    ex_widgets/base/ex_base_w.hpp \
    ex_widgets/base/ex_item_w.hpp \
    ex_widgets/items/ex_checkbox_w.hpp \
    ex_widgets/items/ex_color_frame_w.hpp \
    ex_widgets/items/ex_combo_box_index_w.hpp \
    ex_widgets/items/ex_combo_box_text_w.hpp \
    ex_widgets/items/ex_curve_w.hpp \
    ex_widgets/items/ex_curve_x_w.hpp \
    ex_widgets/items/ex_double_spin_box_w.hpp \
    ex_widgets/items/ex_float_spin_box_w.hpp \
    ex_widgets/items/ex_label_w.hpp \
    ex_widgets/items/ex_line_edit_w.hpp \
    ex_widgets/items/ex_list_labels_w.hpp \
    ex_widgets/items/ex_pushbutton_w.hpp \
    ex_widgets/items/ex_radio_button_w.hpp \
    ex_widgets/items/ex_select_color_w.hpp \
    ex_widgets/items/ex_slider_w.hpp \
    ex_widgets/items/ex_spin_box_w.hpp \
    ex_widgets/items/ex_text_edit_w.hpp \
    ex_widgets/items/ex_transformation_w.hpp \
    ex_widgets/items/ex_vector2d_w.hpp \
    ex_widgets/items/ex_vector3d_w.hpp \
    qt_io.hpp \
    qt_logger.hpp \
    qt_process.hpp \
    qt_str.hpp \
    qt_types.hpp \
    qt_ui.hpp \
    qt_convertors.hpp \
    widgets/custom_combobox_w.hpp \
    widgets/curve_widget.hpp \
    widgets/grabber_parameters_widget.hpp \
    widgets/list_widget.hpp \
    widgets/image_viewer.hpp \
    widgets/rich_text_edit.hpp \
    widgets/sfmlqt_gl_widget.hpp \
    wrapper/qt_gl_wrapper.hpp \

RESOURCES += \
    $$TOOLBOX_CPP_RESOURCES_DIR"/resources.qrc" \

FORMS += \
    ui/grabber_parameters.ui \


