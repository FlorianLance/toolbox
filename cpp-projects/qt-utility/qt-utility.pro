
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
    $$QT_UTILITY_MOC \
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

HEADERS += \
    # global
    qt_io.hpp \
    qt_logger.hpp \
    qt_process.hpp \
    qt_str.hpp \
    qt_types.hpp \
    qt_ui.hpp \
    qt_convertors.hpp \
    # data
    data/argument.hpp \
    data/ex_widgets_types.hpp \
    data/id_key.hpp \
    data/unity_types.hpp \
    # widgets
    widgets/code_editor_w.hpp \
    widgets/custom_combobox_w.hpp \
    widgets/curve_widget.hpp \
    widgets/grabber_parameters_widget.hpp \
    widgets/list_widget.hpp \
    widgets/image_viewer.hpp \
    widgets/rich_text_edit.hpp \
    widgets/sfmlqt_gl_widget.hpp \
    widgets/text_widget_highlighter.hpp \
    wrapper/qt_gl_wrapper.hpp \
    # ex_widgets
    ex_widgets/ex_base_w.hpp \
    ex_widgets/ex_item_w.hpp \
    ex_widgets/ex_checkbox_w.hpp \
    ex_widgets/ex_code_editor_w.hpp \
    ex_widgets/ex_color_frame_w.hpp \
    ex_widgets/ex_combo_box_index_w.hpp \
    ex_widgets/ex_combo_box_text_w.hpp \
    ex_widgets/ex_curve_w.hpp \
    ex_widgets/ex_curve_x_w.hpp \
    ex_widgets/ex_double_spin_box_w.hpp \
    ex_widgets/ex_float_spin_box_w.hpp \
    ex_widgets/ex_label_w.hpp \
    ex_widgets/ex_line_edit_w.hpp \
    ex_widgets/ex_list_labels_w.hpp \
    ex_widgets/ex_pushbutton_w.hpp \
    ex_widgets/ex_radio_button_w.hpp \
    ex_widgets/ex_select_color_w.hpp \
    ex_widgets/ex_slider_w.hpp \
    ex_widgets/ex_spin_box_w.hpp \
    ex_widgets/ex_text_edit_w.hpp \
    ex_widgets/ex_transformation_w.hpp \
    ex_widgets/ex_vector2d_w.hpp \
    ex_widgets/ex_vector3d_w.hpp \
    # wrapper
    wrapper/qt_gl_wrapper.hpp \

SOURCES += \
    # global
    qt_logger.cpp \
    qt_str.cpp \
    qt_ui.cpp \
    qt_utility_main.cpp \
    # data
    data/argument.cpp \
    data/id_key.cpp \
    # widgets
    widgets/code_editor_w.cpp \
    widgets/grabber_parameters_widget.cpp \
    widgets/image_viewer.cpp \
    widgets/list_widget.cpp \
    widgets/rich_text_edit.cpp \
    widgets/sfmlqt_gl_widget.cpp \
    widgets/curve_widget.cpp  \
    widgets/text_widget_highlighter.cpp \
    # ex_widgets
    ex_widgets/ex_base_w.cpp \
    ex_widgets/ex_checkbox_w.cpp \
    ex_widgets/ex_code_editor_w.cpp \
    ex_widgets/ex_color_frame_w.cpp \
    ex_widgets/ex_combo_box_index_w.cpp \
    ex_widgets/ex_combo_box_text_w.cpp \
    ex_widgets/ex_curve_w.cpp \
    ex_widgets/ex_curve_x_w.cpp \
    ex_widgets/ex_double_spin_box_w.cpp \
    ex_widgets/ex_float_spin_box_w.cpp \
    ex_widgets/ex_label_w.cpp \
    ex_widgets/ex_line_edit_w.cpp \
    ex_widgets/ex_list_labels_w.cpp \
    ex_widgets/ex_pushbutton_w.cpp \
    ex_widgets/ex_radio_button_w.cpp \
    ex_widgets/ex_select_color_w.cpp \
    ex_widgets/ex_slider_w.cpp \
    ex_widgets/ex_spin_box_w.cpp \
    ex_widgets/ex_text_edit_w.cpp \
    ex_widgets/ex_transformation_w.cpp \
    ex_widgets/ex_vector2d_w.cpp \
    ex_widgets/ex_vector3d_w.cpp \

RESOURCES += \
    $$TOOLBOX_CPP_RESOURCES_DIR"/resources.qrc" \

FORMS += \
    ui/grabber_parameters.ui \


