
/*******************************************************************************
** Toolbox-qt-utility                                                         **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/

#pragma once

// Qt
#include <QWidget>
#include <QMatrix4x4>
#include <QScrollBar>

// base
#include "geometry/matrix4.hpp"
#include "network/network_utility.hpp"
#include "camera/kinect2_settings_files.hpp"

// ui
#include "ui_grabber_parameters.h"

namespace Ui {
    class GrabberParametersUI;
}

namespace tool::ui{


struct DisplayOptions{

    // cloud
    bool displayCloud = true;
    bool forceCloudColor = false;
    float sizePtsCloud = 3.f;
    QColor colorCloud = Qt::red;
    geo::Mat4<double> model;
};

class GrabberParametersW : public QWidget{
Q_OBJECT
public:

    GrabberParametersW();

    void init_from_grabber(int readingPort);
    void update_grabber_writing_info(QString writingAddress, int writingPort);

    void init_from_manager(QColor color, std_v1<network::Interface> *interfaces, const camera::K2::GrabberTargetInfo &info);

    void init_force_all_cameras();
    void init_force_all_cameras_calibration();

public slots:

    // network
    void update_reading_connection_state(bool state);
    void update_writing_connection_state(bool state);
    void send_writing_connection_parameters();
    void send_reading_connection_parameters();

    // settings
    void update_ui_settings(camera::K2::Settings settings);
    camera::K2::Settings read_settings_from_ui() const;
    void copy_current_ui_settings();
    void send_current_ui_settings();

    // display options
    void send_current_ui_display_options();
    DisplayOptions read_display_options_from_ui() const;

    // camera
    void open_camera(camera::K2::FrameRequest frameMode);
    void close_camera();
    void set_camera_state_ui(bool state);

    // 3D
    void update_transformation(const geo::Mat4<double> &m);
    geo::Mat4<double> get_transformation()const;
    void add_network_log_message(QString message);
    void display_cloud();
    void hide_cloud();
    void set_size_cloud(qreal size);
    void force_color_cloud(bool state);
    void update_frames_info(int pReceived, int pLost, int fReceived, int fLost);
    void update_last_frame_time(std::int64_t microS);
    void set_parameters_tab_index(int index);

signals:

    // network
    void send_grabber_reading_connection_parameters_signal(int readingPort);
    void send_grabber_writing_connection_parameters_signal(QString writingAddress, int writingPort);
    void send_manager_reading_connection_parameters_signal(int readingInterface, int readingPort);
    void send_manager_writing_connection_parameters_signal(QString grabberName, int writingPort, int readingInterface, int readingPort);
    void disable_connection_signal();

    // settings
    void send_settings_parameters_signal(camera::K2::Settings);
    void copy_camera_parameters_signal(camera::K2::Settings);

    // display options
    void send_display_options_signal(DisplayOptions displayOptions);

    // camera
    void open_camera_signal(camera::K2::FrameRequest frameMode);
    void close_camera_signal();

    // state
    void set_display_state_signal(bool state);
    void set_data_state_signal(bool state);

private :


    bool grabberUi = true;
    Ui::GrabberParametersUI ui;

    QVector<QWidget*> cameraParametersElements;
    QVector<QWidget*> displayParametersElements;

    std_v1<QDoubleSpinBox*> rotListW;
    std_v1<QDoubleSpinBox*> trListW;

    std_v1<QDoubleSpinBox*> eulerTrListW;
    std_v1<QDoubleSpinBox*> eulerRotListW;
    std_v1<QDoubleSpinBox*> eulerScListW;
};
}
