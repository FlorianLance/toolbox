
/*******************************************************************************
** exvr-scaner-base                                                           **
** No license (to be defined)                                                 **
** Copyright (c) [2018] [Florian Lance][EPFL-LNCO]                            **
********************************************************************************/

#include "grabber_parameters_widget.hpp"

// Qt
#include <QFile>
#include <QFileInfo>

// std
#include <functional>

// qt-utility
#include "qt_ui.hpp"

#include "qt_logger.hpp"

using namespace tool;
using namespace tool::network;
using namespace tool::ui;
using namespace tool::camera;
//using namespace tool::camera::K2;
using namespace tool::geo;


GrabberParametersW::GrabberParametersW(){

    setObjectName("GrabberParametersW ?");
    ui.setupUi(this);

    // init ui
    ui.sbMaxW->setMinimum(0);
    ui.sbMaxH->setMinimum(0);
    ui.sbMaxW->setMaximum(K2::depth_width);
    ui.sbMaxH->setMaximum(K2::depth_height);
    ui.sbMaxW->setValue(K2::depth_width);
    ui.sbMaxH->setValue(K2::depth_height-30);

    ui.sbMinW->setMinimum(0);
    ui.sbMinH->setMinimum(0);
    ui.sbMinW->setMaximum(K2::depth_width);
    ui.sbMinH->setMaximum(K2::depth_height);
    ui.sbMinW->setValue(0);
    ui.sbMinH->setValue(64);

    for (const auto& m : K2::all_requests_names()) {
        ui.cbDataType->addItem(from_view(m));
    }

    ui.fStateR->setStyleSheet("background-color:black;");
    ui.fStateW->setStyleSheet("background-color:black;");
    ui.twParameters->setCurrentIndex(0);

    // connections
    // # actions
    connect(ui.pbConnect,       &QPushButton::clicked, this, &GrabberParametersW::send_writing_connection_parameters);
    connect(ui.pbConnect,       &QPushButton::clicked, this, &GrabberParametersW::send_reading_connection_parameters);
    connect(ui.pbDisconnect,    &QPushButton::clicked, this, &GrabberParametersW::disable_connection_signal);    
    connect(ui.pbOpen,          &QPushButton::clicked, this, [&]{
        emit open_camera_signal(K2::index_to_mode(to_unsigned(ui.cbDataType->currentIndex())));
    });
    connect(ui.pbClose,         &QPushButton::clicked, this, &GrabberParametersW::close_camera_signal);
    connect(ui.cbDataType,      &QComboBox::currentTextChanged, this, [&](QString text){
        Q_UNUSED(text)
        emit open_camera_signal(K2::index_to_mode(to_unsigned(ui.cbDataType->currentIndex())));
    });

    // # network
    connect(ui.sbReadingPort,   QOverload<int>::of(&QSpinBox::valueChanged), this, [&](int value){ send_reading_connection_parameters(); Q_UNUSED(value)});
    connect(ui.sbWritingPort,   QOverload<int>::of(&QSpinBox::valueChanged), this, [&](int value){ send_writing_connection_parameters(); Q_UNUSED(value)});
    connect(ui.leWAddress,      &QLineEdit::textChanged,                     this, &GrabberParametersW::send_writing_connection_parameters);

    // # camera parameters -> this
    // ## misc
    connect(ui.rbGetData, &QRadioButton::toggled, this, &GrabberParametersW::set_data_state_signal);
    cameraParametersElements << ui.sbFPS;
    // ## size
    cameraParametersElements << ui.sbMinH << ui.sbMaxH << ui.sbMinW << ui.sbMaxW;
    // ## color
    cameraParametersElements << ui.sbRFiltered << ui.sbGFiltered << ui.sbBFiltered << ui.cbFilterDepthWithColor << ui.sbMaxDiffR << ui.sbMaxDiffG << ui.sbMaxDiffB;
    cameraParametersElements << ui.dsbYF << ui.dsbUF << ui.dsbVF;
    cameraParametersElements << ui.sbJpegCompressionRate;
    // ## infra
    cameraParametersElements << ui.sbMinInfra << ui.sbMaxInfra << ui.sbInfraMinRange << ui.sbInfraMaxRange << ui.cbInfraBinary << ui.cbInfraInvert;
    // ## depth
    cameraParametersElements << ui.cbEnableSmoothing << ui.cbErosion << ui.cbTemporal << ui.sbSmoothingKernelSize;
    cameraParametersElements << ui.sbSizeKernelErode << ui.sbMinErosionValue;
    cameraParametersElements << ui.sbMinNeighboursNb << ui.sbMinNeighboursLoops <<ui.dsbMinDepth << ui.dsbMaxDepth << ui.dsbLocalDiff << ui.cbSmoothingMethod << ui.cbErosionType;
    cameraParametersElements << ui.dsbOffsetAfterMin << ui.dsbVMin << ui.dsbVMax << ui.dsbGMin;
    // # display parameters -> this
    connect(ui.cbDisplayData,          &QCheckBox::toggled,                                this, &GrabberParametersW::set_display_state_signal);
    // ## model
    // ### 4x4 transform
    rotListW = {ui.dsbT00,  ui.dsbT01,  ui.dsbT02,
                ui.dsbT10,  ui.dsbT11,  ui.dsbT12,
                ui.dsbT20,  ui.dsbT21,  ui.dsbT22};

    trListW  = {ui.dsbT03, ui.dsbT13, ui.dsbT23};

    for( auto &w : rotListW){
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),   this, [&]{send_display_parameters();});
    }
    for( auto &w : trListW){
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),   this, [&]{send_display_parameters();});
    }
    // ### euclidian transform
    eulerRotListW   = { ui.dsbRx, ui.dsbRy, ui.dsbRz};
    eulerTrListW    = { ui.dsbTx, ui.dsbTy, ui.dsbTz};
    eulerScListW    = { ui.dsbSx, ui.dsbSy, ui.dsbSz};
    for(auto &w : eulerRotListW){
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),   this, [&]{send_display_parameters();});
    }
    for(auto &w : eulerTrListW){
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),   this, [&]{send_display_parameters();});
    }
    for(auto &w : eulerScListW){
        connect(w, QOverload<double>::of(&QDoubleSpinBox::valueChanged),   this, [&]{send_display_parameters();});
    }

    connect(ui.dsbSizePts,             QOverload<double>::of(&QDoubleSpinBox::valueChanged),    this, [&]{send_display_parameters();});
    connect(ui.cbForceCloudColor,      &QCheckBox::clicked,                                     this, [&]{send_display_parameters();});
    connect(ui.cbDisplayCloud,         &QCheckBox::clicked,                                     this, [&]{send_display_parameters();});
    connect(ui.sbCloudColorR,          QOverload<int>::of(&QSpinBox::valueChanged),             this, [&]{send_display_parameters();});
    connect(ui.sbCloudColorG,          QOverload<int>::of(&QSpinBox::valueChanged),             this, [&]{send_display_parameters();});
    connect(ui.sbCloudColorB,          QOverload<int>::of(&QSpinBox::valueChanged),             this, [&]{send_display_parameters();});

    for(auto w : cameraParametersElements){
        if(qobject_cast<QSpinBox*>(w)){
            connect(qobject_cast<QSpinBox*>(w), QOverload<int>::of(&QSpinBox::valueChanged),this, [&]{send_camera_parameters();});
        }else if(qobject_cast<QDoubleSpinBox*>(w)){
            connect(qobject_cast<QDoubleSpinBox*>(w), QOverload<double>::of(&QDoubleSpinBox::valueChanged),this, [&]{send_camera_parameters();});
        }else if(qobject_cast<QCheckBox*>(w)){
            connect(qobject_cast<QCheckBox*>(w),&QCheckBox::clicked, this, [&]{send_camera_parameters();});
        }else if(qobject_cast<QComboBox*>(w)){
            connect(qobject_cast<QComboBox*>(w), &QComboBox::currentTextChanged, this, [&]{send_camera_parameters();});
        }else if(qobject_cast<QRadioButton*>(w)){
            connect(qobject_cast<QRadioButton*>(w), &QRadioButton::toggled, this, [&]{send_camera_parameters();});
        }
    }
}

void GrabberParametersW::init_from_grabber(int readingPort){
    setObjectName("GrabberParametersW grabber");
    ui.fPacketsAndTime->hide();
    ui.laInterface->hide();
    ui.sbInterface->hide();
    w_blocking(ui.sbReadingPort)->setValue(readingPort);

    ui.fNetworkInfo->hide();
}

void GrabberParametersW::update_grabber_writing_info(QString writingAddress, int writingPort){
    w_blocking(ui.sbWritingPort)->setValue(writingPort);
    w_blocking(ui.leWAddress)->setText(writingAddress);
    send_writing_connection_parameters();
}

void GrabberParametersW::init_from_manager(QColor color, std_v1<Interface> *interfaces, const camera::K2::GrabberTargetInfo &info){

    setObjectName("GrabberParametersW manager");
    grabberUi = false;

    ui.laR->hide();
    ui.fStateR->hide();
    ui.rbDoNothing->hide();
    ui.cbDisplayData->hide();
    ui.rbGetData->hide();
    ui.sbReadingPort->hide();

    w_blocking(ui.leWAddress)->setText(info.name.c_str());
    w_blocking(ui.sbInterface)->setMinimum(0);
    w_blocking(ui.sbInterface)->setMaximum(static_cast<int>(interfaces->size()));
    w_blocking(ui.sbInterface)->setValue(info.targetPreferredInterface);
    w_blocking(ui.sbReadingPort)->setValue(info.targetReadingPort);
    w_blocking(ui.sbWritingPort)->setValue(info.targetWritingPort);
    w_blocking(ui.twParameters)->setCurrentIndex(0);
    w_blocking(ui.sbCloudColorR)->setValue(color.red());
    w_blocking(ui.sbCloudColorG)->setValue(color.green());
    w_blocking(ui.sbCloudColorB)->setValue(color.blue());

    ui.laIp->setText("Grabber PC name:");
}

void GrabberParametersW::init_force_all_cameras(){
    ui.twParameters->removeTab(0);
    ui.twParameters->removeTab(0);
    ui.twParameters->removeTab(0);
}

void GrabberParametersW::init_force_all_cameras_calibration(){

    ui.twParameters->removeTab(0);
    ui.twParameters->removeTab(0);
    ui.twParameters->removeTab(0);
    w_blocking(ui.cbForceCloudColor)->setChecked(true);
    w_blocking(ui.cbFilterDepthWithColor)->setChecked(true);
    w_blocking(ui.sbRFiltered)->setValue(70);
    w_blocking(ui.sbGFiltered)->setValue(220);
    w_blocking(ui.sbBFiltered)->setValue(60);
    w_blocking(ui.sbMaxDiffR)->setValue(50);
    w_blocking(ui.sbMaxDiffG)->setValue(65);
    w_blocking(ui.sbMaxDiffB)->setValue(40);
    w_blocking(ui.dsbMaxDepth)->setValue(2.5);
    w_blocking(ui.cbTemporal)->setChecked(false);
    w_blocking(ui.cbErosion)->setChecked(false);
    w_blocking(ui.sbMinNeighboursLoops)->setValue(5);
    w_blocking(ui.sbMinNeighboursNb)->setValue(4);
    w_blocking(ui.dsbLocalDiff)->setValue(5.);
    w_blocking(ui.dsbOffsetAfterMin)->setValue(-0.05);
}

void GrabberParametersW::update_reading_connection_state(bool state){
    ui.fStateR->setStyleSheet("background-color:"+QString(state?"green":"red")+";");
}

void GrabberParametersW::update_writing_connection_state(bool state){
    ui.fStateW->setStyleSheet("background-color:"+QString(state?"green":"red")+";");
}

void GrabberParametersW::open_camera(K2::FrameRequest frameMode){

    emit set_display_state_signal(false);
    emit set_data_state_signal(false);
    QCoreApplication::processEvents(QEventLoop::AllEvents, 30);

    w_blocking(ui.cbDataType)->setCurrentIndex(to_signed(mode_to_index(frameMode)));
    w_blocking(ui.rbDoNothing)->setChecked(true);
    w_blocking(ui.cbDisplayData)->setChecked(false);

    emit open_camera_signal(frameMode);
    QCoreApplication::processEvents(QEventLoop::AllEvents, 30);
}

void GrabberParametersW::close_camera(){

    emit set_display_state_signal(false);
    emit set_data_state_signal(false);
    QCoreApplication::processEvents(QEventLoop::AllEvents, 30);

    w_blocking(ui.rbDoNothing)->setChecked(true);
    w_blocking(ui.cbDisplayData)->setChecked(false);

    emit close_camera_signal();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 30);
}

void GrabberParametersW::set_camera_parameters(K2::Parameters p){

    // misc
    w_blocking(ui.sbFPS)->setValue(static_cast<int>(p.fps));
    // size
    w_blocking(ui.sbMinW)->setValue(static_cast<int>(p.minWidth));
    w_blocking(ui.sbMaxW)->setValue(static_cast<int>(p.maxWidth));
    w_blocking(ui.sbMinH)->setValue(static_cast<int>(p.minHeight));
    w_blocking(ui.sbMaxH)->setValue(static_cast<int>(p.maxHeight));
    // color
    w_blocking(ui.sbRFiltered)->setValue(p.filterColor.x());
    w_blocking(ui.sbGFiltered)->setValue(p.filterColor.y());
    w_blocking(ui.sbBFiltered)->setValue(p.filterColor.z());
    w_blocking(ui.cbFilterDepthWithColor)->setChecked(p.filterDepthWithColor);
    w_blocking(ui.sbMaxDiffR)->setValue(p.maxDiffColor.x());
    w_blocking(ui.sbMaxDiffG)->setValue(p.maxDiffColor.y());
    w_blocking(ui.sbMaxDiffB)->setValue(p.maxDiffColor.z());
    w_blocking(ui.dsbYF)->setValue(static_cast<qreal>(p.yFactor));
    w_blocking(ui.dsbUF)->setValue(static_cast<qreal>(p.uFactor));
    w_blocking(ui.dsbVF)->setValue(static_cast<qreal>(p.vFactor));
    w_blocking(ui.sbJpegCompressionRate)->setValue(static_cast<int>(p.jpegCompressionRate));
    // infrared
    w_blocking(ui.sbMinInfra)->setValue(static_cast<int>(p.minInfra));
    w_blocking(ui.sbMaxInfra)->setValue(static_cast<int>(p.maxInfra));
    w_blocking(ui.sbInfraMinRange)->setValue(static_cast<int>(p.minInfraRange));
    w_blocking(ui.sbInfraMaxRange)->setValue(static_cast<int>(p.maxInfraRange));
    w_blocking(ui.cbInfraBinary)->setChecked(p.infraBinary);
    w_blocking(ui.cbInfraInvert)->setChecked(p.infraInvert);
    // depth
    w_blocking(ui.cbEnableSmoothing)->setChecked(p.smoothingEnabled);
    w_blocking(ui.cbSmoothingMethod)->setCurrentIndex(p.smoothingMethod);
    w_blocking(ui.sbSmoothingKernelSize)->setValue(p.smoothingKernelSize);
    w_blocking(ui.sbSizeKernelErode)->setValue(p.erosionSize);
    w_blocking(ui.cbErosion)->setChecked(p.doErosion);
    w_blocking(ui.sbMinErosionValue)->setValue(p.minErosionValue);
    w_blocking(ui.cbErosionType)->setCurrentIndex(p.erosionType);
    w_blocking(ui.dsbMinDepth)->setValue(static_cast<double>(p.minDepthValue));
    w_blocking(ui.dsbMaxDepth)->setValue(static_cast<double>(p.maxDepthValue));
    w_blocking(ui.dsbLocalDiff)->setValue(static_cast<double>(p.maxLocalDiff));
    w_blocking(ui.dsbOffsetAfterMin)->setValue(static_cast<double>(p.offsetAfterMin));
    w_blocking(ui.sbMinNeighboursNb)->setValue(p.nbMinNeighboursNb);
    w_blocking(ui.sbMinNeighboursLoops)->setValue(p.minNeighboursLoops);
    w_blocking(ui.cbTemporal)->setChecked(p.doTemporalFilter);
    w_blocking(ui.dsbVMin)->setValue(static_cast<double>(p.vmin));
    w_blocking(ui.dsbVMax)->setValue(static_cast<double>(p.vmax));
    w_blocking(ui.dsbGMin)->setValue(static_cast<double>(p.gmin));
}

void GrabberParametersW::set_camera_state_ui(bool state){
    ui.fStateCam->setStyleSheet("background-color:"+QString(state?"green":"red")+";");
}

void GrabberParametersW::update_transformation(const Mat4<double> &m){

    blockSignals(true);
    for(size_t ii = 0; ii < 9; ++ii){
        w_blocking(rotListW[ii])->setValue(m.r(to_signed(ii)));
    }

    w_blocking(ui.dsbT03)->setValue(m.t(to_signed(0)));
    w_blocking(ui.dsbT13)->setValue(m.t(to_signed(1)));
    w_blocking(ui.dsbT23)->setValue(m.t(to_signed(2)));

    blockSignals(false);
}

Mat4<double> GrabberParametersW::get_transformation() const{

    Mat4<double> tr = Mat4d::identity();
    for(size_t ii = 0; ii < 9; ++ii){
        tr.r(to_signed(ii)) = rotListW[ii]->value();
    }

    tr.t(to_signed(0)) = ui.dsbT03->value();
    tr.t(to_signed(1)) = ui.dsbT13->value();
    tr.t(to_signed(2)) = ui.dsbT23->value();

    return tr;
}

K2::Parameters GrabberParametersW::read_camera_parameters() const{
    K2::Parameters p;
    // misc
    p.fps                   = static_cast<unsigned char>(ui.sbFPS->value());
    // size
    p.minWidth              = static_cast<unsigned int>( ui.sbMinW->value());
    p.maxWidth              = static_cast<unsigned int>( ui.sbMaxW->value());
    p.minHeight             = static_cast<unsigned int>( ui.sbMinH->value());
    p.maxHeight             = static_cast<unsigned int>( ui.sbMaxH->value());
    // color
    p.filterColor           = Pt3<std::uint8_t>(
        static_cast<std::uint8_t>(ui.sbRFiltered->value()),
        static_cast<std::uint8_t>(ui.sbGFiltered->value()),
        static_cast<std::uint8_t>(ui.sbBFiltered->value()));
    p.filterDepthWithColor  = ui.cbFilterDepthWithColor->isChecked();
    p.maxDiffColor           = Pt3<std::uint8_t>(
        static_cast<std::uint8_t>(ui.sbMaxDiffR->value()),
        static_cast<std::uint8_t>(ui.sbMaxDiffG->value()),
        static_cast<std::uint8_t>(ui.sbMaxDiffB->value()));
    p.yFactor               = static_cast<float>(ui.dsbYF->value());
    p.uFactor               = static_cast<float>(ui.dsbUF->value());
    p.vFactor               = static_cast<float>(ui.dsbVF->value());
    p.jpegCompressionRate   = static_cast<unsigned char>(ui.sbJpegCompressionRate->value());
    // infra
    p.minInfra              = static_cast<unsigned short>(ui.sbMinInfra->value());
    p.maxInfra              = static_cast<unsigned short>(ui.sbMaxInfra->value());
    p.minInfraRange         = static_cast<unsigned short>(ui.sbInfraMinRange->value());
    p.maxInfraRange         = static_cast<unsigned short>(ui.sbInfraMaxRange->value());
    p.infraBinary           = ui.cbInfraBinary->isChecked();
    p.infraInvert           = ui.cbInfraInvert->isChecked();
    // depth
    p.smoothingEnabled      = ui.cbEnableSmoothing->isChecked();
    p.smoothingMethod       = static_cast<unsigned char>(ui.cbSmoothingMethod->currentIndex());
    p.smoothingKernelSize   = static_cast<unsigned char>(ui.sbSmoothingKernelSize->value());
    p.erosionSize           = static_cast<unsigned char>(ui.sbSizeKernelErode->value());
    p.doErosion             = ui.cbErosion->isChecked();
    p.minErosionValue       = static_cast<unsigned char>(ui.sbMinErosionValue->value());
    p.erosionType           = static_cast<K2::MorphShapes>( ui.cbErosionType->currentIndex());
    p.minDepthValue         = static_cast<float>(ui.dsbMinDepth->value());
    p.maxDepthValue         = static_cast<float>(ui.dsbMaxDepth->value());
    p.maxLocalDiff          = static_cast<float>(ui.dsbLocalDiff->value());
    p.offsetAfterMin        = static_cast<float>(ui.dsbOffsetAfterMin->value());
    p.nbMinNeighboursNb     = static_cast<unsigned char>(ui.sbMinNeighboursNb->value());
    p.minNeighboursLoops    = static_cast<unsigned char>(ui.sbMinNeighboursLoops->value());
    p.doTemporalFilter      = ui.cbTemporal->isChecked();
    p.vmin                  = static_cast<float>(ui.dsbVMin->value());
    p.vmax                  = static_cast<float>(ui.dsbVMax->value());
    p.gmin                  = static_cast<float>(ui.dsbGMin->value());
    return p;
}

DisplayParameters GrabberParametersW::read_display_parameters() const{

    DisplayParameters p;

//    auto offsetM = Mat4d::transform(   Pt3d{ui.dsbSx->value(),ui.dsbSy->value(),ui.dsbSz->value()},
//                                            Pt3d{ui.dsbRx->value(),ui.dsbRy->value(),ui.dsbRz->value()},
//                                            Pt3d{ui.dsbTx->value(),ui.dsbTy->value(),ui.dsbTz->value()});
    p.model = get_transformation();//  p.model;//*offsetM;

    p.sizePtsCloud      = static_cast<float>(ui.dsbSizePts->value());
    p.displayCloud      = ui.cbDisplayCloud->isChecked();
    p.forceCloudColor   = ui.cbForceCloudColor->isChecked();
    p.colorCloud        = QColor(ui.sbCloudColorR->value(),ui.sbCloudColorG->value(),ui.sbCloudColorB->value());

    return p;
}

void GrabberParametersW::copy_camera_parameters(){
    emit copy_camera_parameters_signal(read_camera_parameters());
}

void GrabberParametersW::send_display_parameters(){
    emit send_display_parameters_signal(read_display_parameters());
}

void GrabberParametersW::add_network_log_message(QString message){
    ui.teNetworkInfo->append(message);
    ui.teNetworkInfo->verticalScrollBar()->setValue(ui.teNetworkInfo->verticalScrollBar()->maximum());
}

void GrabberParametersW::display_cloud(){
    w_blocking(ui.cbDisplayCloud)->setChecked(true);
}

void GrabberParametersW::hide_cloud(){
    w_blocking(ui.cbDisplayCloud)->setChecked(false);
}

void GrabberParametersW::set_size_cloud(qreal size){
    w_blocking(ui.dsbSizePts)->setValue(size);
}

void GrabberParametersW::force_color_cloud(bool state){
    w_blocking(ui.cbForceCloudColor)->setChecked(state);
}

void GrabberParametersW::update_frames_info(int pReceived, int pLost, int fReceived, int fLost){
    ui.laPR->setText(QString::number(pReceived));
    ui.laPL->setText(QString::number(pLost));
    ui.laPTot->setText(QString::number(pReceived+pLost));
    ui.laFR->setText(QString::number(fReceived));
    ui.laFL->setText(QString::number(fLost));
    ui.laFT->setText(QString::number(fReceived+fLost));
}

void GrabberParametersW::update_last_frame_time(int64_t microS){
    ui.laTTotal->setText(QString::number(microS*0.001) +"ms");
}

void GrabberParametersW::set_parameters_tab_index(int index){
    ui.twParameters->setCurrentIndex(index);
}

void GrabberParametersW::send_camera_parameters(){
    emit send_camera_parameters_signal(read_camera_parameters());
}

void GrabberParametersW::send_reading_connection_parameters(){

    ui.fStateR->setStyleSheet("background-color:black;");
    if(grabberUi){
        emit send_grabber_reading_connection_parameters_signal(
            ui.sbReadingPort->value()   // grabber reading port
        );
    }else{
        emit send_manager_reading_connection_parameters_signal(
            ui.sbInterface->value(),    // manager reading interface
            ui.sbReadingPort->value()   // manager reading port
        );
    }
}

void GrabberParametersW::send_writing_connection_parameters(){

    ui.fStateW->setStyleSheet("background-color:black;");
    if(grabberUi){
        send_grabber_writing_connection_parameters_signal(
            ui.leWAddress->text(),      // grabber writing address
            ui.sbWritingPort->value()   // grabber writing port
        );
    }else{
        emit send_manager_writing_connection_parameters_signal(
            ui.leWAddress->text(),      // target PC name
            ui.sbWritingPort->value(),  // port to write
            ui.sbInterface->value(),    // grabber interface to be send as writing address
            ui.sbReadingPort->value()   // grabber writing port to be send
        );
    }
}

#include "moc_grabber_parameters_widget.cpp"
