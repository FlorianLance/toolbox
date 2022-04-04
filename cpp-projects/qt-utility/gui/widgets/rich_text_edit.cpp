

///****************************************************************************
//**
//** Copyright (C) 2016 The Qt Company Ltd.
//** Contact: https://www.qt.io/licensing/
//**
//** This file is part of the demonstration applications of the Qt Toolkit.
//**
//** $QT_BEGIN_LICENSE:BSD$
//** Commercial License Usage
//** Licensees holding valid commercial Qt licenses may use this file in
//** accordance with the commercial license agreement provided with the
//** Software or, alternatively, in accordance with the terms contained in
//** a written agreement between you and The Qt Company. For licensing terms
//** and conditions see https://www.qt.io/terms-conditions. For further
//** information use the contact form at https://www.qt.io/contact-us.
//**
//** BSD License Usage
//** Alternatively, you may use this file under the terms of the BSD license
//** as follows:
//**
//** "Redistribution and use in source and binary forms, with or without
//** modification, are permitted provided that the following conditions are
//** met:
//**   * Redistributions of source code must retain the above copyright
//**     notice, this list of conditions and the following disclaimer.
//**   * Redistributions in binary form must reproduce the above copyright
//**     notice, this list of conditions and the following disclaimer in
//**     the documentation and/or other materials provided with the
//**     distribution.
//**   * Neither the name of The Qt Company Ltd nor the names of its
//**     contributors may be used to endorse or promote products derived
//**     from this software without specific prior written permission.
//**
//**
//** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
//**
//** $QT_END_LICENSE$
//**
//****************************************************************************/


///*******************************************************************************
//** PhotosConsigne                                                             **
//** MIT License                                                                **
//** Copyright (c) [2016] [Florian Lance]                                       **
//**                                                                            **
//** Permission is hereby granted, free of charge, to any person obtaining a    **
//** copy of this software and associated documentation files (the "Software"), **
//** to deal in the Software without restriction, including without limitation  **
//** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
//** and/or sell copies of the Software, and to permit persons to whom the      **
//** Software is furnished to do so, subject to the following conditions:       **
//**                                                                            **
//** The above copyright notice and this permission notice shall be included in **
//** all copies or substantial portions of the Software.                        **
//**                                                                            **
//** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
//** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
//** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
//** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
//** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
//** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
//** DEALINGS IN THE SOFTWARE.                                                  **
//**                                                                            **
//********************************************************************************/


///**
// * \file RichTextEditW.hpp
// * \brief defines RichTextEditW/TextEdit
// * \author Florian Lance
// * \date 17/05/2017
// */


//// Qt
//#include <QPushButton>
//#include <QAction>
//#include <QApplication>
//#include <QClipboard>
//#include <QColorDialog>
//#include <QFontComboBox>
//#include <QFile>
//#include <QFileDialog>
//#include <QFileInfo>
//#include <QFontDatabase>
//#include <QMenu>
//#include <QMenuBar>
//#include <QTextCodec>
//#include <QTextEdit>
//#include <QStatusBar>
//#include <QToolBar>
//#include <QTextCursor>
//#include <QTextDocumentWriter>
//#include <QTextList>
//#include <QtDebug>
//#include <QCloseEvent>
//#include <QMessageBox>
//#include <QMimeData>

//#ifndef QT_NO_PRINTER
//#include <QPrintDialog>
//#include <QPrinter>
//#include <QPrintPreviewDialog>
//#endif

//// local
//#include "rich_text_edit.hpp"

//// generated
////#include "ui_InsertLink.h"


//namespace Ui {
//    class InsertLinkW;
//}

//using namespace pc;



//pc::Photo::Photo(QImage image){

//    scaledPhoto  = image;
//    originalSize = scaledPhoto.size();
//}

//pc::Photo::Photo(const QString &path, bool isWhiteSpace) : isWhiteSpace(isWhiteSpace), pathPhoto(path){

//    constexpr int maxHeight = 800;
//    constexpr int maxWidth  = 800;

//    if(isWhiteSpace){
//        namePhoto = "Espace transparent";
//    }else{

//        info            = QFileInfo(path);
//        scaledPhoto     = QImage(path);
//        originalSize    = scaledPhoto.size();

//        if(!scaledPhoto.isNull()){

//            namePhoto = pathPhoto.split('/').last().split('.').first();
//            if(scaledPhoto.size().width() > maxWidth)
//                scaledPhoto = scaledPhoto.scaledToWidth(maxWidth);
//            if(scaledPhoto.size().height() > maxHeight)
//                scaledPhoto = scaledPhoto.scaledToHeight(maxHeight);
//        }
//        else{
//            namePhoto = "Erreur";
//            qWarning() << "-Error: photo not loaded: " << pathPhoto;
//        }
//    }
//}

//void pc::Photo::draw(QPainter &painter, const ImagePositionSettings &position, const QRectF &rectPhoto, const ExtraPCInfo &infos, const QSizeF &pageSize){

//    if(isWhiteSpace){
//        return;
//    }

//    if(scaledPhoto.isNull()){
//        qWarning() << "-Error: photo is null, can't be drawn";
//        return;
//    }

//    if(infos.preview){
//        draw_small(painter, position, rectPhoto, scaledPhoto, infos, pageSize);
//    }else{
//        if(rectPhoto.width() > 32000 || rectPhoto.height() > 32000){
////            draw_huge(painter, rectPhoto);
//            qWarning() << "-Error: Format too huge: " << namePhoto << " can't be drawn.";
//        }else{
//            draw_small(painter, position, rectPhoto, (pathPhoto.size() > 0) ? QImage(pathPhoto).transformed((QTransform().rotate(rotation))) : scaledPhoto, infos, pageSize);
//        }
//    }
//}

//QRectF pc::Photo::draw_small(QPainter &painter, const ImagePositionSettings &position, const QRectF &rectPhoto, const QImage &photo, const ExtraPCInfo &infos, const QSizeF &pageSize){

//    int startX, startY;
//    qreal newX =0., newY =0., newWidth =0., newHeight =0.;
//    QImage photoToDraw;

//    int rectPhotoWidth  = static_cast<int>(rectPhoto.width());
//    int rectPhotoHeight = static_cast<int>(rectPhoto.height());

//    // scale and rotate photo
//    qreal scaleCenter;
//    switch(position.adjustment){

//        case PhotoAdjust::adjust:

//            photoToDraw = photo.scaled(rectPhotoWidth,rectPhotoHeight, Qt::KeepAspectRatio);

//            {
//                qreal diffX = rectPhoto.width() - photoToDraw.width();
//                qreal diffY = rectPhoto.height() - photoToDraw.height();
//                newX      = static_cast<int>(diffX * position.xPos) + rectPhoto.x();
//                newY      = static_cast<int>(diffY * (1. - position.yPos)) + rectPhoto.y();
//                newWidth  = photoToDraw.width();
//                newHeight = photoToDraw.height();
//            }

//        break;
//        case PhotoAdjust::extend:

//            photoToDraw = photo.scaled(rectPhotoWidth, rectPhotoHeight, Qt::IgnoreAspectRatio);
//            newX      = rectPhoto.x();
//            newY      = rectPhoto.y();
//            newWidth  = photoToDraw.width();
//            newHeight = photoToDraw.height();

//        break;
//        case PhotoAdjust::center:

////            if(photo.width() > rectPhoto.width() || photo.height() > rectPhoto.height()){
////                scaleCenter = position.scale;//(position.scale < 1.) ? position.scale : 1.;
////                qDebug() << "rescale";
////                photoToDraw = photo.scaled(static_cast<int>(rectPhotoWidth*position.scale), static_cast<int>(rectPhotoHeight*position.scale), Qt::KeepAspectRatio);
////            }else{
////                photoToDraw = photo;
////            }

//            photoToDraw = photo.scaled(static_cast<int>(rectPhotoWidth*position.scale), static_cast<int>(rectPhotoHeight*position.scale), Qt::KeepAspectRatio);


//            {
//                qreal diffX = rectPhoto.width() - photoToDraw.width();
//                qreal diffY = rectPhoto.height() - photoToDraw.height();
//                newX      = static_cast<int>(diffX * position.xPos) + rectPhoto.x();
//                newY      = static_cast<int>(diffY * (1. - position.yPos)) + rectPhoto.y();
//                newWidth  = photoToDraw.width();
//                newHeight = photoToDraw.height();
//            }

//        break;
//        case PhotoAdjust::fill:

//            scaleCenter = (position.scale > 1.) ? position.scale : 1.;
//            photoToDraw = photo.scaled(static_cast<int>(rectPhotoWidth*scaleCenter), static_cast<int>(rectPhotoHeight*scaleCenter), Qt::KeepAspectRatioByExpanding);

//            startX = static_cast<int>((photoToDraw.width() - rectPhoto.width())*position.xPos);
//            startY = static_cast<int>((photoToDraw.height() - rectPhoto.height())*(1. - position.yPos));

//            photoToDraw = photoToDraw.copy(QRect(startX, startY, static_cast<int>(rectPhoto.width()), static_cast<int>(rectPhoto.height())));
//            newX      = rectPhoto.x();
//            newY      = rectPhoto.y();
//            newWidth  = photoToDraw.width();
//            newHeight = photoToDraw.height();

//        break;
//        case PhotoAdjust::mosaic:

//            QSize scaledSize(infos.factorUpscale* position.scale*photo.width(), infos.factorUpscale*position.scale*photo.height());

//            if(!infos.preview){
//                scaledSize.setWidth(scaledSize.width() *(1.*scaledPhoto.width()/photo.width()));
//                scaledSize.setHeight(scaledSize.height() *(1.*scaledPhoto.height()/photo.height()));
//            }

//            QImage tile = photo.scaled(scaledSize, Qt::IgnoreAspectRatio);
//            if(tile.width() > rectPhoto.width() || tile.height() > rectPhoto.height()){
//                tile = tile.scaled(rectPhotoWidth, rectPhotoHeight, Qt::KeepAspectRatio);
//            }

//            int nbTilesH = rectPhotoWidth  / tile.width()  + ((rectPhotoWidth % tile.width() > 0) ? 1 : 0);
//            int nbTilesV = rectPhotoHeight / tile.height() + ((rectPhotoHeight % tile.height() > 0) ? 1 : 0);

//            QImage tiles(tile.width()*nbTilesH, tile.height()*nbTilesV, QImage::Format_ARGB32);
//            QPainter tilePainter(&tiles);
//            for(int ii = 0; ii < nbTilesH; ++ii){
//                for(int jj = 0; jj < nbTilesV; ++jj){
//                    tilePainter.drawImage(QRectF(ii*tile.width(), jj* tile.height(), tile.width(), tile.height()), tile, QRectF(0,0,tile.width(), tile.height()));
//                }
//            }
//            tilePainter.end();


////            startX = 0; // left
////            if(alignment & Qt::AlignHCenter){ // right
////                startX = static_cast<int>((tiles.width() - rectPhoto.width())*0.5);
////            }else if(alignment & Qt::AlignRight){ // h center
////                startX = static_cast<int>(tiles.width() - rectPhoto.width());
////            }

////            startY = 0; // top
////            if(alignment & Qt::AlignVCenter){ // V center
////                startY = static_cast<int>((tiles.height() - rectPhoto.height())*0.5);
////            }else if(alignment & Qt::AlignBottom){ // bottom
////                startY = static_cast<int>(tiles.height() - rectPhoto.height());
////            }

//            startX = static_cast<int>((tiles.width() - rectPhoto.width())*position.xPos);
//            startY = static_cast<int>((tiles.height() - rectPhoto.height())*(1. - position.yPos));


//            photoToDraw = tiles.copy(QRect(startX,startY,rectPhotoWidth,rectPhotoHeight));

//            newX      = rectPhoto.x();
//            newY      = rectPhoto.y();
//            newWidth  = photoToDraw.width();
//            newHeight = photoToDraw.height();

//        break;
//    }


//    // draw image
//    QRectF newRectPhoto(newX, newY, newWidth, newHeight);

//    // ########### TEST
////    qreal square = std::min(newRectPhoto.right()-newRectPhoto.left(), newRectPhoto.bottom()-newRectPhoto.top());
////    QRect ree(newRectPhoto.center().x()-square*0.5, newRectPhoto.center().y()-square*0.5, square,square);
////    painter.setClipping(true);
////    QRegion r(ree, QRegion::Ellipse);
////    painter.setClipRegion(r);

//    // ###########
//    painter.drawImage(newRectPhoto, photoToDraw, QRectF(0.,0.,photoToDraw.width(),photoToDraw.height()));
//    // ###########

////    painter.setClipping(false);
//    // ########### END TEST

//    // draw size
//    if(infos.displaySizes && newWidth > 100 && newHeight > 50){

//        QPen pen;
//        pen.setColor(Qt::red);
//        painter.setPen(pen);
//        QFont font;

//        QString sizeImageStr = QString::number(newRectPhoto.width()/pageSize.width()*infos.paperFormat.sizeMM.width(), 'i',0) + "x" +
//                QString::number(newRectPhoto.height()/pageSize.height()*infos.paperFormat.sizeMM.height(), 'i', 0) +"(mm)";
//        font.setPointSizeF(newRectPhoto.width()/sizeImageStr.size());
//        painter.setFont(font);
//        painter.drawText(QRectF(newX, newY, newWidth, newHeight),  Qt::AlignCenter,sizeImageStr);
//    }

//    return newRectPhoto;
//}


//void pc::Photo::draw_huge_photo_whith_tiles(QPainter &painter, const QImage &photoToUpscale, const QRectF &rectPhoto){

//    QSizeF upscaledSize(rectPhoto.width(),rectPhoto.height());

//    qreal ratioW = 1.*photoToUpscale.width()  / upscaledSize.width();
//    qreal ratioH = 1.*photoToUpscale.height() / upscaledSize.height();
//    if(ratioW > ratioH){
//        upscaledSize = QSizeF(upscaledSize.width(), photoToUpscale.height()/ratioW);
//    }else{
//        upscaledSize = QSizeF(photoToUpscale.width()/ratioH, upscaledSize.height());
//    }

//    // draw tiles
//    int widthUpPerTile  = (upscaledSize.width() /10);
//    int heightUpPerTile = (upscaledSize.height()/10);

//    int widthUpTileRemainder  = static_cast<int>(upscaledSize.width())%widthUpPerTile;
//    int heightUpTileRemainder = static_cast<int>(upscaledSize.height())%heightUpPerTile;

//    int nbTilesH            = upscaledSize.width()/widthUpPerTile;
//    int widthPerTile        = photoToUpscale.width()/nbTilesH;
//    int widthTileRemainder  = static_cast<int>(photoToUpscale.width())%widthPerTile;

//    int nbTilesV            = upscaledSize.height()/heightUpPerTile;
//    int heightPerTile       = photoToUpscale.height()/nbTilesV;
//    int heightTileRemainder = static_cast<int>(photoToUpscale.height())%heightPerTile;

//    int nbTotalTilesH = nbTilesH + (widthTileRemainder  > 0 ? 1:0);
//    int nbTotalTilesV = nbTilesV + (heightTileRemainder > 0 ? 1:0);

//    for(int ii = 0; ii < nbTotalTilesH; ++ii){

//        bool lastTileH   = (widthTileRemainder > 0) && (ii == nbTotalTilesH-1);
//        bool lastTileUpH = (widthUpTileRemainder > 0) && (ii == nbTotalTilesH-1);

//        for (int jj = 0; jj < nbTotalTilesV; ++jj){

//            bool lastTileV = (heightTileRemainder > 0) && (jj == nbTotalTilesV-1);
//            bool lastTileUpV = (heightUpTileRemainder > 0) && (jj == nbTotalTilesV-1);
//            QRectF itRect(ii* widthPerTile, jj * heightPerTile, lastTileH ? widthTileRemainder : widthPerTile, lastTileV ?  heightTileRemainder : heightPerTile);
//            QRectF itUpRect(ii* widthUpPerTile, jj * heightUpPerTile, lastTileUpH ? widthUpTileRemainder : widthUpPerTile, lastTileUpV ? heightUpTileRemainder : heightUpPerTile);
//            size_t offset = itRect.x() * photoToUpscale.depth() / 8 + itRect.y() * photoToUpscale.bytesPerLine();

//            painter.drawImage(rectPhoto.x()+itUpRect.x(),rectPhoto.y()+itUpRect.y(), QImage(photoToUpscale.bits() + offset, itRect.width(),itRect.height(),
//                                                                                            photoToUpscale.bytesPerLine(), photoToUpscale.format()).scaled(itUpRect.width(),itUpRect.height()));
//        }
//    }
//}

//void pc::Photo::draw_huge(QPainter &painter, const QRectF &rectPhoto){

//    Q_UNUSED(painter)
//    Q_UNUSED(rectPhoto)
////    // TODO
////    QImage rotatedPhoto =  (pathPhoto.size() > 0 ? QImage(pathPhoto) : scaledPhoto).transformed(QTransform().rotate(rotation));
//////    qreal newX = rectPhoto.x(), newY = rectPhoto.y();
//////    draw_huge_photo_whith_tiles(painter, rotatedPhoto, QRectF(newX,newY,rectPhoto.width(),rectPhoto.height()));

////    qreal ratio;
////    qreal ratioW = (rectPhoto.width()  / 5000);
////    qreal ratioH = (rectPhoto.height() / 5000);

////    qreal smallWidth, smallHeight;
////    if(ratioW < ratioH){
////        ratio = ratioW;
////    }else{
////        ratio = ratioH;
////    }

////    smallWidth  = (rectPhoto.width() /ratio);
////    smallHeight = (rectPhoto.height()/ratio);

////    QImage smallPhoto(smallWidth, smallHeight, QImage::Format_ARGB32);
////    smallPhoto.save("G:/t.png");
////    QPainter smallPainter(&smallPhoto);
////    QRectF newRectPhoto = draw_small(smallPainter, QRectF(0.,0.,smallWidth,smallHeight), rotatedPhoto, false, false, QSizeF(), QSizeF());

////    draw_huge_photo_whith_tiles(painter, smallPhoto, QRectF(newRectPhoto.x()*ratio, newRectPhoto.y()*ratio, rectPhoto.width()*ratio, rectPhoto.height()*ratio));
//}



//SettingsW::~SettingsW(){}

//void SettingsW::update_UI(){}

//void SettingsW::write_to_xml(QXmlStreamWriter &xml) const{Q_UNUSED(xml)}

//void SettingsW::load_from_xml(QXmlStreamReader &xml){Q_UNUSED(xml)}


//void SettingsW::init_checkboxes_connections(QVector<QCheckBox *> checkBoxes, bool displayZones){

//    for(auto &&cb : checkBoxes){
//        connect(cb, &QCheckBox::clicked, this, [=]{
//            emit settings_updated_signal(displayZones);
//        });
//    }
//}

//void SettingsW::init_comboboxes_connections(QVector<QComboBox *> comboBox, bool displayZones){

//    for(auto &&cb : comboBox){
//        connect(cb, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=]{
//            emit settings_updated_signal(displayZones);
//        });
//    }
//}

//void pc::SettingsW::init_color_dialog_connections(QToolButton *tb, QString actionToolTip, QString dialogColorText, QColor *color, QSize iconSize, bool alpha){

//    QPixmap pixColor(iconSize.width(), iconSize.height());
//    pixColor.fill(*color);
//    QAction *actionColor = new QAction(pixColor, actionToolTip, this);
//    connect(actionColor, &QAction::triggered, this, [=]{

//        QColor newColor;
//        if(alpha){
//            newColor = QColorDialog::getColor(*color, nullptr, dialogColorText, QColorDialog::ColorDialogOption::ShowAlphaChannel);
//        }else{
//            newColor = QColorDialog::getColor(*color, nullptr, dialogColorText);
//        }

//        if (!newColor.isValid())
//            return;

//        *color = newColor;

//        QPixmap pix(iconSize.width(), iconSize.height());
//        pix.fill(*color);
//        actionColor->setIcon(pix);
//        emit settings_updated_signal(false);
//    });
//    tb->setDefaultAction(actionColor);
//}

//void SettingsW::init_image_dialog_connections(QToolButton *tb, QString actionToolTip, QString dialogText, SPhoto &photo, QSize iconeSIze){

//    QAction *actionImage = new QAction(tb->icon(), actionToolTip, this);
//    connect(actionImage, &QAction::triggered, this, [=,&photo]{

//        QString filePath = QFileDialog::getOpenFileName(nullptr, dialogText, QDir::homePath(), "Images (*.png *.jpg *.jpeg *.jfif, *.jpe *.tif *.bmp *.pdm *.ppm *.xdm *.xpm)");
//        if(filePath.size() == 0){
//            return;
//        }

//        photo = std::make_shared<Photo>(Photo(filePath));
//        actionImage->setIcon(QPixmap::fromImage(photo->scaledPhoto.scaled(iconeSIze.width(), iconeSIze.height(), Qt::KeepAspectRatio)));
//        emit settings_updated_signal(false);
//    });
//    tb->setDefaultAction(actionImage);
//}

//void SettingsW::init_pushbuttons_connections(QVector<QPushButton *> buttons, bool displayZones){
//    for(auto &&button : buttons){
//        connect(button, &QPushButton::clicked, this, [=]{
//            emit settings_updated_signal(displayZones);
//        });
//    }
//}

//void pc::SettingsW::init_sliders_connections(QVector<QSlider *> sliders, bool displayZones){
//    for(auto &&slider : sliders){
//        connect(slider, &QSlider::valueChanged, this, [=]{
//            emit settings_updated_signal(displayZones);
//        });
//    }
//}

//void pc::SettingsW::init_spinboxes_connections(QVector<QSpinBox *> spinBoxes, bool displayZones){
//    for(auto &&spinBox : spinBoxes){
//        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=]{
//            emit settings_updated_signal(displayZones);
//        });
//    }
//}

//void pc::SettingsW::init_doublespinboxes_connections(QVector<QDoubleSpinBox *> dSpinBoxes, bool displayZones){
//    for(auto &&dSpinBox : dSpinBoxes){
//        connect(dSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=]{
//            emit settings_updated_signal(displayZones);;
//        });
//    }
//}

//void SettingsW::init_radiobuttons_connections(QVector<QRadioButton *> buttons, bool displayZones){

//    for(auto &&cb : buttons){
//        connect(cb, &QRadioButton::clicked, this, [=]{
//            emit settings_updated_signal(displayZones);
//        });
//    }
//}




//int TextEdit::currentDroppedImage = 0;


//#ifdef Q_OS_MAC
//const QString rsrcPath = ":/images/mac";
//#else
//const QString rsrcPath = ":/images";
//#endif

//RichTextEditW::RichTextEditW():  SettingsW() {

//#ifdef Q_OS_OSX
//    setUnifiedTitleAndToolBarOnMac(true);
//#endif
//    setWindowTitle(QCoreApplication::applicationName());

////    m_textEdit = new TextEdit();
//    m_textEdit = new Test();
//    m_backGround.setAlpha(0);

//    QFont textFont("Calibri");
//    textFont.setStyleHint(QFont::SansSerif);

//    QTextCharFormat format;
//    format.setFont(textFont);
//    format.setBackground(Qt::white);
//    format.setFont(textFont);
//    format.setForeground(Qt::black);
//    textEdit()->setCurrentCharFormat(format);
//    connect(textEdit(), &TextEdit::currentCharFormatChanged,this, &RichTextEditW::current_char_format_changed);
//    connect(textEdit(), &TextEdit::cursorPositionChanged,this, &RichTextEditW::cursor_position_changed);
//    connect(textEdit(), &TextEdit::textChanged, this, []{});

//    // set ui
//    m_mainLayout = new QVBoxLayout();
//    m_mainLayout->setContentsMargins(0,0,0,0);

//    QWidget *menuWTop = new QWidget();
//    QWidget *menuWCenter = new QWidget();
//    QWidget *menuWBottom = new QWidget();
//    m_menuLayoutTop = new QHBoxLayout();
//    m_menuLayoutCenter = new QHBoxLayout();
//    m_menuLayoutBottom = new QHBoxLayout();
//    menuWTop->setLayout(m_menuLayoutTop);
//    menuWCenter->setLayout(m_menuLayoutCenter);
//    menuWBottom->setLayout(m_menuLayoutBottom);

//    this->setLayout(m_mainLayout);
//    m_mainLayout->addWidget(menuWTop);
//    m_mainLayout->addWidget(menuWCenter);

//    QFrame* line = new QFrame();
//    line->setGeometry(QRect(/* ... */));
//    line->setFrameShape(QFrame::HLine); // Replace by VLine for vertical line
//    line->setFrameShadow(QFrame::Sunken);
//    m_mainLayout->addWidget(line);
//    m_mainLayout->addWidget(menuWBottom);
//    m_mainLayout->addWidget(m_textEdit);
//    m_mainLayout->setStretchFactor(m_textEdit, 30);

//    m_mainLayout->setSpacing(2);
//    m_mainLayout->setContentsMargins(1,1,1,1);
//    m_menuLayoutTop->setContentsMargins(1,1,1,1);
//    m_menuLayoutTop->setSpacing(1);
//    m_menuLayoutCenter->setContentsMargins(1,1,1,1);
//    m_menuLayoutCenter->setSpacing(1);
//    m_menuLayoutBottom->setContentsMargins(1,1,1,1);
//    m_menuLayoutBottom->setSpacing(1);

//    setup_edit_actions();
//    setup_text_actions();

//    font_changed(textEdit()->font());
//    color_text_changed(textEdit()->textColor());
//    background_color_text_changed(textEdit()->textBackgroundColor());
//    alignment_changed(textEdit()->alignment());

//    connect(textEdit()->document(), &QTextDocument::modificationChanged, this, &QWidget::setWindowModified);
//    connect(textEdit()->document(), &QTextDocument::undoAvailable, actionUndo, &QAction::setEnabled);
//    connect(textEdit()->document(), &QTextDocument::redoAvailable,actionRedo, &QAction::setEnabled);

//    setWindowModified(textEdit()->document()->isModified());
//    actionUndo->setEnabled(textEdit()->document()->isUndoAvailable());
//    actionRedo->setEnabled(textEdit()->document()->isRedoAvailable());
//    actionCut->setEnabled(true);
//    actionCopy->setEnabled(true);
//    connect(QApplication::clipboard(), &QClipboard::dataChanged, this, &RichTextEditW::clipboard_data_changed);

//    textEdit()->setFocus();

//    // send html
//    connect(textEdit(), &TextEdit::textChanged, this, [=]{
//        m_html = std::make_shared<QString>(textEdit()->toHtml());
//        emit settings_updated_signal(false);
//    });
//}

//void RichTextEditW::init_style(RichTextType type){

//    // init html
//    switch(type){
//    case RichTextType::individualConsign:
//        init_as_individual_consign();
//        init_colors_icons(qRgba(0,0,0,255), qRgba(225,225,225,0));
//        break;
//    case RichTextType::globalConsign:
//        init_colors_icons(qRgba(0,0,0,255), qRgba(225,225,225,0));
//        init_as_consign();
//        break;
//    case RichTextType::footer:
//        init_colors_icons(qRgba(0,0,0,255), qRgba(225,225,225,0));
//        init_as_footer();
//        break;
//    case RichTextType::header:
//        init_colors_icons(qRgba(0,0,0,255), qRgba(225,225,225,0));
//        init_as_header();
//        break;
//    }
//}

//void RichTextEditW::init_as_title(){

//    QString html = "<p align=\"center\" align=\"center style= margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">"
//    "<span style=\"font-family:'Calibri'; font-size:20pt; background-color:transparent;\">" //
//    "<br />Entrez votre titre ici..."
//    "</span>";
//    textEdit()->setHtml(html);
//    m_comboSize->setCurrentIndex(9);
//}

//void RichTextEditW::init_as_footer(){

//    QString html = "<p align=\"center\" align=\"center style= margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">"
//    "<span style=\"font-family:'Calibri'; font-size:20pt; background-color:transparent;\">" //
//    "<br />Texte de bas de page..."
//    "</span>";
//    textEdit()->setHtml(html);
//    m_comboSize->setCurrentIndex(9);
//}


//void RichTextEditW::init_as_header(){

//    QString html = "<p align=\"center\" align=\"center style= margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">"
//    "<span style=\"font-family:'Calibri'; font-size:20pt; background-color:transparent;\">" //
//    "<br />Texte de haut de page..."
//    "</span>";
//    textEdit()->setHtml(html);
//    m_comboSize->setCurrentIndex(9);
//}

//void RichTextEditW::init_as_consign(){

//    QString html = "<p align=\"center\" align=\"center style= margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">"
//    "<span style=\"font-family:'Calibri'; font-size:14pt; background-color:transparent;\">"
//    "<br />Texte commun à toutes les photos..."
//    "</span>";
//    textEdit()->setHtml(html);
//    m_comboSize->setCurrentIndex(7);
//}

//void RichTextEditW::init_as_individual_consign(){

//    QString html = "<p align=\"center\" align=\"center style= margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">"
//    "<span style=\"font-family:'Calibri'; font-size:14pt; background-color:transparent;\">"
//    "<br />Texte individuel..."
//    "</span>";
//    textEdit()->setHtml(html);
//    m_comboSize->setCurrentIndex(7);
//}

//void RichTextEditW::init_colors_icons(QColor foreGround, QColor backGround){

//    color_text_changed(foreGround);
//    background_color_text_changed(backGround);
//}

//void RichTextEditW::init_with_another(const RichTextEditW &richTextEdit, std::shared_ptr<QString> html){

//    if(html != nullptr){
//        textEdit()->blockSignals(true);
//        textEdit()->setHtml(*html);
//        m_html = std::make_shared<QString>(*html);
//        textEdit()->blockSignals(false);
//    }

//    m_comboStyle->blockSignals(true);
//    m_comboStyle->setCurrentIndex(richTextEdit.m_comboSize->currentIndex());
//    m_comboStyle->blockSignals(false);
//    m_comboCodes->blockSignals(true);
//    m_comboCodes->setCurrentIndex(richTextEdit.m_comboCodes->currentIndex());
//    m_comboCodes->blockSignals(false);
//    m_comboSize->blockSignals(true);
//    m_comboSize->setCurrentIndex(richTextEdit.m_comboSize->currentIndex());
//    m_comboSize->blockSignals(false);
//    m_comboFont->blockSignals(true);
//    m_comboFont->setCurrentIndex(richTextEdit.m_comboFont->currentIndex());
//    m_comboFont->blockSignals(false);

//    m_boldButton->blockSignals(true);
//    m_boldButton->setChecked(richTextEdit.m_boldButton->isChecked());
//    m_boldButton->blockSignals(false);
//    m_italicButton->blockSignals(true);
//    m_italicButton->setChecked(richTextEdit.m_italicButton->isChecked());
//    m_italicButton->blockSignals(false);
//    m_underlineButton->blockSignals(true);
//    m_underlineButton->setChecked(richTextEdit.m_underlineButton->isChecked());
//    m_underlineButton->blockSignals(false);
//    m_overlineButton->blockSignals(true);
//    m_overlineButton->setChecked(richTextEdit.m_overlineButton->isChecked());
//    m_overlineButton->blockSignals(false);
//    m_leftAButton->blockSignals(true);
//    m_leftAButton->setChecked(richTextEdit.m_leftAButton->isChecked());
//    m_leftAButton->blockSignals(false);

//    m_centerRButton->blockSignals(true);
//    m_centerRButton->setChecked(richTextEdit.m_centerRButton->isChecked());
//    m_centerRButton->blockSignals(false);
//    m_centerAButton->blockSignals(true);
//    m_centerAButton->setChecked(richTextEdit.m_centerAButton->isChecked());
//    m_centerAButton->blockSignals(false);
//    m_justifyButton->blockSignals(true);
//    m_justifyButton->setChecked(richTextEdit.m_justifyButton->isChecked());
//    m_justifyButton->blockSignals(false);

//    m_colorTextButton->blockSignals(true);
//    m_colorTextButton->setIcon(richTextEdit.m_colorTextButton->icon());
//    m_colorTextButton->blockSignals(false);
//    m_backgroundColorTextButton->blockSignals(true);
//    m_backgroundColorTextButton->setIcon(richTextEdit.m_backgroundColorTextButton->icon());
//    m_backgroundColorTextButton->blockSignals(false);
//}


//void RichTextEditW::write_to_xml(QXmlStreamWriter &xml) const{

//    xml.writeStartElement("RichText");
//    const auto forGroundColorStr = QString::number(m_foreGround.red()) +  " " + QString::number(m_foreGround.green()) + " " + QString::number(m_foreGround.blue()) + " " + QString::number(m_foreGround.alpha());
//    const auto backGroundColorStr = QString::number(m_backGround.red()) +  " " + QString::number(m_backGround.green()) + " " + QString::number(m_backGround.blue()) + " " + QString::number(m_backGround.alpha());
//    xml.writeAttribute("foreGroundColor", forGroundColorStr);
//    xml.writeAttribute("backGroundColor", backGroundColorStr);
//    xml.writeTextElement("xml", (m_html != nullptr) ? *m_html : "");
//    xml.writeEndElement();
//}

//void RichTextEditW::load_from_xml(QXmlStreamReader &xml){

//    QStringList col = xml.attributes().value("foreGroundColor").toString().split(" ");
//    m_foreGround = QColor(col[0].toInt(), col[1].toInt(),col[2].toInt(),col[3].toInt());
//    col = xml.attributes().value("backGroundColor").toString().split(" ");
//    m_backGround = QColor(col[0].toInt(), col[1].toInt(),col[2].toInt(),col[3].toInt());

//    QXmlStreamReader::TokenType token = QXmlStreamReader::TokenType::StartElement;
//    while(!xml.hasError()) {

//        if(token == QXmlStreamReader::TokenType::EndElement && xml.name() == "RichText"){
//            break;

//        }else if(token == QXmlStreamReader::TokenType::StartElement){

//            if(xml.name() == "xml"){

//                textEdit()->blockSignals(true);
//                textEdit()->setHtml(xml.readElementText());
//                m_html = std::make_shared<QString>(textEdit()->toHtml());
//                textEdit()->blockSignals(false);
//            }
//        }

//        token = xml.readNext();
//    }
//}

//void RichTextEditW::setup_edit_actions()
//{
//    m_menuLayoutCenter->setContentsMargins(0,0,0,0);

//    // undo
//    const QIcon undoIcon = QIcon(":/images/editundo");
//    actionUndo = new QAction(undoIcon,tr("Annuler"), this );
//    actionUndo->setShortcut(QKeySequence::Undo);
//    connect(actionUndo, &QAction::triggered, textEdit(), &QTextEdit::undo);

//    QToolButton *undoButton = new QToolButton();
//    undoButton->setDefaultAction(actionUndo);
//    undoButton->setIcon(undoIcon);
//    m_menuLayoutCenter->addWidget(undoButton);

//    // redo
//    const QIcon redoIcon = QIcon(":/images/editredo");
//    actionRedo = new QAction(redoIcon,tr("Rétablir"), this );
//    connect(actionRedo, &QAction::triggered, textEdit(), &QTextEdit::redo);
//    actionRedo->setShortcut(QKeySequence::Redo);
//    actionRedo->setPriority(QAction::LowPriority);

//    QToolButton *redoButton = new QToolButton();
//    redoButton->setDefaultAction(actionRedo);
//    m_menuLayoutCenter->addWidget(redoButton);

//    m_menuLayoutCenter->addSpacing(8);

//    // cut
//    const QIcon cutIcon = QIcon(":/images/editcut");
//    actionCut = new QAction(cutIcon,tr("Couper"), this );
//    connect(actionCut, &QAction::triggered, textEdit(), &QTextEdit::cut);
//    actionCut->setPriority(QAction::LowPriority);
//    actionCut->setShortcut(QKeySequence::Cut);

//    QToolButton *cutButton = new QToolButton();
//    cutButton->setDefaultAction(actionCut);
//    m_menuLayoutCenter->addWidget(cutButton);

//    // copy
//    const QIcon copyIcon = QIcon(":/images/editcopy");
//    actionCopy = new QAction(copyIcon,tr("Copier"), this );
//    connect(actionCopy, &QAction::triggered, textEdit(), &QTextEdit::copy);
//    actionCopy->setShortcut(QKeySequence::Copy);
//    actionCopy->setPriority(QAction::LowPriority);

//    QToolButton *copyButton = new QToolButton();
//    copyButton->setDefaultAction(actionCopy);
//    m_menuLayoutCenter->addWidget(copyButton);

//    // paste
//    const QIcon pasteIcon = QIcon(":/images/editpaste");
//    actionPaste = new QAction(pasteIcon,tr("Coller"), this );
//    connect(actionPaste, &QAction::triggered, textEdit(), &QTextEdit::paste);
//    actionPaste->setShortcut(QKeySequence::Paste);
//    actionPaste->setPriority(QAction::LowPriority);

//    QToolButton *pasteButton = new QToolButton();
//    pasteButton->setDefaultAction(actionPaste);
//    m_menuLayoutCenter->addWidget(pasteButton);

//    if (const QMimeData *md = QApplication::clipboard()->mimeData()){
//        actionPaste->setEnabled(md->hasText());
//    }

//    m_menuLayoutCenter->addSpacing(8);
//}

//void RichTextEditW::setup_text_actions(){

//    // bold
//    const QIcon boldIcon = QIcon(":/images/textbold");
//    actionTextBold = new QAction(boldIcon, tr("Gras"), this);
//    connect(actionTextBold, &QAction::triggered, this, &RichTextEditW::text_bold);
//    actionTextBold->setShortcut(Qt::CTRL + Qt::Key_B);
//    actionTextBold->setPriority(QAction::LowPriority);

//    QFont bold;
//    bold.setBold(true);
//    actionTextBold->setFont(bold);
//    actionTextBold->setCheckable(true);

//    m_boldButton = new QToolButton();
//    m_boldButton->setDefaultAction(actionTextBold);
//    m_boldButton->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonFollowStyle);
//    m_menuLayoutCenter->addWidget(m_boldButton);
//    connect(m_boldButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });


//    // italic
//    const QIcon italicIcon = QIcon(":/images/textitalic");
//    actionTextItalic = new QAction(italicIcon, tr("Italic"), this);
//    connect(actionTextItalic, &QAction::triggered, this, &RichTextEditW::text_italic);
//    actionTextItalic->setPriority(QAction::LowPriority);
//    actionTextItalic->setShortcut(Qt::CTRL + Qt::Key_I);

//    QFont italic;
//    italic.setItalic(true);
//    actionTextItalic->setFont(italic);
//    actionTextItalic->setCheckable(true);

//    m_italicButton = new QToolButton();
//    m_italicButton->setDefaultAction(actionTextItalic);
//    m_menuLayoutCenter->addWidget(m_italicButton);
//    connect(m_italicButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    // underline
//    const QIcon underlineIcon = QIcon(":/images/textunder");
//    actionTextUnderline = new QAction(underlineIcon, tr("Souligner"), this);
//    connect(actionTextUnderline, &QAction::triggered, this, &RichTextEditW::text_underline);
//    actionTextUnderline->setShortcut(Qt::CTRL + Qt::Key_U);
//    actionTextUnderline->setPriority(QAction::LowPriority);

//    QFont underline;
//    underline.setUnderline(true);
//    actionTextUnderline->setFont(underline);
//    actionTextUnderline->setCheckable(true);

//    m_underlineButton = new QToolButton();
//    m_underlineButton->setDefaultAction(actionTextUnderline);
//    m_menuLayoutCenter->addWidget(m_underlineButton);
//    connect(m_underlineButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    // overline
//    const QIcon overlineIcon = QIcon(":/images/overline");
//    actionTextOverline = new QAction(overlineIcon, tr("Surligner"), this);
//    connect(actionTextOverline, &QAction::triggered, this, &RichTextEditW::text_overline);
//    actionTextOverline->setShortcut(Qt::CTRL + Qt::Key_O);
//    actionTextOverline->setPriority(QAction::LowPriority);

//    QFont overline;
//    overline.setOverline(true);
//    actionTextOverline->setFont(overline);
//    actionTextOverline->setCheckable(true);

//    m_overlineButton = new QToolButton();
//    m_overlineButton->setDefaultAction(actionTextOverline);
//    m_menuLayoutCenter->addWidget(m_overlineButton);
//    connect(m_overlineButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    // strike
//    const QIcon strikeIcon = QIcon(":/images/strike");
//    actionTextStrike = new QAction(strikeIcon, tr("Barrer"), this);
//    connect(actionTextStrike, &QAction::triggered, this, &RichTextEditW::text_strike);
//    actionTextStrike->setShortcut(Qt::CTRL + Qt::Key_S);
//    actionTextStrike->setPriority(QAction::LowPriority);

//    QFont strike;
//    strike.setStrikeOut(true);
//    actionTextStrike->setFont(strike);
//    actionTextStrike->setCheckable(true);

//    m_strikeButton = new QToolButton();
//    m_strikeButton->setDefaultAction(actionTextStrike);
//    m_menuLayoutCenter->addWidget(m_strikeButton);
//    connect(m_strikeButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    m_menuLayoutCenter->addSpacing(8);
//    // left
//    const QIcon leftIcon = QIcon(":/icons/textLeft");
//    actionAlignLeft = new QAction(leftIcon, tr("Aligner à gauche"), this);
//    actionAlignLeft->setShortcut(Qt::CTRL + Qt::Key_L);
//    actionAlignLeft->setCheckable(true);
//    actionAlignLeft->setPriority(QAction::LowPriority);

//    m_leftAButton = new QToolButton();
//    m_leftAButton->setDefaultAction(actionAlignLeft);
//    m_menuLayoutCenter->addWidget(m_leftAButton);
//    connect(m_leftAButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    // center
//    const QIcon centerIcon = QIcon(":/images/textcenter");
//    actionAlignCenter = new QAction(centerIcon, tr("Centrer"), this);
//    actionAlignCenter->setShortcut(Qt::CTRL + Qt::Key_E);
//    actionAlignCenter->setCheckable(true);
//    actionAlignCenter->setPriority(QAction::LowPriority);

//    m_centerAButton = new QToolButton();
//    m_centerAButton->setDefaultAction(actionAlignCenter);
//    m_menuLayoutCenter->addWidget(m_centerAButton);
//    connect(m_centerAButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    // right
//    const QIcon rightIcon = QIcon(":/icons/textRight");
//    actionAlignRight = new QAction(rightIcon, tr("Aligner à droite"), this);
//    actionAlignRight->setShortcut(Qt::CTRL + Qt::Key_R);
//    actionAlignRight->setCheckable(true);
//    actionAlignRight->setPriority(QAction::LowPriority);

//    m_centerRButton = new QToolButton();
//    m_centerRButton->setDefaultAction(actionAlignRight);
//    m_menuLayoutCenter->addWidget(m_centerRButton);
//    connect(m_centerRButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    // justify
//    const QIcon justifyIcon = QIcon(":/images/textjustify");
//    actionAlignJustify = new QAction(justifyIcon, tr("Justifier"), this);
//    actionAlignJustify->setShortcut(Qt::CTRL + Qt::Key_J);
//    actionAlignJustify->setCheckable(true);
//    actionAlignJustify->setPriority(QAction::LowPriority);

//    m_justifyButton = new QToolButton();
//    m_justifyButton->setDefaultAction(actionAlignJustify);
//    m_menuLayoutCenter->addWidget(m_justifyButton);
//    connect(m_justifyButton,&QToolButton::clicked, this, [=]{
//        textEdit()->setFocus();
//    });

//    m_menuLayoutCenter->addStretch();
//    m_insertLabel = new QLabel("Insérer: ");
//    m_insertLabel->setStyleSheet("QWidget::enabled {color : rgb(0,106,255);}");
//    m_menuLayoutBottom->addWidget(m_insertLabel);
//    m_menuLayoutBottom->addSpacing(8);

//    // image insert
//    const QIcon imageIcon = QIcon(":/images/insertimage");
//    actionInsertImage = new QAction(imageIcon, tr("Insérer image"), this);
//    actionInsertImage->setShortcut(Qt::CTRL + Qt::Key_P);
//    actionInsertImage->setCheckable(false);
//    actionInsertImage->setPriority(QAction::LowPriority);

//    m_insertImageButton = new QToolButton();
//    m_insertImageButton->setDefaultAction(actionInsertImage);
//    m_menuLayoutBottom->addWidget(m_insertImageButton);
//    connect(m_insertImageButton,&QToolButton::triggered, this, [=]{
//        textEdit()->insert_image();
//        textEdit()->setFocus();
//    });

//    m_menuLayoutBottom->addSpacing(4);

//    // web link
//    const QIcon linkIcon = QIcon(":/images/link");
//    actionLink = new QAction(linkIcon, tr("Insérer lien web"), this);
//    actionLink->setShortcut(Qt::CTRL + Qt::Key_W);
//    actionLink->setCheckable(false);
//    actionLink->setPriority(QAction::LowPriority);

//    m_insertLinkButton = new QToolButton();
//    m_insertLinkButton->setDefaultAction(actionLink);
//    m_menuLayoutBottom->addWidget(m_insertLinkButton);

//    connect(m_insertLinkButton, &QToolButton::triggered, this, [=]{
//        textEdit()->insert_link();
//        textEdit()->setFocus();
//    });

//    m_menuLayoutBottom->addSpacing(4);

//    // textes codes inserts
//    m_comboCodes = new QComboBox();
//    m_menuLayoutBottom->addWidget(m_comboCodes);
//    m_comboCodes->setStyleSheet("color : rgb(0,106,255);");
//    m_comboCodes->setToolTip("Insérer un code, il sera remplacé dans l'aperçu.");
//    m_comboCodes->addItem("Nom photo");
//    m_comboCodes->addItem("Date");
//    m_comboCodes->addItem("Date de la photo");
//    m_comboCodes->addItem("Numéro de la photo");
//    m_comboCodes->addItem("Numéro de page");
//    m_comboCodes->addItem("Numéro total de pages");
//    m_comboCodes->addItem("Numéro total de photos");
//    m_comboCodes->addItem("Nom de la page");
//    connect(m_comboCodes, QOverload<int>::of(&QComboBox::activated), this, [=](int index){
//        m_comboCodes->clearFocus();
//        textEdit()->setFocus();
//        QString code;
//        switch(index){
//        case 0:
//            code = "$nom_photo$";
//            break;
//        case 1:
//            code = "$date$";
//            break;
//        case 2:
//            code = "$date_photo$";
//            break;
//        case 3:
//            code = "$num_photo$";
//            break;
//        case 4:
//            code = "$num_page$";
//            break;
//        case 5:
//            code = "$nb_pages$";
//            break;
//        case 6:
//            code = "$nb_photos$";
//            break;
//        case 7:
//            code = "$nom_page$";
//            break;
//        }

//        textEdit()->textCursor().insertText(code);
//    });

//    m_menuLayoutBottom->addSpacing(4);



//    m_comboStyle = new QComboBox();
//    m_comboStyle->setStyleSheet("color : rgb(0,106,255);");
//    m_comboStyle->setMaximumWidth(230);
//    m_menuLayoutBottom->addWidget(m_comboStyle);
//    m_comboStyle->setToolTip("Insérer une liste");
//    m_comboStyle->addItem("Standard (pas de liste)"); // Standard
//    m_comboStyle->addItem("Liste en balle (Disque)"); // Bullet List (Disc)
//    m_comboStyle->addItem("Liste en balle (Cercle)"); // Bullet List (Circle)
//    m_comboStyle->addItem("Liste en balle (Carré)"); // Bullet List (Square)
//    m_comboStyle->addItem("Liste ordonnée (Décimale)"); // Ordered List (Decimal)
//    m_comboStyle->addItem("Liste ordonnée (Alpha min.)"); // Ordered List (Alpha lower)
//    m_comboStyle->addItem("Liste ordonnée (Alpha maj.)"); // Ordered List (Alpha upper)
//    m_comboStyle->addItem("Liste ordonnée (Roman min.)"); // Ordered List (Roman lower)
//    m_comboStyle->addItem("Liste ordonnée (Roman maj.)"); // Ordered List (Roman upper)


//    connect(m_comboStyle, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=]{
//        m_comboStyle->clearFocus();
//        textEdit()->setFocus();
//    });

//    typedef void (QComboBox::*QComboIntSignal)(int);
//    connect(m_comboStyle, static_cast<QComboIntSignal>(&QComboBox::activated), this, &RichTextEditW::text_style);

//    m_menuLayoutBottom->addStretch();

//    // Make sure the alignLeft  is always left of the alignRight
//    QActionGroup *alignGroup = new QActionGroup(this);
//    connect(alignGroup, &QActionGroup::triggered, this, &RichTextEditW::text_align);

//    if (QApplication::isLeftToRight()) {
//        alignGroup->addAction(actionAlignLeft);
//        alignGroup->addAction(actionAlignCenter);
//        alignGroup->addAction(actionAlignRight);
//    } else {
//        alignGroup->addAction(actionAlignRight);
//        alignGroup->addAction(actionAlignCenter);
//        alignGroup->addAction(actionAlignLeft);
//    }
//    alignGroup->addAction(actionAlignJustify);


//    // text color
//    QPixmap pixColor(16, 16);
//    pixColor.fill(Qt::black);
//    actionTextColor = new QAction(pixColor, tr("Couleur du texte"), this);
//    actionTextColor->setShortcut(Qt::CTRL + Qt::Key_Q);
//    connect(actionTextColor, &QAction::triggered, this, [=]{
//        text_color();
//        textEdit()->setFocus();
//    });

//    m_colorTextButton = new QToolButton();
//    m_colorTextButton->setDefaultAction(actionTextColor);
//    m_menuLayoutTop->addWidget(m_colorTextButton);
//    m_menuLayoutTop->addSpacing(4);

//    // background text color
//    QPixmap pixBgColor(16, 16);
//    pixBgColor.fill(Qt::white);
//    actionBackgroundTextColor = new QAction(pixBgColor, tr("Couleur de l'arrière plan du texte"), this);
//    actionBackgroundTextColor->setShortcut(Qt::CTRL + Qt::Key_F);
//    connect(actionBackgroundTextColor, &QAction::triggered, this, [=]{
//        background_text_color();
//        textEdit()->setFocus();
//    });
//    m_backgroundColorTextButton = new QToolButton();
//    m_backgroundColorTextButton->setDefaultAction(actionBackgroundTextColor);
//    m_menuLayoutTop->addWidget(m_backgroundColorTextButton);
//    m_menuLayoutTop->addSpacing(8);

//    // font
//    typedef void (QComboBox::*QComboStringSignal)(const QString &);
//    m_comboFont = new QFontComboBox();
//    m_comboFont->setToolTip("Choix de la police");
//    m_comboFont->setStyleSheet("color : rgb(0,106,255);");
//    connect(m_comboFont, static_cast<QComboStringSignal>(&QComboBox::activated), this, [=](QString police){
//        text_family(police);
//        textEdit()->setFocus();
//    });
//    m_menuLayoutTop->addWidget(m_comboFont);
//    m_menuLayoutTop->addSpacing(4);

//    // font size
//    m_comboSize = new QComboBox();
//    m_comboSize->setToolTip("Définir la taille de la police");
//    m_comboSize->setStyleSheet("color : rgb(0,106,255);");
//    m_comboSize->setObjectName("comboSize");
//    m_comboSize->setEditable(true);
//    m_comboSize->setInsertPolicy(QComboBox::InsertAtBottom);
//    m_comboSize->setValidator( new QIntValidator(0, 150, this) );
//    m_comboSize->setMinimumWidth(45);
//    m_comboSize->setMaximumWidth(45);

//    connect(m_comboSize, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=]{
//        m_comboSize->clearFocus();
//        textEdit()->setFocus();
//    });

//    m_menuLayoutTop->addWidget(m_comboSize);
//    m_menuLayoutTop->addSpacing(4);

//    const QList<int> standardSizes = {6,7,8,9,10,11,12,13,14,15,16,18,20,22,24,26,28,30,32,34,36,40,44,48,52,56,60,66,72,78,84,90,100,120};// QFontDatabase::standardSizes();
//    foreach (int size, standardSizes)
//        m_comboSize->addItem(QString::number(size));
//    m_comboSize->setCurrentIndex(standardSizes.indexOf(QApplication::font().pointSize()));

//    connect(m_comboSize, static_cast<QComboStringSignal>(&QComboBox::activated), this, &RichTextEditW::text_size);

//    // zoom
//    m_comboZoom = new QComboBox();
//    m_comboZoom->setToolTip("Définir le zoom de la zone d'édition");
//    m_comboZoom->setStyleSheet("color : rgb(0,106,255);");
//    m_comboZoom->setObjectName("comboZoom");
//    m_comboZoom->setEditable(false);
//    m_comboZoom->setMinimumWidth(53);
//    m_comboZoom->setMaximumWidth(53);

//    QStringList items;
//    items << "10%" << "20%" << "30%" << "40%" << "50%" << "75%" << "100%";// << "150%" << "200%" << "300%";
//    m_comboZoom->addItems(items);
//    m_comboZoom->setCurrentIndex(6);
//    m_textEdit->set_scale_factor(1.);

//    connect(m_comboZoom, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=]{
//        m_comboZoom->clearFocus();
//        textEdit()->setFocus();
//    });
//    connect(m_comboZoom, static_cast<QComboStringSignal>(&QComboBox::activated), this, [&]{
//        QString txt = m_comboZoom->currentText();
//        txt.resize(txt.size()-1);
//        m_textEdit->set_scale_factor(txt.toDouble()*0.01);
//    });

//    m_menuLayoutTop->addWidget(m_comboZoom);


//    m_menuLayoutTop->addStretch();

//    return;
//}

//void RichTextEditW::text_bold(){

//    QTextCharFormat fmt;
//    fmt.setFontWeight(actionTextBold->isChecked() ? QFont::Bold : QFont::Normal);
//    merge_format_on_word_or_selection(fmt);
//}

//void RichTextEditW::text_underline(){

//    QTextCharFormat fmt;
//    fmt.setFontUnderline(actionTextUnderline->isChecked());
//    merge_format_on_word_or_selection(fmt);
//}


//void RichTextEditW::text_overline(){

//    QTextCharFormat fmt;
//    fmt.setFontOverline(actionTextOverline->isChecked());
//    merge_format_on_word_or_selection(fmt);
//}

//void RichTextEditW::text_strike(){

//    QTextCharFormat fmt;
//    fmt.setFontStrikeOut(actionTextStrike->isChecked());
//    merge_format_on_word_or_selection(fmt);
//}

//void RichTextEditW::text_italic(){

//    QTextCharFormat fmt;
//    fmt.setFontItalic(actionTextItalic->isChecked());
//    merge_format_on_word_or_selection(fmt);
//}

//void RichTextEditW::text_family(const QString &f){

//    QTextCharFormat fmt;
//    fmt.setFontFamily(f);
//    merge_format_on_word_or_selection(fmt);
//}

//void RichTextEditW::text_size(const QString &p){

//    qreal pointSize = p.toDouble();
//    if (p.toFloat() > 0) {
//        QTextCharFormat fmt;
//        fmt.setFontPointSize(pointSize);
//        merge_format_on_word_or_selection(fmt);
//    }
//}

//void RichTextEditW::text_style(int styleIndex){

//    QTextCursor cursor = textEdit()->textCursor();

//    if (styleIndex != 0) {
//        QTextListFormat::Style style = QTextListFormat::ListDisc;

//        switch (styleIndex) {
//            default:
//            case 1:
//                style = QTextListFormat::ListDisc;
//                break;
//            case 2:
//                style = QTextListFormat::ListCircle;
//                break;
//            case 3:
//                style = QTextListFormat::ListSquare;
//                break;
//            case 4:
//                style = QTextListFormat::ListDecimal;
//                break;
//            case 5:
//                style = QTextListFormat::ListLowerAlpha;
//                break;
//            case 6:
//                style = QTextListFormat::ListUpperAlpha;
//                break;
//            case 7:
//                style = QTextListFormat::ListLowerRoman;
//                break;
//            case 8:
//                style = QTextListFormat::ListUpperRoman;
//                break;
//        }

//        cursor.beginEditBlock();

//        QTextBlockFormat blockFmt = cursor.blockFormat();

//        QTextListFormat listFmt;

//        if (cursor.currentList()) {
//            listFmt = cursor.currentList()->format();
//        } else {
//            listFmt.setIndent(blockFmt.indent() + 1);
//            blockFmt.setIndent(0);
//            cursor.setBlockFormat(blockFmt);
//        }

//        listFmt.setStyle(style);

//        cursor.createList(listFmt);

//        cursor.endEditBlock();
//    } else {
//        // ####
//        QTextBlockFormat bfmt;
//        bfmt.setObjectIndex(-1);
//        cursor.mergeBlockFormat(bfmt);
//    }
//}

//void RichTextEditW::text_color(){

//    m_foreGround = QColorDialog::getColor(m_foreGround, this, "Choix de la couleur du texte", QColorDialog::ColorDialogOption::ShowAlphaChannel);
//    if (!m_foreGround.isValid()){
//        return;
//    }

//    QTextCharFormat fmt;
//    fmt.setForeground(m_foreGround);
//    merge_format_on_word_or_selection(fmt);
//    color_text_changed(m_foreGround);
//}

//void RichTextEditW::background_text_color(){

//    m_backGround = QColorDialog::getColor(m_backGround, this, "CHoix de la couleur de l'arrière-plan du texte", QColorDialog::ColorDialogOption::ShowAlphaChannel);
//    if (!m_backGround.isValid()){
//        return;
//    }

//    QTextCharFormat fmt;
//    fmt.setBackground(m_backGround);
//    merge_format_on_word_or_selection(fmt);
//    background_color_text_changed(m_backGround);
//}

//void RichTextEditW::text_align(QAction *a){

//    if (a == actionAlignLeft)
//        textEdit()->setAlignment(Qt::AlignLeft | Qt::AlignAbsolute);
//    else if (a == actionAlignCenter)
//        textEdit()->setAlignment(Qt::AlignHCenter);
//    else if (a == actionAlignRight)
//        textEdit()->setAlignment(Qt::AlignRight | Qt::AlignAbsolute);
//    else if (a == actionAlignJustify)
//        textEdit()->setAlignment(Qt::AlignJustify);
//}

//void RichTextEditW::current_char_format_changed(const QTextCharFormat &format){

//    font_changed(format.font());
//    color_text_changed(format.foreground().color());
//    background_color_text_changed(format.background().color());
//}

//void RichTextEditW::cursor_position_changed(){

//    alignment_changed(textEdit()->alignment());
//}

//void RichTextEditW::clipboard_data_changed(){

//    if (const QMimeData *md = QApplication::clipboard()->mimeData()){
//        actionPaste->setEnabled(md->hasText());
//    }
//}

//void RichTextEditW::merge_format_on_word_or_selection(const QTextCharFormat &format){

//    QTextCursor cursor = textEdit()->textCursor();
//    if (!cursor.hasSelection()){
//        cursor.select(QTextCursor::WordUnderCursor);
//    }
//    cursor.mergeCharFormat(format);

//    textEdit()->mergeCurrentCharFormat(format);
//}

//void RichTextEditW::font_changed(const QFont &f){

//    m_comboFont->setCurrentIndex(m_comboFont->findText(QFontInfo(f).family()));
//    m_comboSize->setCurrentIndex(m_comboSize->findText(QString::number(f.pointSize())));
//    actionTextBold->setChecked(f.bold());
//    actionTextItalic->setChecked(f.italic());
//    actionTextOverline->setChecked(f.overline());
//    actionTextStrike->setChecked(f.strikeOut());
//    actionTextUnderline->setChecked(f.underline());;
//}

//void RichTextEditW::color_text_changed(const QColor &c){

//    QPixmap pix(16, 16);
//    pix.fill(c);
//    actionTextColor->setIcon(pix);
//}

//void RichTextEditW::background_color_text_changed(const QColor &c){

//    QPixmap pix(16, 16);
//    pix.fill(c);
//    actionBackgroundTextColor->setIcon(pix);
//}

//void RichTextEditW::alignment_changed(Qt::Alignment a){

//    if (a & Qt::AlignLeft)
//        actionAlignLeft->setChecked(true);
//    else if (a & Qt::AlignHCenter)
//        actionAlignCenter->setChecked(true);
//    else if (a & Qt::AlignRight)
//        actionAlignRight->setChecked(true);
//    else if (a & Qt::AlignJustify)
//        actionAlignJustify->setChecked(true);
//}

//bool TextEdit::canInsertFromMimeData(const QMimeData *source) const {
//    return source->hasImage() || source->hasUrls() || QTextEdit::canInsertFromMimeData(source);
//}

//void TextEdit::insertFromMimeData(const QMimeData *source){

//    if (source->hasImage()){

//        QUrl url(QString("dropped_image_%1").arg(currentDroppedImage++));
//        drop_image(url, qvariant_cast<QImage>(source->imageData()));
//    }
//    else if (source->hasUrls()) {

//        foreach (QUrl url, source->urls()) {

//            QFileInfo info(url.toLocalFile());
//            if (QImageReader::supportedImageFormats().contains(info.suffix().toLower().toLatin1()))
//                drop_image(url, QImage(info.filePath()));
//            else
//                drop_text_file(url);
//        }
//    }
//    else{
//        QTextEdit::insertFromMimeData(source);
//    }
//}

//void TextEdit::insert_image(){

//    QString file = QFileDialog::getOpenFileName(nullptr, tr("Sélectionner une image"), QDir::homePath(), tr("Images files (*.bmp *.jpg *jpeg *.gif *.png)"));
//    if(file.size() == 0){
//        return;
//    }

//    QUrl url ( QString ( "file://%1" ).arg ( file ) );
//    QImage image = QImageReader ( file ).read();
//    document()->addResource(QTextDocument::ImageResource, url, image);
//    emit resource_added_signal(url, image);

//    QTextImageFormat imageFormat;
//    imageFormat.setWidth( image.width() );
//    imageFormat.setHeight( image.height() );
//    imageFormat.setName( url.toString() );
//    textCursor().insertImage(imageFormat);
//}

//void TextEdit::insert_link(){

//    m_insertLinkW = std::make_unique<QWidget>();
//    m_insertLinkW->setWindowIcon(QIcon(":/images/icon"));
//    m_insertLinkW->setWindowModality(Qt::ApplicationModal);

//    Ui::InsertLinkW insertLink;
//    insertLink.setupUi(m_insertLinkW.get());

//    connect(insertLink.pbCancel, &QPushButton::clicked, this, [&]{
//        m_insertLinkW = nullptr;
//    });

//    connect(insertLink.pbValidate, &QPushButton::clicked, this, [&]{

//        QString link = m_insertLinkW->findChild<QLineEdit *>("leLink")->text();
//        QString text = m_insertLinkW->findChild<QLineEdit *>("leText")->text();

//        if(text.size() == 0 || link.size() == 0){
//            m_insertLinkW = nullptr;
//            return;
//        }

//        QTextCursor cursor = textCursor();
//        if (!cursor.hasSelection()){
//            cursor.select(QTextCursor::WordUnderCursor);
//        }

//        insertPlainText("$$LINK$$");
//        setHtml(toHtml().replace("$$LINK$$", "<a href='" + link + "'>" + text + "</a>"));
//        m_insertLinkW = nullptr;
//    });

//    m_insertLinkW->show();

//}

//void TextEdit::drop_image(const QUrl &url, const QImage &image){

//    if (!image.isNull()){

//        document()->addResource(QTextDocument::ImageResource, url, image);
//        emit resource_added_signal(url, image);

//        QTextImageFormat format;
//        format.setWidth(image.width());
//        format.setHeight(image.height());
//        format.setName(url.toString());
//        textCursor().insertImage(format);
//    }
//}

//void TextEdit::drop_text_file(const QUrl &url){

//    QFile file(url.toLocalFile());
//    if (file.open(QIODevice::ReadOnly | QIODevice::Text)){
//        textCursor().insertText(file.readAll());
//    }
//}
