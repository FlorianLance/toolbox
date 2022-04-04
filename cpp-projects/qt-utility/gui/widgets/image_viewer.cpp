
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

#include "image_viewer.hpp"

// Qt
#include <QPainter>
#include <QPaintEvent>
#include <QCoreApplication>


using namespace tool::ui;

ImageViewer::ImageViewer(QWidget *parent) : QWidget(parent) {

}

void ImageViewer::paintEvent(QPaintEvent *event) {

    QWidget::paintEvent(event);

    if (m_image.isNull())
        return;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);


    QSize widgetSize = event->rect().size();
    QSize imageSize = m_image.size();
//    imageSize.scale(event->rect().size(), Qt::KeepAspectRatio);
    imageSize.scale(QSize(widgetSize.width()-2, widgetSize.height()-2), Qt::KeepAspectRatio);

    QImage scaledImage = m_image.scaled(imageSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    m_imageRect = QRectF(width()*0.5-scaledImage.size().width()*0.5,height()*0.5-scaledImage.size().height()*0.5,
                         scaledImage.width(), scaledImage.height());

    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::black);
    painter.setPen(pen);
    painter.drawImage(static_cast<int>(m_imageRect.x()), static_cast<int>(m_imageRect.y()), scaledImage);
    painter.drawRect(QRectF(m_imageRect.x()-1, m_imageRect.y(), scaledImage.width()+1, scaledImage.height()+1));
}

const QImage* ImageViewer::image() const {
    return &m_image;
}

void ImageViewer::set_image (QImage image){
    m_image = std::move(image);
    repaint();
}
