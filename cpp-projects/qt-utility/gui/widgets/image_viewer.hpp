
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
#include <QScrollArea>
#include <QLabel>
#include <QHBoxLayout>

namespace tool::ui {

/**
 * @brief Define a QWidget displaying an image.
 */
class ImageViewer : public QWidget{

    Q_OBJECT

public:

    /**
     * @brief ImageLabel constructor
     * @param [in] parent
     */
    explicit ImageViewer(QWidget *parent = nullptr);

    /**
     * @brief Return current image pointer
     */
    const QImage* image() const;

public slots:

    /**
     * @brief Set the current image
     */
    void set_image(QImage image);


protected:

    /**
     * @brief paintEvent for drawing in the widget
     */
    virtual void paintEvent(QPaintEvent *);

    QImage m_image;
    QRectF m_imageRect;
};

}
