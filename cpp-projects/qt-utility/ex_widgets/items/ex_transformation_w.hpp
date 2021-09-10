
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
#include <QGridLayout>

// local
#include "qt_str.hpp"
#include "ex_vector3d_w.hpp"
#include "ex_line_edit_w.hpp"

namespace tool::ex {

class ExTransformationW : public ExItemW<QFrame>{

    constexpr static Vector3dSettings DefaultTranVec    = {ExVector3dW::DefautlTraV,ExVector3dW::DefautlTraV,ExVector3dW::DefautlTraV};
    constexpr static Vector3dSettings DefaultRotVec     = {ExVector3dW::DefaultRotV,ExVector3dW::DefaultRotV,ExVector3dW::DefaultRotV};
    constexpr static Vector3dSettings DefaultScaVec     = {ExVector3dW::DefaultScaV,ExVector3dW::DefaultScaV,ExVector3dW::DefaultScaV};

public:

    ExTransformationW(QString name ="");
    ExTransformationW *init_widget(const QString &title, Vector3dSettings transS = DefaultTranVec, Vector3dSettings rotationS = DefaultRotVec, Vector3dSettings scaleS = DefaultScaVec, bool enabled = true);

    void update_from_arg(const Arg &arg) override;
    Arg convert_to_arg() const override;
    void set_generator(QString genName) override;

    void set_from_transform(ExTransformationW *tr);
    void set_enable_state(bool enableTranslation, bool enableRotation, bool enableScale);

    void set_trans_decimals(int d);
    void set_trans_min(geo::Pt3<qreal> m);
    void set_trans_max(geo::Pt3<qreal> m);
    void set_trans_steps(geo::Pt3<qreal> s);
    void set_trans_values(geo::Pt3<qreal> v);

    void set_rotation_decimals(int d);
    void set_rotation_min(geo::Pt3<qreal> m);
    void set_rotation_max(geo::Pt3<qreal> m);
    void set_rotation_steps(geo::Pt3<qreal> s);
    void set_rotation_values(geo::Pt3<qreal> v);

    void set_scale_decimals(int d);
    void set_scale_min(geo::Pt3<qreal> m);
    void set_scale_max(geo::Pt3<qreal> m);
    void set_scale_steps(geo::Pt3<qreal> s);
    void set_scale_values(geo::Pt3<qreal> v);

public:

    QLabel *trName = nullptr;
    ExFloatSpinBoxW trX, trY, trZ;
    ExFloatSpinBoxW rotX, rotY, rotZ;
    ExFloatSpinBoxW scX, scY, scZ;

private:

    QGridLayout *layout = nullptr;
};

}
