#include "NodePainter.hpp"

#include <cmath>

#include <QtCore/QMargins>

#include "StyleCollection.hpp"
#include "PortType.hpp"
#include "NodeGraphicsObject.hpp"
#include "NodeGeometry.hpp"
#include "NodeState.hpp"
#include "NodeDataModel.hpp"
#include "Node.hpp"
#include "FlowScene.hpp"

// base
#include "utility/benchmark.hpp"

using QtNodes::NodePainter;
using QtNodes::NodeGeometry;
using QtNodes::NodeGraphicsObject;
using QtNodes::Node;
using QtNodes::NodeState;
using QtNodes::NodeDataModel;
using QtNodes::FlowScene;

void NodePainter::paint(QPainter* painter, Node &node, FlowScene const& scene){

    tool::Bench::start("NodePainter::paint");

    const NodeGeometry          &geom           = node.nodeGeometry();
    const NodeState             &state          = node.nodeState();
    const NodeGraphicsObject    &graphicsObject = node.nodeGraphicsObject();

    tool::Bench::start("NodePainter::paint1");
    geom.recalculate_size(painter->font());
    tool::Bench::stop();

    //--------------------------------------------
    NodeDataModel const * model = node.nodeDataModel();

    tool::Bench::start("NodePainter::paint2");
    drawNodeRect(painter, geom, model, graphicsObject);
    drawConnectionPoints(painter, geom, state, model, scene);
    drawFilledConnectionPoints(painter, geom, state, model);
    tool::Bench::stop();

    tool::Bench::start("NodePainter::paint3");
    drawModelName(painter, geom, state, model);
    drawEntryLabels(painter, geom, state, model);
    tool::Bench::stop();

    tool::Bench::start("NodePainter::paint4");
    drawResizeRect(painter, geom, model);
    drawValidationRect(painter, geom, model, graphicsObject);
    tool::Bench::stop();

    tool::Bench::start("NodePainter::paint5");
    /// call custom painter
    if (auto painterDelegate = model->painterDelegate()){
        painterDelegate->paint(painter, geom, model);
    }
    tool::Bench::stop();

    tool::Bench::stop();
}


void NodePainter::drawNodeRect(QPainter* painter, NodeGeometry const& geom, NodeDataModel const* model,NodeGraphicsObject const & graphicsObject){

    tool::Bench::start("NodePainter::drawNodeRect");

    NodeStyle const& nodeStyle = model->node_style();
    painter->setPen(
        QPen(
            graphicsObject.isSelected() ? nodeStyle.SelectedBoundaryColor : nodeStyle.NormalBoundaryColor,
            geom.hovered() ? nodeStyle.HoveredPenWidth : nodeStyle.PenWidth
        )
    );

    QLinearGradient gradient(QPointF(0.0, 0.0), QPointF(2.0, geom.height()));
    gradient.setColorAt(0.0,    nodeStyle.GradientColor0);
    gradient.setColorAt(0.03,   nodeStyle.GradientColor1);
    gradient.setColorAt(0.97,   nodeStyle.GradientColor2);
    gradient.setColorAt(1.0,    nodeStyle.GradientColor3);

    painter->setBrush(gradient);

    const float diam = nodeStyle.ConnectionPointDiameter;
    painter->drawRoundedRect(
        QRectF( -diam, -diam, 2.0 * diam + geom.width(), 2.0 * diam + geom.height()),
        nodeRadius,
        nodeRadius
    );

    tool::Bench::stop();
}


void NodePainter::drawConnectionPoints(QPainter* painter, NodeGeometry const& geom, NodeState const& state, NodeDataModel const * model, FlowScene const & scene){

    tool::Bench::start("NodePainter::drawConnectionPoints");

    NodeStyle const& nodeStyle      = model->node_style();
    auto const &connectionStyle = StyleCollection::connectionStyle();

    const float diameter = nodeStyle.ConnectionPointDiameter;
    const auto reducedDiameter = diameter * 0.6;

    for(PortType portType: {PortType::Out, PortType::In}){

        size_t n = state.getEntries(portType).size();
        for (unsigned int i = 0; i < n; ++i){

            QPointF p = geom.port_scene_position(i, portType);
            auto const & dataType = model->port_data_type(portType, i);

            bool canConnect = (state.getEntries(portType)[i].empty() || (portType == PortType::Out && model->port_out_connection_policy(i) == NodeDataModel::ConnectionPolicy::Many) );

            double r = 1.0;
            if (state.isReacting() && canConnect && portType == state.reactingPortType()) {

                const auto   diff = geom.draggingPos() - p;
                const double dist = std::sqrt(QPointF::dotProduct(diff, diff));
                bool   typeConvertable = false;

                {
                    if (portType == PortType::In){
                        typeConvertable = scene.registry().getTypeConverter(state.reactingDataType(), dataType) != nullptr;
                    }else{
                        typeConvertable = scene.registry().getTypeConverter(dataType, state.reactingDataType()) != nullptr;
                    }
                }

                if (state.reactingDataType().id == dataType.id || typeConvertable){
                    double const thres = 40.0; r = (dist < thres) ? (2.0 - dist / thres ) : 1.0;
                } else {
                    double const thres = 80.0; r = (dist < thres) ? (dist / thres) : 1.0;
                }
            }

            if (connectionStyle.useDataDefinedColors()){
                painter->setBrush(connectionStyle.normalColor(dataType.id));
            } else {
                painter->setBrush(nodeStyle.ConnectionPointColor);
            }

            painter->drawEllipse(p, reducedDiameter * r, reducedDiameter * r);
        }
    };

    tool::Bench::stop();
}


void NodePainter::drawFilledConnectionPoints(QPainter * painter,NodeGeometry const & geom, NodeState const & state,NodeDataModel const * model){

    tool::Bench::start("NodePainter::drawFilledConnectionPoints");

    NodeStyle const& nodeStyle       = model->node_style();
    auto const     & connectionStyle = StyleCollection::connectionStyle();

    for(PortType portType: {PortType::Out, PortType::In}){

        for (size_t ii = 0; ii < state.getEntries(portType).size(); ++ii){

            if (!state.getEntries(portType)[ii].empty()){

                if (connectionStyle.useDataDefinedColors()){
                    QColor const c = connectionStyle.normalColor(model->port_data_type(portType, ii).id);
                    painter->setPen(c);
                    painter->setBrush(c);
                }else{
                    painter->setPen(nodeStyle.FilledConnectionPointColor);
                    painter->setBrush(nodeStyle.FilledConnectionPointColor);
                }

                painter->drawEllipse(
                    geom.port_scene_position(ii, portType),
                    nodeStyle.ellipseSize, nodeStyle.ellipseSize
                );
            }
        }
    }
    tool::Bench::stop();
}


void NodePainter::drawModelName(QPainter * painter,NodeGeometry const & geom,NodeState const & state,NodeDataModel const * model){

    tool::Bench::start("NodePainter::drawModelName");

    Q_UNUSED(state);

    NodeStyle const& nodeStyle = model->node_style();

    if (!model->caption_visible()){
        tool::Bench::stop();
        return;
    }

    const QString &name = model->caption();

    QFont f = painter->font();

    f.setBold(true);

    QFontMetrics metrics(f);


//    auto rect = metrics.boundingRect(name);

//    QPointF position((geom.width() - rect.width()) / 2.0,
    QPointF position(
        (geom.width() - metrics.horizontalAdvance(name)) / 2.0,
        (geom.spacing() + geom.entry_height()) / 3.0
    );

    painter->setFont(f);
    painter->setPen(nodeStyle.FontColor);
    painter->drawText(position, name);

    f.setBold(false);
    painter->setFont(f);
    tool::Bench::stop();
}

void NodePainter::drawEntryLabels(QPainter * painter,NodeGeometry const & geom,NodeState const & state, NodeDataModel const * model){

    tool::Bench::start("NodePainter::drawModelName");

    QFontMetrics const & metrics =painter->fontMetrics();
    for(PortType portType: {PortType::Out, PortType::In}){

        auto const &nodeStyle = model->node_style();
        auto& entries = state.getEntries(portType);

        size_t n = entries.size();
        for (size_t i = 0; i < n; ++i){

            QPointF p = geom.port_scene_position(i, portType);
            if (entries[i].empty()){
                painter->setPen(nodeStyle.FontColorFaded);
            }else{
                painter->setPen(nodeStyle.FontColor);
            }

            const QString &s = model->port_caption_visible(portType, i) ?
                model->port_caption(portType, i) : model->port_data_type(portType, i).name;

            const auto rect = metrics.boundingRect(s);
            p.setY(p.y() + rect.height() / 4.0);

            if(portType == PortType::In){
                p.setX(5.0);
            }else if(portType == PortType::Out){
                p.setX(geom.width() - 5.0 - rect.width());
            }

            painter->drawText(p, s);
        }
    }
    tool::Bench::stop();
}


void NodePainter::drawResizeRect(QPainter * painter,NodeGeometry const & geom,NodeDataModel const * model){

    tool::Bench::start("NodePainter::drawResizeRect");

    if (model->resizable()){
        painter->setBrush(Qt::gray);
        painter->drawEllipse(geom.resize_rect());
    }

    tool::Bench::stop();
}


void NodePainter::drawValidationRect(QPainter * painter,NodeGeometry const & geom,NodeDataModel const * model,NodeGraphicsObject const & graphicsObject){

    tool::Bench::start("NodePainter::drawValidationRect");

    auto modelValidationState = model->validation_state();
    if (modelValidationState != NodeValidationState::Valid){

        NodeStyle const& nodeStyle = model->node_style();
        auto color = graphicsObject.isSelected() ? nodeStyle.SelectedBoundaryColor : nodeStyle.NormalBoundaryColor;
        if (geom.hovered()){
            QPen p(color, nodeStyle.HoveredPenWidth);
            painter->setPen(p);
        }else{
            QPen p(color, nodeStyle.PenWidth);
            painter->setPen(p);
        }

        //Drawing the validation message background
        if (modelValidationState == NodeValidationState::Error){
            painter->setBrush(nodeStyle.ErrorColor);
        }else{
            painter->setBrush(nodeStyle.WarningColor);
        }

        double const radius = 3.0;
        float diam = nodeStyle.ConnectionPointDiameter;

        QRectF boundary(
            -diam,
            -diam + geom.height() - geom.validationHeight(),
            2.0 * diam + geom.width(),
            2.0 * diam + geom.validationHeight()
        );
        painter->drawRoundedRect(boundary, radius, radius);

        painter->setBrush(Qt::gray);

        //Drawing the validation message itself
        QString const &errorMsg = model->validation_message();

        QFont f = painter->font();

        QFontMetrics metrics(f);

        auto rect = metrics.boundingRect(errorMsg);

        QPointF position((geom.width() - rect.width()) / 2.0,
        geom.height() - (geom.validationHeight() - diam) / 2.0);

        painter->setFont(f);
        painter->setPen(nodeStyle.FontColor);
        painter->drawText(position, errorMsg);
    }
    tool::Bench::stop();
}
