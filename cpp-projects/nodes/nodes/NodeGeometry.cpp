#include "NodeGeometry.hpp"

// std
#include <iostream>
#include <cmath>

// local
#include "PortType.hpp"
#include "NodeState.hpp"
#include "NodeDataModel.hpp"
#include "Node.hpp"
#include "NodeGraphicsObject.hpp"
#include "StyleCollection.hpp"

// base
#include "utility/benchmark.hpp"
#include "utility/logger.hpp"
#include "utility/format.hpp"

using QtNodes::NodeGeometry;
using QtNodes::NodeDataModel;
using QtNodes::PortIndex;
using QtNodes::PortType;
using QtNodes::Node;

NodeGeometry::NodeGeometry(std::unique_ptr<NodeDataModel> const &dataModel)
    : _width(100)
    , _height(150)
    , _inputPortWidth(70)
    , _outputPortWidth(70)
    , _entryHeight(20)
    , _spacing(20)
    , _hovered(false)
    , _nSources(dataModel->nb_Ports(PortType::Out))
    , _nSinks(dataModel->nb_Ports(PortType::In))
    , _draggingPos(-1000, -1000)
    , _dataModel(dataModel)
    ,_entryFontMetrics(QFont())
    ,_captionFontMetrics(QFont())
    ,_validationFontMetrics(QFont()){

    QFont f;
    f.setBold(true);
    _validationFontMetrics = QFontMetrics(f);
    _captionFontMetrics = _validationFontMetrics;
}

unsigned int NodeGeometry::nb_sources() const{
    return _dataModel->nb_Ports(PortType::Out);
}

unsigned int NodeGeometry::nb_sinks() const{
    return _dataModel->nb_Ports(PortType::In);
}

QRectF NodeGeometry::entryBoundingRect() const{

    double const addon = 0.0;
    return QRectF(
        0 - addon,
        0 - addon,
        _entryWidth + 2 * addon,
        _entryHeight + 2 * addon
    );
}

QRectF NodeGeometry::boundingRect() const{
    auto const &nodeStyle = StyleCollection::nodeStyle();
    double addon = 4 * nodeStyle.ConnectionPointDiameter;
    return QRectF(
        0 - addon,
        0 - addon,
        _width + 2 * addon,
        _height + 2 * addon
    );
}

void NodeGeometry::recalculate_size() const{

    tool::Bench::start("NodeGeometry::recalculateSize2");

    // compute height
    _entryHeight = _entryFontMetrics.height();
    _height      = (_entryHeight + _spacing) * std::max(_nSinks, _nSources);
    if (auto w = _dataModel->embeddedWidget()){
        _height  = std::max(_height, static_cast<unsigned>(w->height()));
    }
    _height     += caption_height();

    // compute width
    _inputPortWidth  = port_width(PortType::In);
    _outputPortWidth = port_width(PortType::Out);
    _width           = _inputPortWidth + _outputPortWidth + 2 * _spacing;

    if (auto w = _dataModel->embeddedWidget()){
        _width += w->width();
    }
    _width = std::max(_width, caption_width() + 2 * _spacing);

    if (_dataModel->validation_state() != NodeValidationState::Valid){
        _width   = std::max(_width, validation_width()+ 2 * _spacing);
        _height += validationHeight() + _spacing;
    }

    tool::Bench::stop();
}

void NodeGeometry::recalculate_size(const QFont &font) const{

    tool::Bench::start("NodeGeometry::recalculateSize1");

    QFontMetrics fontMetrics(font);
    QFont boldFont = font;
    boldFont.setBold(true);
    QFontMetrics boldFontMetrics(boldFont);

    if (_captionFontMetrics != boldFontMetrics){
        _captionFontMetrics    = boldFontMetrics;
        _validationFontMetrics = boldFontMetrics;
        _entryFontMetrics      = fontMetrics;
        recalculate_size();
    }

    tool::Bench::stop();
}


QPointF NodeGeometry::port_scene_position(PortIndex index, PortType portType, const QTransform &t) const{

    tool::Bench::start("NodeGeometry::port_scene_position");

    const float cpd = StyleCollection::nodeStyle().ConnectionPointDiameter;
    const double totalHeight = caption_height() + (_step * index) + (_halfStep);
    QPointF res;
    if(portType == PortType::Out){
        res = t.map(QPointF(_width + cpd, totalHeight));
    }else if(portType == PortType::In){
        res = t.map(QPointF(-cpd, totalHeight));
    }else{
        res = t.map(QPointF());
    }
    tool::Bench::stop();
    return res;
}

PortIndex NodeGeometry::check_hit_scene_point(PortType portType,QPointF const scenePoint, const QTransform &sceneTransform) const{

    tool::Bench::start("NodeGeometry::checkHitScenePoint");

    auto const &nodeStyle = StyleCollection::nodeStyle();
    PortIndex result = INVALID;
    if (portType == PortType::None){
        tool::Bench::stop();
        return result;
    }

    const double tolerance     = 2.0 * nodeStyle.ConnectionPointDiameter;
    const unsigned int  nItems = _dataModel->nb_Ports(portType);

    for (unsigned int ii = 0; ii < nItems; ++ii){

        QPointF p = port_scene_position(ii, portType, sceneTransform) - scenePoint;
        auto distance = std::sqrt(QPointF::dotProduct(p, p));
        if (distance < tolerance){
            result = PortIndex(ii);
            break;
        }
    }

    tool::Bench::stop();
    return result;
}

QRect NodeGeometry::resize_rect() const{

    return QRect(
        _width  - rectSize,
        _height - rectSize,
        rectSize,
        rectSize
    );
}

QPointF NodeGeometry::widget_position() const{

    if (auto w = _dataModel->embeddedWidget()){
        if (w->sizePolicy().verticalPolicy() & static_cast<QSizePolicy::Policy>(QSizePolicy::ExpandFlag)){
            // If the widget wants to use as much vertical space as possible, place it immediately after the caption.
            return QPointF(_spacing + port_width(PortType::In), caption_height());
        }else{
            if (_dataModel->validation_state() != NodeValidationState::Valid){
                return QPointF(
                    _spacing + port_width(PortType::In),
                    (caption_height() + _height - validationHeight() - _spacing - w->height()) / 2.0);
            }

            return QPointF(
                _spacing + port_width(PortType::In),
                (caption_height() + _height - w->height()) / 2.0
            );
        }
    }
    return QPointF();
}

int NodeGeometry::equivalent_widget_height() const{

    if (_dataModel->validation_state() != NodeValidationState::Valid){
        return height() - caption_height() + validationHeight();
    }
    return height() - caption_height();
}

unsigned int NodeGeometry::caption_height() const{

    if (!_dataModel->caption_visible()){
        return 0;
    }
    return _captionFontMetrics.boundingRect(_dataModel->caption()).height();
}

unsigned int NodeGeometry::caption_width() const{

    if (!_dataModel->caption_visible()){
        return 0;
    }
    return _captionFontMetrics.boundingRect(_dataModel->caption()).width();
}

unsigned int NodeGeometry::validationHeight() const{
    return _validationFontMetrics.boundingRect(_dataModel->validation_message()).height();
}

unsigned int NodeGeometry::validation_width() const{
    return _validationFontMetrics.boundingRect(_dataModel->validation_message()).width();
}


QPointF NodeGeometry::calculateNodePositionBetweenNodePorts(
    PortIndex targetPortIndex, PortType targetPort, Node* targetNode,
    PortIndex sourcePortIndex, PortType sourcePort, Node* sourceNode,
    Node& newNode){

    //Calculating the nodes position in the scene. It'll be positioned half way between the two ports that it "connects".
    //The first line calculates the halfway point between the ports (node position + port position on the node for both nodes averaged).
    //The second line offsets this coordinate with the size of the new node, so that the new nodes center falls on the originally
    //calculated coordinate, instead of it's upper left corner.
    auto converterNodePos =
        (sourceNode->nodeGraphicsObject().pos() + sourceNode->nodeGeometry().port_scene_position(sourcePortIndex, sourcePort) +
        targetNode->nodeGraphicsObject().pos() + targetNode->nodeGeometry().port_scene_position(targetPortIndex, targetPort)) / 2.0f;

    converterNodePos.setX(converterNodePos.x() - newNode.nodeGeometry().width() / 2.0f);
    converterNodePos.setY(converterNodePos.y() - newNode.nodeGeometry().height() / 2.0f);

    return converterNodePos;
}

unsigned int NodeGeometry::port_width(PortType portType) const{

    unsigned width = 0;
    for (auto ii = 0ul; ii < _dataModel->nb_Ports(portType); ++ii){

        const QString &name = _dataModel->port_caption_visible(portType, ii) ?
            _dataModel->port_caption(portType, ii) :
            _dataModel->port_data_type(portType, ii).name;

//        width = std::max(unsigned(_fontMetrics.width(name)), width);
        width = std::max(static_cast<unsigned>(_entryFontMetrics.boundingRect(name).width()), width);
    }

    return width;
}
