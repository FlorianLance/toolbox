#include "NodeGraphicsObject.hpp"

// std
#include <iostream>
#include <cstdlib>

// Qt
#include <QtWidgets/QtWidgets>
#include <QtWidgets/QGraphicsEffect>

// base
#include "utility/benchmark.hpp"

// local
#include "ConnectionGraphicsObject.hpp"
#include "ConnectionState.hpp"
#include "FlowScene.hpp"
#include "NodePainter.hpp"
#include "Node.hpp"
#include "NodeDataModel.hpp"
#include "NodeConnectionInteraction.hpp"
#include "StyleCollection.hpp"

using QtNodes::NodeGraphicsObject;
using QtNodes::Node;
using QtNodes::FlowScene;

NodeGraphicsObject::NodeGraphicsObject(FlowScene &scene, Node& node) :
    _scene(scene),
    _node(node),
    _locked(false),
    _proxyWidget(nullptr){

    _scene.addItem(this);

    setFlag(QGraphicsItem::ItemDoesntPropagateOpacityToChildren, true);
    setFlag(QGraphicsItem::ItemIsMovable, true);
    setFlag(QGraphicsItem::ItemIsFocusable, true);
    setFlag(QGraphicsItem::ItemIsSelectable, true);
    setFlag(QGraphicsItem::ItemSendsScenePositionChanges, true);

    setCacheMode( QGraphicsItem::DeviceCoordinateCache );

    auto const &nodeStyle = node.nodeDataModel()->node_style();

    {
        auto effect = new QGraphicsDropShadowEffect;
        effect->setOffset(4, 4);
        effect->setBlurRadius(20);
        effect->setColor(nodeStyle.ShadowColor);
        setGraphicsEffect(effect);
    }

    setOpacity(nodeStyle.Opacity);

    setAcceptHoverEvents(true);

    setZValue(0);

    embedQWidget();

    // connect to the move signals to emit the move signals in FlowScene
    auto onMoveSlot = [this] {
        _scene.nodeMoved(_node, pos());
    };
    connect(this, &QGraphicsObject::xChanged, this, onMoveSlot);
    connect(this, &QGraphicsObject::yChanged, this, onMoveSlot);
}

NodeGraphicsObject::~NodeGraphicsObject(){
    _scene.removeItem(this);
}

Node& NodeGraphicsObject::node(){
    return _node;
}

Node const& NodeGraphicsObject::node() const{
    return _node;
}

void NodeGraphicsObject::embedQWidget(){

    NodeGeometry & geom = _node.nodeGeometry();
    if (auto w = _node.nodeDataModel()->embeddedWidget()){

        _proxyWidget = new QGraphicsProxyWidget(this);
        _proxyWidget->setWidget(w);
        _proxyWidget->setPreferredWidth(5);
        geom.recalculate_size();

        if (w->sizePolicy().verticalPolicy() & QSizePolicy::ExpandFlag){
            // If the widget wants to use as much vertical space as possible, set it to have the geom's equivalentWidgetHeight.
            _proxyWidget->setMinimumHeight(geom.equivalent_widget_height());
        }

        _proxyWidget->setPos(geom.widget_position());

        update();

        _proxyWidget->setOpacity(1.0);
        _proxyWidget->setFlag(QGraphicsItem::ItemIgnoresParentOpacity);
    }
}

QRectF NodeGraphicsObject::boundingRect() const{
    return _node.nodeGeometry().boundingRect();
}

void NodeGraphicsObject::setGeometryChanged(){
    prepareGeometryChange();
}

void NodeGraphicsObject::moveConnections() const{

    NodeState const & nodeState = _node.nodeState();
    for (PortType portType: {PortType::In, PortType::Out}){
        auto const & connectionEntries = nodeState.getEntries(portType);
        for (auto const & connections : connectionEntries){
            for (auto & con : connections){
                con.second->getConnectionGraphicsObject().move();
            }
        }
    }
}


void NodeGraphicsObject::lock(bool locked){
    _locked = locked;
    setFlag(QGraphicsItem::ItemIsMovable, !locked);
    setFlag(QGraphicsItem::ItemIsFocusable, !locked);
    setFlag(QGraphicsItem::ItemIsSelectable, !locked);
}

void NodeGraphicsObject::paint(QPainter * painter, const QStyleOptionGraphicsItem *option, QWidget*){

    tool::Bench::start("NodeGraphicsObject::paint");
    painter->setClipRect(option->exposedRect);

//    auto font = painter->font();
//    QFontMetrics fontMetrics(font);
//    QFont boldFont = font;
//    boldFont.setBold(true);
//    QFontMetrics boldFontMetrics(boldFont);

    NodePainter::paint(painter, _node, _scene);
    tool::Bench::stop();
}

QVariant NodeGraphicsObject::itemChange(GraphicsItemChange change, const QVariant &value){

    if (change == ItemPositionChange && scene()){
        moveConnections();
    }

    return QGraphicsItem::itemChange(change, value);
}


void NodeGraphicsObject::mousePressEvent(QGraphicsSceneMouseEvent * event){

    if (_locked){
        return;
    }

    // deselect all other items after this one is selected
    if (!isSelected() && !(event->modifiers() & Qt::ControlModifier)){
        _scene.clearSelection();
    }

    for (PortType portToCheck: {PortType::In, PortType::Out}){

        NodeGeometry const & nodeGeometry = _node.nodeGeometry();

        // TODO do not pass sceneTransform
        int const portIndex = nodeGeometry.check_hit_scene_point(portToCheck,
        event->scenePos(),
        sceneTransform());

        if (portIndex != INVALID){

            NodeState const & nodeState = _node.nodeState();

            std::unordered_map<QUuid, Connection*> connections =
            nodeState.connections(portToCheck, portIndex);

            // start dragging existing connection
            if (!connections.empty() && portToCheck == PortType::In){
                auto con = connections.begin()->second;

                NodeConnectionInteraction interaction(_node, *con, _scene);

                interaction.disconnect(portToCheck);
            }else {// initialize new Connection

                if (portToCheck == PortType::Out){
                    auto const outPolicy = _node.nodeDataModel()->port_out_connection_policy(portIndex);
                    if (!connections.empty() && outPolicy == NodeDataModel::ConnectionPolicy::One){
                        _scene.deleteConnection( *connections.begin()->second );
                    }
                }

                // todo add to FlowScene
                auto connection = _scene.createConnection(portToCheck,
                _node,
                portIndex);

                _node.nodeState().setConnection(portToCheck,
                portIndex,
                *connection);

                connection->getConnectionGraphicsObject().grabMouse();
            }
        }
    }

    auto pos     = event->pos();
    auto & geom  = _node.nodeGeometry();
    auto & state = _node.nodeState();

    if (_node.nodeDataModel()->resizable() && geom.resize_rect().contains(QPoint(pos.x(), pos.y()))){
        state.setResizing(true);
    }
}


void NodeGraphicsObject::mouseMoveEvent(QGraphicsSceneMouseEvent * event){

    tool::Bench::start("NodeGraphicsObject::itemChange");

    auto & geom  = _node.nodeGeometry();
    auto & state = _node.nodeState();
    if (state.resizing()){

        auto diff = event->pos() - event->lastPos();
        if (auto w = _node.nodeDataModel()->embeddedWidget()){
            prepareGeometryChange();

            auto oldSize = w->size();

            oldSize += QSize(diff.x(), diff.y());

            w->setFixedSize(oldSize);

            _proxyWidget->setMinimumSize(oldSize);
            _proxyWidget->setMaximumSize(oldSize);
            _proxyWidget->setPos(geom.widget_position());

            geom.recalculate_size();
            update();

            moveConnections();

            event->accept();
        }
    }else{
        QGraphicsObject::mouseMoveEvent(event);

        if (event->lastPos() != event->pos())
        moveConnections();

        event->ignore();
    }

    QRectF r = scene()->sceneRect();
    r = r.united(mapToScene(boundingRect()).boundingRect());
    scene()->setSceneRect(r);
    tool::Bench::stop();
}

void NodeGraphicsObject::mouseReleaseEvent(QGraphicsSceneMouseEvent* event){
    auto & state = _node.nodeState();

    state.setResizing(false);

    QGraphicsObject::mouseReleaseEvent(event);

    // position connections precisely after fast node move
    moveConnections();
}

void NodeGraphicsObject::hoverEnterEvent(QGraphicsSceneHoverEvent * event){

    tool::Bench::start("NodeGraphicsObject::hoverEnterEvent");

    // bring all the colliding nodes to background
    QList<QGraphicsItem *> overlapItems = collidingItems();

    for (QGraphicsItem *item : overlapItems){
        if (item->zValue() > 0.0){
            item->setZValue(0.0);
        }
    }

    // bring this node forward
    setZValue(1.0);

    _node.nodeGeometry().set_hovered(true);
    update();
    _scene.nodeHovered(node(), event->screenPos());
    event->accept();
    tool::Bench::stop();
}


void NodeGraphicsObject::hoverLeaveEvent(QGraphicsSceneHoverEvent * event){
    _node.nodeGeometry().set_hovered(false);
    update();
    _scene.nodeHoverLeft(node());
    event->accept();
}

void NodeGraphicsObject::hoverMoveEvent(QGraphicsSceneHoverEvent * event){

    auto pos    = event->pos();
    auto & geom = _node.nodeGeometry();
    if (_node.nodeDataModel()->resizable() && geom.resize_rect().contains(QPoint(pos.x(), pos.y()))){
        setCursor(QCursor(Qt::SizeFDiagCursor));
    }else{
        setCursor(QCursor());
    }

    event->accept();
}

void NodeGraphicsObject::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event){
    QGraphicsItem::mouseDoubleClickEvent(event);
    _scene.nodeDoubleClicked(node());
}

void NodeGraphicsObject::contextMenuEvent(QGraphicsSceneContextMenuEvent* event){
    _scene.nodeContextMenu(node(), mapToScene(event->pos()));
}
