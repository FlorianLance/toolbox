#pragma once

// Qt
#include <QtCore/QRectF>
#include <QtCore/QPointF>
#include <QtGui/QTransform>
#include <QtGui/QFontMetrics>

// local
#include "PortType.hpp"
#include "Export.hpp"
#include "memory.hpp"

namespace QtNodes{

class NodeState;
class NodeDataModel;
class Node;

class NODE_EDITOR_PUBLIC NodeGeometry{
public:

    NodeGeometry(const std::unique_ptr<NodeDataModel> &dataModel);

public:

    constexpr unsigned int height() const noexcept{ return _height; }
    constexpr void set_height(unsigned int h) noexcept{ _height = h; }

    constexpr unsigned int width() const noexcept{ return _width; }
    constexpr void set_width(unsigned int w) noexcept{ _width = w; }

    constexpr unsigned int entry_height() const noexcept { return _entryHeight; }
    constexpr void set_entry_height(unsigned int h) noexcept{
        _entryHeight = h;
        update_step();
    }

    constexpr unsigned int entry_width() const noexcept { return _entryWidth;}
    constexpr void set_entry_width(unsigned int w) noexcept {_entryWidth = w;}

    constexpr unsigned int spacing() const  noexcept{ return _spacing; }
    constexpr void set_spacing(unsigned int s) noexcept {
        _spacing = s;
        update_step();
    }

    constexpr bool hovered() const noexcept { return _hovered; }
    constexpr void set_hovered(unsigned int h) noexcept{ _hovered = h; }

    unsigned int nb_sources() const;
    unsigned int nb_sinks() const;

    QPointF const& draggingPos() const { return _draggingPos; }
    void setDraggingPosition(QPointF const& pos) { _draggingPos = pos; }

public:

    QRectF entryBoundingRect() const;
    QRectF boundingRect() const;

    /// Updates size unconditionally
    void recalculate_size() const;

    /// Updates size if the QFontMetrics is changed
    void recalculate_size(const QFont &font) const;

    QPointF port_scene_position(PortIndex index, PortType portType, const QTransform &t = defaultTr) const;
    PortIndex check_hit_scene_point(PortType portType, QPointF point, const QTransform &t = defaultTr) const;
    QRect resize_rect() const;

    /// Returns the position of a widget on the Node surface
    QPointF widget_position() const;

    /// Returns the maximum height a widget can be without causing the node to grow.
    int equivalent_widget_height() const;
    unsigned int validationHeight() const;
    unsigned int validation_width() const;

    static QPointF calculateNodePositionBetweenNodePorts(
        PortIndex targetPortIndex, PortType targetPort, Node* targetNode,
        PortIndex sourcePortIndex, PortType sourcePort, Node* sourceNode,
        Node& newNode
    );

private:

    constexpr void update_step() noexcept{
        _step     = _entryHeight + _spacing;
        _halfStep = _step/2;
    }

    unsigned int caption_height() const;
    unsigned int caption_width() const;
    unsigned int port_width(PortType portType) const;

private:

    // some variables are mutable because
    // we need to change drawing metrics
    // corresponding to fontMetrics
    // but this doesn't change constness of Node
    mutable unsigned int _width;
    mutable unsigned int _height;
    unsigned int _entryWidth;
    mutable unsigned int _inputPortWidth;
    mutable unsigned int _outputPortWidth;
    mutable unsigned int _entryHeight;
    unsigned int _spacing;
    unsigned int _step;
    unsigned int _halfStep;

    bool _hovered;

    unsigned int _nSources;
    unsigned int _nSinks;

    QPointF _draggingPos;

    const std::unique_ptr<NodeDataModel> &_dataModel;

    mutable QFontMetrics _entryFontMetrics;
    mutable QFontMetrics _captionFontMetrics;
    mutable QFontMetrics _validationFontMetrics;

    static constexpr unsigned int rectSize = 7;
    static inline QTransform defaultTr = QTransform();
};
}
