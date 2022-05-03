#pragma once

// Qt
#include <QtWidgets/QWidget>

// local
#include "PortType.hpp"
#include "NodeData.hpp"
#include "NodeGeometry.hpp"
#include "NodeStyle.hpp"
#include "NodePainterDelegate.hpp"

namespace QtNodes{

enum class NodeValidationState{
    Valid,
    Warning,
    Error
};

struct PortInfo{
    bool captionVisibility;
    QString caption;
    QtNodes::NodeDataType type;
};

class Connection;

class StyleCollection;

class NODE_EDITOR_PUBLIC NodeDataModel: public QObject{
  Q_OBJECT

public:

    enum class ConnectionPolicy{
        One,
        Many,
    };

    NodeDataModel();
    virtual ~NodeDataModel() = default;

    const QString &name() const noexcept {return m_name;}

    const QString &caption() const noexcept { return m_caption;}
    constexpr bool caption_visible() const noexcept { return m_captionVisibility; }

    const QString &port_caption(QtNodes::PortType t, QtNodes::PortIndex i ) const;
    const QtNodes::NodeDataType &port_data_type(QtNodes::PortType portType, QtNodes::PortIndex portIndex) const;
    bool port_caption_visible(QtNodes::PortType portType, QtNodes::PortIndex index) const;
    inline unsigned int nb_Ports(PortType portType) const noexcept{ return nbPorts[static_cast<int>(portType)];}
    constexpr ConnectionPolicy port_out_connection_policy(PortIndex) const noexcept { return portOutConnectionPolicy; }

    const NodeStyle &node_style() const;
    void set_node_style(const NodeStyle &style);

    /// Triggers the algorithm
    virtual void setInData(std::shared_ptr<NodeData> nodeData,PortIndex port) = 0;
    virtual std::shared_ptr<NodeData> outData(PortIndex port) = 0;

    virtual QWidget * embeddedWidget() = 0;

    virtual bool resizable() const { return false; }

    NodeValidationState validation_state() const { return modelValidationState; }
    const QString &validation_message() const { return modelValidationMessage; }

    virtual NodePainterDelegate* painterDelegate() const { return nullptr; }

protected:

    void set_valid_state();
    void set_invalid_state(QString warning);
    void set_error_state(QString error);

public slots:

    virtual void input_connection_created(const Connection &c){Q_UNUSED(c)}
    virtual void input_connection_deleted(const Connection &c){Q_UNUSED(c)}
    virtual void output_connection_created(const Connection &c){Q_UNUSED(c)}
    virtual void output_connection_deleted(const Connection &c){Q_UNUSED(c)}
    virtual void wheel_event(QWheelEvent *event){Q_UNUSED(event)}

signals:

    void dataUpdated(PortIndex index);
    void dataInvalidated(PortIndex index);
    void computingStarted();
    void computingFinished();
    void embeddedWidgetSizeUpdated();

    void input_connection_validity_signal(bool state);
    void force_node_update_signal();


protected:

    static const inline bool defaultPortCaptionVisibility = false;
    static const inline QString defaultPortCaption = QStringLiteral("default");

    QString m_caption;
    QString m_name;

    QtNodes::NodeValidationState modelValidationState = QtNodes::NodeValidationState::Valid;
    QString modelValidationMessage;

    bool m_captionVisibility = true;

    ConnectionPolicy portOutConnectionPolicy = ConnectionPolicy::Many;

    std::vector<unsigned int> nbPorts = {0,0,0};
    std::vector<PortInfo> inPortsInfo;
    std::vector<PortInfo> outPortsInfo;

private:
    NodeStyle _nodeStyle;
};
}
