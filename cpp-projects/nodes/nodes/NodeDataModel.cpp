#include "NodeDataModel.hpp"

// local
#include "StyleCollection.hpp"

using QtNodes::NodeDataModel;
using QtNodes::NodeStyle;

NodeDataModel::NodeDataModel() : _nodeStyle(StyleCollection::nodeStyle()){
  // Derived classes can initialize specific style here
}

NodeStyle const& NodeDataModel::node_style() const{
    return _nodeStyle;
}

void NodeDataModel::set_node_style(const NodeStyle &style){
    _nodeStyle = style;
}

const QString &NodeDataModel::port_caption(QtNodes::PortType t, QtNodes::PortIndex i) const{

    const auto id = static_cast<size_t>(i);
    if(t == QtNodes::PortType::In){
        return inPortsInfo[id].caption;
    }else if(t == QtNodes::PortType::Out){
        return outPortsInfo[id].caption;
    }
    return defaultPortCaption;
}

const QtNodes::NodeDataType &NodeDataModel::port_data_type(QtNodes::PortType portType, QtNodes::PortIndex portIndex) const {

    const auto id = static_cast<size_t>(portIndex);
    if(portType == QtNodes::PortType::In){
        return inPortsInfo[id].type;
    }else if(portType == QtNodes::PortType::Out){
        return outPortsInfo[id].type;
    }
    return NodeData::defaultType;
}

bool NodeDataModel::port_caption_visible(QtNodes::PortType portType, QtNodes::PortIndex index) const{

    if(portType == QtNodes::PortType::In){
        return inPortsInfo[index].captionVisibility;
    }else if(portType == QtNodes::PortType::Out){
        return outPortsInfo[index].captionVisibility;
    }
    return  false;
}

void NodeDataModel::set_valid_state(){
    modelValidationState   = QtNodes::NodeValidationState::Valid;
    emit input_connection_validity_signal(true);
}

void NodeDataModel::set_invalid_state(QString warning){
    modelValidationState   = QtNodes::NodeValidationState::Warning;
    modelValidationMessage = warning;
    emit input_connection_validity_signal(false);
}

void NodeDataModel::set_error_state(QString error){
    modelValidationState   = QtNodes::NodeValidationState::Error;
    modelValidationMessage = error;
    emit input_connection_validity_signal(false);
}

