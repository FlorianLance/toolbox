//#pragma once

//#include <QtCore/QObject>

//#include "nodes/NodeData.hpp"
//#include "nodes/NodeDataModel.hpp"

//#include <memory>

//using QtNodes::PortType;
//using QtNodes::PortIndex;
//using QtNodes::NodeData;
//using QtNodes::NodeDataType;
//using QtNodes::NodeDataModel;
//using QtNodes::NodeValidationState;

///// The class can potentially incapsulate any user data which
///// need to be transferred within the Node Editor graph
//class StyleMyNodeData : public NodeData
//{
//public:
//    const NodeDataType &type() const override
//    {
//        return m_type;
//    }
//  static const inline NodeDataType m_type = {"MyNodeData", "My Node Data"};
//};

////------------------------------------------------------------------------------

///// The model dictates the number of inputs and outputs for the Node.
///// In this example it has no logic.
//class StyleMyDataModel : public NodeDataModel
//{
//  Q_OBJECT

//public:

//  virtual
//  ~StyleMyDataModel() {}

//  static const inline QString m_caption = QStringLiteral("My Data Model");
//  static const inline QString m_name = QStringLiteral("MyDataModel");


//public:

//    const QString &caption() const override{return m_caption;}
//    const QString &name() const override{return m_name;}


//public:

//  QJsonObject
//  save() const override
//  {
//    QJsonObject modelJson;

//    modelJson["name"] = name();

//    return modelJson;
//  }

//public:

//  unsigned int
//  nb_Ports(PortType) const override
//  {
//    return 3;
//  }

//  const NodeDataType&
//  dataType(PortType, PortIndex) const override
//  {
//    return StyleMyNodeData().type();
//  }

//  std::shared_ptr<NodeData>
//  outData(PortIndex) override
//  {
//    return std::make_shared<StyleMyNodeData>();
//  }

//  void
//  setInData(std::shared_ptr<NodeData>, int) override
//  {
//    //
//  }

//  QWidget *
//  embeddedWidget() override { return nullptr; }
//};
