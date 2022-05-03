//#pragma once

//#include <QtCore/QObject>
//#include <QtWidgets/QLabel>

//#include "nodes/NodeDataModel.hpp"

//#include <iostream>

//using QtNodes::PortType;
//using QtNodes::PortIndex;
//using QtNodes::NodeData;
//using QtNodes::NodeDataType;
//using QtNodes::NodeDataModel;
//using QtNodes::NodeValidationState;

///// The model dictates the number of inputs and outputs for the Node.
///// In this example it has no logic.
//class NumberDisplayDataModel : public NodeDataModel
//{
//  Q_OBJECT

//public:
//  NumberDisplayDataModel();

//  virtual
//  ~NumberDisplayDataModel() {}

//  static const inline QString m_caption = QStringLiteral("Result");
//  static const inline QString m_name = QStringLiteral("Result");


//public:

//    const QString &caption() const override{return m_caption;}
//    const QString &name() const override{return m_name;}


//  bool
//  caption_visible() const override
//  { return false; }


//public:

//  unsigned int
//  nb_Ports(PortType portType) const override;

//  const NodeDataType&
//  dataType(PortType portType,
//           PortIndex portIndex) const override;

//  std::shared_ptr<NodeData>
//  outData(PortIndex port) override;

//  void
//  setInData(std::shared_ptr<NodeData> data, int) override;

//  QWidget *
//  embeddedWidget() override { return _label; }

//  NodeValidationState
//  validation_state() const override;

//  QString
//  validationMessage() const override;

//private:

//  NodeValidationState modelValidationState = NodeValidationState::Warning;
//  QString modelValidationError = QStringLiteral("Missing or incorrect inputs");

//  QLabel * _label;
//};
