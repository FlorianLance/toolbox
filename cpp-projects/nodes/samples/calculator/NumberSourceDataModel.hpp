//#pragma once

//#include <QtCore/QObject>
//#include <QtWidgets/QLineEdit>

//#include "nodes/NodeDataModel.hpp"

//#include <iostream>

//class DecimalData;

//using QtNodes::PortType;
//using QtNodes::PortIndex;
//using QtNodes::NodeData;
//using QtNodes::NodeDataType;
//using QtNodes::NodeDataModel;
//using QtNodes::NodeValidationState;

///// The model dictates the number of inputs and outputs for the Node.
///// In this example it has no logic.
//class NumberSourceDataModel
//  : public NodeDataModel
//{
//  Q_OBJECT

//public:
//  NumberSourceDataModel();

//  virtual
//  ~NumberSourceDataModel() {}

//  static const inline QString m_caption = QStringLiteral("Number Source");
//  static const inline QString m_name = QStringLiteral("NumberSource");

//public:

//  bool
//  caption_visible() const override
//  { return false; }

//  const QString &caption() const override{return m_caption;}
//  const QString &name() const override{return m_name;}

//public:

//  QJsonObject
//  save() const override;

//  void
//  restore(QJsonObject const &p) override;

//public:

//  unsigned int
//  nb_Ports(PortType portType) const override;

//  const NodeDataType&
//  dataType(PortType portType, PortIndex portIndex) const override;

//  std::shared_ptr<NodeData>
//  outData(PortIndex port) override;

//  void
//  setInData(std::shared_ptr<NodeData>, int) override
//  { }

//  QWidget *
//  embeddedWidget() override { return _lineEdit; }

//private Q_SLOTS:

//  void
//  onTextEdited(QString const &string);

//private:

//  std::shared_ptr<DecimalData> _number;

//  QLineEdit * _lineEdit;
//};
