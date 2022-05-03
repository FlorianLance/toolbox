//#pragma once

//#include <QtCore/QObject>
//#include <QtWidgets/QLineEdit>

//#include "TextData.hpp"

//#include "nodes/NodeDataModel.hpp"

//#include <iostream>

//using QtNodes::PortType;
//using QtNodes::PortIndex;
//using QtNodes::NodeData;
//using QtNodes::NodeDataModel;

///// The model dictates the number of inputs and outputs for the Node.
///// In this example it has no logic.
//class TextSourceDataModel : public NodeDataModel
//{
//  Q_OBJECT

//public:
//  TextSourceDataModel();

//  virtual
//  ~TextSourceDataModel() {}

//  static const inline QString m_caption = QStringLiteral("Text Source");
//  static const inline QString m_name = QStringLiteral("TextSourceDataModel");



//public:


//    const QString &caption() const override{return m_caption;}
//  bool
//  caption_visible() const override { return false; }

//  static const QString&
//  Name()
//  { return m_name; }

//  const QString&
//  name() const override
//  { return TextSourceDataModel::Name(); }

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

//  QLineEdit * _lineEdit;
//};
