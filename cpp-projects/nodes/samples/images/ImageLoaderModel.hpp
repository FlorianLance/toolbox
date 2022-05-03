//#pragma once

//#include <iostream>

//#include <QtCore/QObject>
//#include <QtWidgets/QLabel>

//#include "nodes/DataModelRegistry.hpp"
//#include "nodes/NodeDataModel.hpp"

//#include "PixmapData.hpp"

//using QtNodes::PortType;
//using QtNodes::PortIndex;
//using QtNodes::NodeData;
//using QtNodes::NodeDataType;
//using QtNodes::NodeDataModel;
//using QtNodes::NodeValidationState;

///// The model dictates the number of inputs and outputs for the Node.
///// In this example it has no logic.
//class ImageLoaderModel : public NodeDataModel
//{
//  Q_OBJECT

//public:
//  ImageLoaderModel();

//  virtual
//  ~ImageLoaderModel() {}

//  static const inline QString m_caption = QStringLiteral("Image Source");
//  static const inline QString m_name = QStringLiteral("ImageLoaderModel");


//public:

//    const QString &caption() const override{return m_caption;}
//    const QString &name() const override{return m_name;}


//public:

//  virtual QString
//  modelName() const
//  { return QString("Source Image"); }

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
//  embeddedWidget() override { return _label; }

//  bool
//  resizable() const override { return true; }

//protected:

//  bool
//  eventFilter(QObject *object, QEvent *event) override;

//private:

//  QLabel * _label;

//  QPixmap _pixmap;
//};
