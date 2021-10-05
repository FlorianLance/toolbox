#pragma once

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>

#include "nodes/NodeDataModel.hpp"

#include <iostream>

using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::NodeValidationState;

class IntegerData;

class ModuloModel
  : public NodeDataModel
{
  Q_OBJECT

public:
  ModuloModel() = default;

  virtual
  ~ModuloModel() = default;

  static const inline QString dividend = QStringLiteral("Dividend");
  static const inline QString divisor = QStringLiteral("Divisor");
  static const inline QString result = QStringLiteral("Result");
  static const inline QString empty = QStringLiteral("");

  static const inline QString m_caption = QStringLiteral("Modulo");
  static const inline QString m_name = QStringLiteral("Modulo");

public:

    const QString &caption() const override{return m_caption;}
    const QString &name() const override{return m_name;}


  bool
  captionVisible() const override
  { return true; }

  bool
  portCaptionVisible(PortType, PortIndex ) const override
  { return true; }

  const QString &
  portCaption(PortType portType, PortIndex portIndex) const override
  {
    switch (portType)
    {
      case PortType::In:
        if (portIndex == 0)
          return dividend;
        else if (portIndex == 1)
          return divisor;

        break;

      case PortType::Out:
        return result;

      default:
        break;
    }
    return empty;
  }


public:

  QJsonObject
  save() const override;

public:

  unsigned int
  nPorts(PortType portType) const override;

  const NodeDataType&
  dataType(PortType portType, PortIndex portIndex) const override;

  std::shared_ptr<NodeData>
  outData(PortIndex port) override;

  void
  setInData(std::shared_ptr<NodeData>, int) override;

  QWidget *
  embeddedWidget() override { return nullptr; }

  NodeValidationState
  validationState() const override;

  QString
  validationMessage() const override;

private:

  std::weak_ptr<IntegerData> _number1;
  std::weak_ptr<IntegerData> _number2;

  std::shared_ptr<IntegerData> _result;

  NodeValidationState modelValidationState = NodeValidationState::Warning;
  QString modelValidationError = QStringLiteral("Missing or incorrect inputs");
};
