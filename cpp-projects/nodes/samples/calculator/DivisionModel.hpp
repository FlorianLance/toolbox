//#pragma once

//#include <QtCore/QObject>
//#include <QtWidgets/QLabel>

//#include "nodes/NodeDataModel.hpp"

//#include "MathOperationDataModel.hpp"

//#include "DecimalData.hpp"

///// The model dictates the number of inputs and outputs for the Node.
///// In this example it has no logic.
//class DivisionModel : public MathOperationDataModel
//{
//public:

//  virtual
//  ~DivisionModel() {}

//  static const inline QString dividend = QStringLiteral("Dividend");
//  static const inline QString divisor = QStringLiteral("Divisor");
//  static const inline QString result = QStringLiteral("Result");
//  static const inline QString empty = QStringLiteral("");

//  static const inline QString m_caption = QStringLiteral("Division");
//  static const inline QString m_name = QStringLiteral("Division");


//public:
//    const QString &caption() const override{return m_caption;}
//    const QString &name() const override{return m_name;}


//  bool
//  port_caption_visible(PortType portType, PortIndex portIndex) const override
//  {
//    Q_UNUSED(portType); Q_UNUSED(portIndex);
//    return true;
//  }

//  const QString &
//  port_caption(PortType portType, PortIndex portIndex) const override
//  {
//    switch (portType)
//    {
//      case PortType::In:
//        if (portIndex == 0)
//          return dividend;
//        else if (portIndex == 1)
//          return divisor;

//        break;

//      case PortType::Out:
//        return result;

//      default:
//        break;
//    }
//    return empty;
//  }


//private:

//  void
//  compute() override
//  {
//    PortIndex const outPortIndex = 0;

//    auto n1 = _number1.lock();
//    auto n2 = _number2.lock();

//    if (n2 && (n2->number() == 0.0))
//    {
//      modelValidationState = NodeValidationState::Error;
//      modelValidationError = QStringLiteral("Division by zero error");
//      _result.reset();
//    }
//    else if (n1 && n2)
//    {
//      modelValidationState = NodeValidationState::Valid;
//      modelValidationError = QString();
//      _result = std::make_shared<DecimalData>(n1->number() /
//                                              n2->number());
//    }
//    else
//    {
//      modelValidationState = NodeValidationState::Warning;
//      modelValidationError = QStringLiteral("Missing or incorrect inputs");
//      _result.reset();
//    }

//    Q_EMIT dataUpdated(outPortIndex);
//  }
//};
