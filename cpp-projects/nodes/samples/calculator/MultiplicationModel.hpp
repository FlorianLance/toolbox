//#pragma once

//#include <QtCore/QObject>
//#include <QtWidgets/QLabel>

//#include "nodes/NodeDataModel.hpp"

//#include "MathOperationDataModel.hpp"

//#include "DecimalData.hpp"

///// The model dictates the number of inputs and outputs for the Node.
///// In this example it has no logic.
//class MultiplicationModel : public MathOperationDataModel
//{
//public:

//  virtual
//  ~MultiplicationModel() {}

//  static const inline QString m_caption = QStringLiteral("Multiplication");
//  static const inline QString m_name = QStringLiteral("Multiplication");


//public:

//    const QString &caption() const override{return m_caption;}
//    const QString &name() const override{return m_name;}



//private:

//  void
//  compute() override
//  {
//    PortIndex const outPortIndex = 0;

//    auto n1 = _number1.lock();
//    auto n2 = _number2.lock();

//    if (n1 && n2)
//    {
//      modelValidationState = NodeValidationState::Valid;
//      modelValidationError = QString();
//      _result = std::make_shared<DecimalData>(n1->number() *
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
