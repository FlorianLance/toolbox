//#include "MathOperationDataModel.hpp"

//#include "DecimalData.hpp"

//MathOperationDataModel::MathOperationDataModel(){
//    nbPorts[static_cast<int>(PortType::In)]   = 2;
//    nbPorts[static_cast<int>(PortType::Out)]  = 1;


//    bool captionVisibility;
//    QString caption;
//    QtNodes::NodeDataType type;

//    inPortsInfo.emplace_back(
//        QtNodes::PortInfo{
//            true,
//        }
//    );
////    for(size_t ii = 0; ii < slotsF.size(); ++ii){
////        inPortsInfo[ii].caption = from_view(slotsF[ii].first.functionName)
////                                  % QSL(" (") % from_view(ConnectionNode::get_name(slotsF[ii].first.argType)) % QSL(")");

////        inPortsInfo[ii].type = generate_node_data_type(slotsF[ii].first.argType);
////        inPortsInfo[ii].captionVisibility = true;
////    }

////    outPortsInfo.resize(1);
////    for(size_t ii = 0; ii < signalsF.size(); ++ii){
////        outPortsInfo[ii].caption = from_view(signalsF[ii].first.functionName)
////                                   % QSL(" (") % from_view(ConnectionNode::get_name(signalsF[ii].first.argType)) % QSL(")");

////        outPortsInfo[ii].type = generate_node_data_type(signalsF[ii].first.argType);
////        outPortsInfo[ii].captionVisibility = true;
////    }
//}



//const NodeDataType &
//MathOperationDataModel::
//dataType(PortType, PortIndex) const
//{
//  return DecimalData().type();
//}


//std::shared_ptr<NodeData>
//MathOperationDataModel::
//outData(PortIndex)
//{
//  return std::static_pointer_cast<NodeData>(_result);
//}


//void
//MathOperationDataModel::
//setInData(std::shared_ptr<NodeData> data, PortIndex portIndex)
//{
//  auto numberData =
//    std::dynamic_pointer_cast<DecimalData>(data);

//  if (portIndex == 0)
//  {
//    _number1 = numberData;
//  }
//  else
//  {
//    _number2 = numberData;
//  }

//  compute();
//}


