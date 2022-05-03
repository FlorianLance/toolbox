
//#pragma once


//#include <QtWidgets/QApplication>

//#include "nodes/NodeData.hpp"
//#include "nodes/FlowScene.hpp"
//#include "nodes/FlowView.hpp"
//#include "nodes/DataModelRegistry.hpp"
//#include "nodes/ConnectionStyle.hpp"

//#include "connections_colors_models.hpp"

//using QtNodes::DataModelRegistry;
//using QtNodes::FlowScene;
//using QtNodes::FlowView;
//using QtNodes::ConnectionStyle;

//static std::shared_ptr<DataModelRegistry>
//connection_colors_registerDataModels()
//{
//  auto ret = std::make_shared<DataModelRegistry>();

//  ret->registerModel<NaiveDataModel>();

//  /*
//     We could have more models registered.
//     All of them become items in the context meny of the scene.

//     ret->registerModel<AnotherDataModel>();
//     ret->registerModel<OneMoreDataModel>();

//   */

//  return ret;
//}


//static
//void
//connection_colors_setStyle()
//{
//  ConnectionStyle::setConnectionStyle(
//    R"(
//  {
//    "ConnectionStyle": {
//      "UseDataDefinedColors": true
//    }
//  }
//  )");
//}


////------------------------------------------------------------------------------

//int
//connection_colors(int argc, char* argv[])
//{
//  QApplication app(argc, argv);

//  connection_colors_setStyle();

//  FlowScene scene(connection_colors_registerDataModels());

//  FlowView view(&scene);

//  view.setWindowTitle("Node-based flow editor");
//  view.resize(800, 600);
//  view.show();

//  return app.exec();
//}
