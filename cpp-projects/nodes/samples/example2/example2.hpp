
//#pragma once

//#include "nodes/NodeData.hpp"
//#include "nodes/FlowScene.hpp"
//#include "nodes/FlowView.hpp"

//#include <QtWidgets/QApplication>

//#include "nodes/DataModelRegistry.hpp"

//#include "TextSourceDataModel.hpp"
//#include "TextDisplayDataModel.hpp"

//using QtNodes::DataModelRegistry;
//using QtNodes::FlowView;
//using QtNodes::FlowScene;

//static std::shared_ptr<DataModelRegistry>
//example2_registerDataModels()
//{
//  auto ret = std::make_shared<DataModelRegistry>();

//  ret->registerModel<TextSourceDataModel>();

//  ret->registerModel<TextDisplayDataModel>();

//  return ret;
//}


//int
//example2(int argc, char *argv[])
//{
//  QApplication app(argc, argv);

//  FlowScene scene(example2_registerDataModels());

//  FlowView view(&scene);

//  view.setWindowTitle("Node-based flow editor");
//  view.resize(800, 600);
//  view.show();

//  return app.exec();
//}
