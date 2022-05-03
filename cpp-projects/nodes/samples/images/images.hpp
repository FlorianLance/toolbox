
//#pragma once

//#include "nodes/NodeData.hpp"
//#include "nodes/FlowScene.hpp"
//#include "nodes/FlowView.hpp"

//#include <QtWidgets/QApplication>

//#include "ImageShowModel.hpp"
//#include "ImageLoaderModel.hpp"

//using QtNodes::DataModelRegistry;
//using QtNodes::FlowScene;
//using QtNodes::FlowView;

//static std::shared_ptr<DataModelRegistry>
//images_registerDataModels()
//{
//  auto ret = std::make_shared<DataModelRegistry>();
//  ret->registerModel<ImageShowModel>();

//  ret->registerModel<ImageLoaderModel>();

//  return ret;
//}


//int
//images(int argc, char *argv[])
//{
//  QApplication app(argc, argv);

//  FlowScene scene(images_registerDataModels());

//  FlowView view(&scene);

//  view.setWindowTitle("Node-based flow editor");
//  view.resize(800, 600);
//  view.show();

//  return app.exec();
//}
