
//#pragma once

//#include <QtWidgets/QApplication>

//#include "nodes/NodeData.hpp"
//#include "nodes/FlowScene.hpp"
//#include "nodes/FlowView.hpp"
//#include "nodes/DataModelRegistry.hpp"
//#include <nodes/NodeStyle>
//#include <nodes/FlowViewStyle>
//#include "nodes/ConnectionStyle.hpp"

//#include "styles_models.hpp"

//using QtNodes::DataModelRegistry;
//using QtNodes::FlowScene;
//using QtNodes::FlowView;
//using QtNodes::FlowViewStyle;
//using QtNodes::NodeStyle;
//using QtNodes::ConnectionStyle;

//static std::shared_ptr<DataModelRegistry>
//styles_registerDataModels()
//{
//  auto ret = std::make_shared<DataModelRegistry>();

//  ret->registerModel<StyleMyDataModel>();

//  return ret;
//}


//static
//void
//styles_setStyle()
//{
//  FlowViewStyle::setStyle(
//  R"(
//  {
//    "FlowViewStyle": {
//      "BackgroundColor": [255, 255, 240],
//      "FineGridColor": [245, 245, 230],
//      "CoarseGridColor": [235, 235, 220]
//    }
//  }
//  )");

//  NodeStyle::setNodeStyle(
//  R"(
//  {
//    "NodeStyle": {
//      "NormalBoundaryColor": "darkgray",
//      "SelectedBoundaryColor": "deepskyblue",
//      "GradientColor0": "mintcream",
//      "GradientColor1": "mintcream",
//      "GradientColor2": "mintcream",
//      "GradientColor3": "mintcream",
//      "ShadowColor": [200, 200, 200],
//      "FontColor": [10, 10, 10],
//      "FontColorFaded": [100, 100, 100],
//      "ConnectionPointColor": "white",
//      "PenWidth": 2.0,
//      "HoveredPenWidth": 2.5,
//      "ConnectionPointDiameter": 10.0,
//      "Opacity": 1.0
//    }
//  }
//  )");

//  ConnectionStyle::setConnectionStyle(
//  R"(
//  {
//    "ConnectionStyle": {
//      "ConstructionColor": "gray",
//      "NormalColor": "black",
//      "SelectedColor": "gray",
//      "SelectedHaloColor": "deepskyblue",
//      "HoveredColor": "deepskyblue",

//      "LineWidth": 3.0,
//      "ConstructionLineWidth": 2.0,
//      "PointDiameter": 10.0,

//      "UseDataDefinedColors": false
//    }
//  }
//  )");
//}


////------------------------------------------------------------------------------

//int
//styles(int argc, char* argv[])
//{
//  QApplication app(argc, argv);

//  styles_setStyle();

//  FlowScene scene(styles_registerDataModels());

//  FlowView view(&scene);

//  view.setWindowTitle("Style example");
//  view.resize(800, 600);
//  view.show();

//  return app.exec();
//}
