# GIS_ROS

In Rounting_Widget.h:
publicL
osgEarth::MapNode* m_mapNode = nullptr;
 
Get the mapNode from constructor of Routing_Widget.
 
Routing_Widget where it gets the mapNode:
In mainwindow.cpp
 
Line 1809:
m_routingWidget = new Routing_Widget(m_mapView, ThreeDMapScene::getInstance()->getSceneManager()->getMapNode());
