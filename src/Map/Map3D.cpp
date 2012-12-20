#include "Map3D.h"
#include "Frame_input.h"

using namespace std;

Map3D::Map3D(){}
Map3D::~Map3D(){}
void Map3D::addFrame(Frame_input * fi){printf("Map3D::addFrame(Frame_input * fi)\n");}
void Map3D::addFrame(RGBDFrame * frame){printf("Map3D::addFrame(RGBDFrame * frame)\n");}
void Map3D::addTransformation(Transformation * transformation){}
void Map3D::estimate(){}
void Map3D::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
void Map3D::visualize(){}

