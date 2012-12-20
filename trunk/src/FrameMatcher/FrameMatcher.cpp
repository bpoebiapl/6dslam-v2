#include "FrameMatcher.h"

using namespace std;

FrameMatcher::~FrameMatcher(){}
Transformation * FrameMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst){return 0;}
void FrameMatcher::print(){printf("%s\n",name.c_str());}
void FrameMatcher::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
