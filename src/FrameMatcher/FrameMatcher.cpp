#include "FrameMatcher.h"

using namespace std;

FrameMatcher::~FrameMatcher(){}
Transformation * FrameMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst){

	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = Eigen::Matrix4f::Identity();
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 0;
	return transformation;
}
void FrameMatcher::print(){printf("%s\n",name.c_str());}
void FrameMatcher::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
