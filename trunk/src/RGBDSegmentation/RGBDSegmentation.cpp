#include "RGBDSegmentation.h"

using namespace std;

RGBDSegmentation::~RGBDSegmentation(){}
vector<Plane * > * RGBDSegmentation::segment(IplImage * rgb_img,IplImage * depth_img){return 0;}
void RGBDSegmentation::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
