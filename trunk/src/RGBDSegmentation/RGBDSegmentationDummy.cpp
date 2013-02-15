#include "RGBDSegmentationDummy.h"
#include "mygeometry/mygeometry.h"
using namespace std;

RGBDSegmentationDummy::RGBDSegmentationDummy(){}
RGBDSegmentationDummy::~RGBDSegmentationDummy(){}
vector<Plane * > * RGBDSegmentationDummy::segment(IplImage * rgb_img,IplImage * depth_img){return new vector<Plane * >();}
