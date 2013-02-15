#ifndef RGBDSegmentationDummy_H_
#define RGBDSegmentationDummy_H_
#include "RGBDSegmentation.h"

using namespace std;

class RGBDSegmentationDummy: public RGBDSegmentation
{
	public:
		
		RGBDSegmentationDummy();
		~RGBDSegmentationDummy();
		vector<Plane * > * segment(IplImage * rgb_img,IplImage * depth_img);
};

#endif
