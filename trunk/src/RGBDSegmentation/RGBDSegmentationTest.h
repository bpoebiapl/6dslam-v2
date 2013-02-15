#ifndef RGBDSegmentationTest_H_
#define RGBDSegmentationTest_H_
#include "RGBDSegmentation.h"

using namespace std;

class RGBDSegmentationTest: public RGBDSegmentation
{
	public:
		
		RGBDSegmentationTest();
		~RGBDSegmentationTest();
		vector<Plane * > * segment(IplImage * rgb_img,IplImage * depth_img);
};

#endif
