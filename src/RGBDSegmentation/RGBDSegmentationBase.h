#ifndef RGBDSegmentationBase_H_
#define RGBDSegmentationBase_H_
#include "RGBDSegmentation.h"

using namespace std;

class RGBDSegmentationBase: public RGBDSegmentation
{
	public:
		
		RGBDSegmentationBase();
		~RGBDSegmentationBase();
		vector<Plane * > * segment(IplImage * rgb_img,IplImage * depth_img);
};

#endif
