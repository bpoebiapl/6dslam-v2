#ifndef RGBDSegmentationScaleSearch_H_
#define RGBDSegmentationScaleSearch_H_
#include "RGBDSegmentation.h"

using namespace std;

class RGBDSegmentationScaleSearch: public RGBDSegmentation
{
	public:
		
		RGBDSegmentationScaleSearch();
		~RGBDSegmentationScaleSearch();
		vector<Plane * > * segment(IplImage * rgb_img,IplImage * depth_img);
};

#endif
