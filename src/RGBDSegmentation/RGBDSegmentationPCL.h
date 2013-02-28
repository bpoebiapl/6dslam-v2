#ifndef RGBDSegmentationPCL_H_
#define RGBDSegmentationPCL_H_
#include "RGBDSegmentation.h"

using namespace std;

class RGBDSegmentationPCL: public RGBDSegmentation
{
	public:
		
		RGBDSegmentationPCL();
		~RGBDSegmentationPCL();
		vector<Plane * > * segment(IplImage * rgb_img,IplImage * depth_img);
};

#endif
