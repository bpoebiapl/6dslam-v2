#ifndef OrbExtractor_H_
#define OrbExtractor_H_
//OpenCV
#include "FeatureExtractor.h"

class OrbExtractor: public FeatureExtractor
{
	public:
		int nr_features;
		KeyPointSet * getKeyPointSet(IplImage * rgb_img,IplImage * depth_img);
		OrbExtractor();
		~OrbExtractor();
};

#endif
