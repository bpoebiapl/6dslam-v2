#ifndef FPFHExtractor_H_
#define FPFHExtractor_H_
//OpenCV
#include "FeatureExtractor.h"

class FPFHExtractor: public FeatureExtractor
{
	public:
		float feature_radius;
		float voxel_grid_size;
		KeyPointSet * getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud);
		KeyPointSet * getKeyPointSet(IplImage * rgb_img,IplImage * depth_img);
		FPFHExtractor();
		~FPFHExtractor();
};

#endif
