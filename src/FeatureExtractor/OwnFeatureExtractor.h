#ifndef OwnFeatureExtractor_H_
#define OwnFeatureExtractor_H_
//OpenCV
#include "FeatureExtractor.h"

class OwnFeatureExtractor: public FeatureExtractor
{
	public:
		int max_kps;
		int histogram_bins;
		float max_dist;
		float ** currentDistances;
		KeyPointSet * getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud);
		KeyPointSet * getKeyPointSet(IplImage * rgb_img,IplImage * depth_img);
		OwnFeatureExtractor();
		~OwnFeatureExtractor();
};

#endif
