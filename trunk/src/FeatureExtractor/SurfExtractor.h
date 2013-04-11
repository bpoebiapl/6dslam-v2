#ifndef SurfExtractor_H_
#define SurfExtractor_H_
//OpenCV
#include "FeatureExtractor.h"

class SurfExtractor: public FeatureExtractor
{
	public:
		double total_time;
		int total_frames;
		int total_keypoints;
		int nr_features;
		bool upright;		// run in rotation invariant mode? 
		int octaves;		// number of octaves to calculate
		int intervals;		// number of intervals per octave
		int init_sample;	// initial sampling step
		float thres;		// blob response threshold
		KeyPointSet * getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud);
		KeyPointSet * getKeyPointSet(IplImage * rgb_img,IplImage * depth_img);
		SurfExtractor();
		~SurfExtractor();
};

#endif
