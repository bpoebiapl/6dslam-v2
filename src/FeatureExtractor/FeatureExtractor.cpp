#include "FeatureExtractor.h"

using namespace std;

FeatureExtractor::FeatureExtractor(){}

FeatureExtractor::~FeatureExtractor(){}

KeyPointSet * FeatureExtractor::getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud){
	printf("Extracting unknown feature\n");
	return NULL;
}

KeyPointSet * FeatureExtractor::getKeyPointSet(IplImage * rgb_img,IplImage * depth_img){
	printf("Extracting unknown feature\n");
	return NULL;
}
