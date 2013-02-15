#include "FeatureDistanceFilter.h"

using namespace std;
FeatureDistanceFilter::FeatureDistanceFilter(){
	feature_threshold = 0.15f;
}
FeatureDistanceFilter::~FeatureDistanceFilter(){}
Transformation * FeatureDistanceFilter::filterTransformation(Transformation * input){
	//printf("FeatureDistanceFilter:start\n");
	
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = input->transformationMatrix;
	transformation->src = input->src;
	transformation->dst = input->dst;
	
	transformation->weight = 0;
	for(int i = 0; i < input->matches.size();i++){
		KeyPoint * src_kp = input->matches.at(i).first;
		KeyPoint * dst_kp = input->matches.at(i).second;
		if(src_kp->descriptor->distance(dst_kp->descriptor) < feature_threshold){	
			transformation->matches.push_back(make_pair(src_kp, dst_kp));
		}
	}
	//printf("FeatureDistanceFilter:end\n");
	return transformation;
}
void FeatureDistanceFilter::print(){printf("%s\n",name.c_str());}
void FeatureDistanceFilter::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
