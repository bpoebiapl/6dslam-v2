#include "BowFilter.h"

using namespace std;
BowFilter::BowFilter(){
	feature_threshold = 0.11f;
}
BowFilter::~BowFilter(){}
Transformation * BowFilter::filterTransformation(Transformation * input){
	//printf("BowFilter:start\n");
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = input->transformationMatrix;
	transformation->src = input->src;
	transformation->dst = input->dst;
	
	transformation->weight = 0;
	for(int i = 0; i < input->matches.size();i++){
		KeyPoint * src_kp = input->matches.at(i).first;
		KeyPoint * dst_kp = input->matches.at(i).second;
		int src_word = src_kp->cluster_distance_pairs.at(0).first;
		float src_word_dist = dst_kp->cluster_distances.at(src_word);
		
		int dst_word = dst_kp->cluster_distance_pairs.at(0).first;
		float dst_word_dist = src_kp->cluster_distances.at(dst_word);
		
		if(src_word_dist < feature_threshold && dst_word_dist < feature_threshold){	
			transformation->matches.push_back(make_pair(src_kp, dst_kp));
		}
	}
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("BowFilter cost: %f\n",time);
	//printf("BowFilter:end\n");
	return transformation;
}
void BowFilter::print(){printf("%s\n",name.c_str());}
void BowFilter::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
