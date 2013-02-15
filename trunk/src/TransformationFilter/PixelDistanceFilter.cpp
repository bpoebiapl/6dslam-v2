#include "PixelDistanceFilter.h"

using namespace std;
PixelDistanceFilter::PixelDistanceFilter(){
	pixel_threshold = 30;
}
PixelDistanceFilter::~PixelDistanceFilter(){}
Transformation * PixelDistanceFilter::filterTransformation(Transformation * input){
	
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
		float pixel_distance = sqrt((src_kp->point->w-dst_kp->point->w)*(src_kp->point->w-dst_kp->point->w)+(src_kp->point->h-dst_kp->point->h)*(src_kp->point->h-dst_kp->point->h));
		if(pixel_distance<pixel_threshold){	
			transformation->matches.push_back(make_pair(src_kp, dst_kp));
		}
	}
	
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	//printf("PixelDistanceFilter cost: %f\n",time);
	
	return transformation;
}
void PixelDistanceFilter::print(){printf("%s\n",name.c_str());}
void PixelDistanceFilter::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
