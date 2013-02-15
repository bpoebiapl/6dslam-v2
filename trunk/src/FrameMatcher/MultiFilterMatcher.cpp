#include "MultiFilterMatcher.h"

using namespace std;

MultiFilterMatcher::MultiFilterMatcher()
{
	name = "MultiFilterMatcher";
	printf("new MultiFilterMatcher\n");
	//exit(0);
}

MultiFilterMatcher::~MultiFilterMatcher()
{
	printf("delete MultiFilterMatcher\n");
}

void MultiFilterMatcher::addFilter(TransformationFilter * f){
	filters.push_back(f);
}

void MultiFilterMatcher::update(){}

Transformation * MultiFilterMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	//printf("MultiFilterMatcher...\n");
	Transformation * transformation_old = new Transformation();
	transformation_old->src = src;
	transformation_old->dst = dst;
	transformation_old->transformationMatrix =  Eigen::Matrix4f::Identity();
	transformation_old->weight = 0;
	
	for(int i = 0; i < src->keypoints->valid_key_points.size();i++){		
		for(int j = 0; j < dst->keypoints->valid_key_points.size();j++){
			transformation_old->weight++;
			transformation_old->matches.push_back(make_pair(src->keypoints->valid_key_points.at(i), dst->keypoints->valid_key_points.at(j)));
		}
	}
	for(int i = 0; i < filters.size(); i++){
		//transformation_old->show();
		Transformation * transformation_new = filters.at(i)->filterTransformation(transformation_old);
		delete transformation_old;
		transformation_old = transformation_new;
	}
	//transformation_old->show();
	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("MultiFilterMatcher cost: %f\n",time);
	printf("transformation->weight: %f\n",transformation_old->weight);
	//transformation_old->show();
	
	return transformation_old;
}
