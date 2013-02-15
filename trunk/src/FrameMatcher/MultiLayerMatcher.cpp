#include "MultiLayerMatcher.h"

using namespace std;

MultiLayerMatcher::MultiLayerMatcher()
{
	name = "MultiLayerMatcher";
	printf("new MultiLayerMatcher\n");
	weightLimit = 30;
	//exit(0);
}

MultiLayerMatcher::~MultiLayerMatcher()
{
	printf("delete MultiLayerMatcher\n");
}

void MultiLayerMatcher::addMatcher(FrameMatcher * fm){
	matchers.push_back(fm);
}

void MultiLayerMatcher::update(){}

Transformation * MultiLayerMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	Transformation * transformation = new Transformation();
	transformation->weight = 0;
	for(int i = 0; i < matchers.size(); i++){
		delete transformation;
		transformation = matchers.at(i)->getTransformation(src,dst);
		
		if(transformation->weight < weightLimit){
			//transformation->show();
		}
		else{break;}
	}
	if(transformation->weight < weightLimit){
		transformation->transformationMatrix = Eigen::Matrix4f::Identity();
		transformation->weight = 0;
	}
	return transformation;
}
