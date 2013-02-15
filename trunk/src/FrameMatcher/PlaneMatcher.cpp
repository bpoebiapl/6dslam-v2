#include "PlaneMatcher.h"

using namespace std;

PlaneMatcher::PlaneMatcher()
{
	name = "PlaneMatcher";
	printf("new PlaneMatcher\n");
	weightLimit = 30;
	//exit(0);
}

PlaneMatcher::~PlaneMatcher()
{
	printf("delete PlaneMatcher\n");
}

void PlaneMatcher::addMatcher(FrameMatcher * fm){
	matchers.push_back(fm);
}

void PlaneMatcher::update(){}

Transformation * PlaneMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	//printf("PlaneMatcher::getTransformation\n");
	Transformation * transformation = new Transformation();
	transformation->weight = 0;
	for(int i = 0; i < matchers.size(); i++){
		delete transformation;
		//printf("%i:getTransformation\n",i);
		transformation = matchers.at(i)->getTransformation(src,dst);
		//printf("transformation->weight = %f\n",transformation->weight);
		if(transformation->weight < weightLimit){/*transformation->show();*/}
		else{break;}
	}
	printf("-----------------------------------------------------------------------------\n");
	printf("transformation->weight: %f\n",transformation->weight);
	if(transformation->weight < weightLimit){
		transformation->transformationMatrix = Eigen::Matrix4f::Identity();
		transformation->weight = 0;
	}
	return transformation;
}
