#include "RansacPCLCopyMatcher.h"
#include "Point.h"
#include <vector>

using namespace std;

RansacPCLCopyMatcher::RansacPCLCopyMatcher()
{
	name = "RansacPCLCopyMatcher";
	printf("new RansacPCLCopyMatcher\n");
	nr_iter 			= 1000;
	distance_threshold 	= 0.015f;
	feature_threshold 	= 0.15f;
}

RansacPCLCopyMatcher::~RansacPCLCopyMatcher(){}

void RansacPCLCopyMatcher::update(){}

const bool debugg_RansacPCLCopyMatcher = false;
Transformation * RansacPCLCopyMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	if(debugg_RansacPCLCopyMatcher){printf("RansacPCLCopyMatcher::getTransformation(%i,%i)\n",src->id,dst->id);}
	
	Transformation * transformation = new Transformation();
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 0;
	transformation->transformationMatrix = Eigen::Matrix4f::Identity();

	vector<KeyPoint * > src_keypoints = src->keypoints->valid_key_points;
	vector<KeyPoint * > dst_keypoints = dst->keypoints->valid_key_points;

	int src_nr_points = src_keypoints.size();
	int dst_nr_points = dst_keypoints.size();

	vector<int> ** possibleMatches	= new vector<int>*[src_nr_points];
	for(int i = 0; i < src_nr_points;i++)
	{
		possibleMatches[i]		= new vector<int>();	
		for(int j = 0; j < dst_nr_points;j++)
		{
			FeatureDescriptor * descriptorA = src_keypoints.at(i)->descriptor;
			FeatureDescriptor * descriptorB = dst_keypoints.at(j)->descriptor;
			if(descriptorA->distance(descriptorB) < feature_threshold){possibleMatches[i]->push_back(j);}
		}
		//printf("%i:%i\n",i,possibleMatches[i]->size());
	}

	
	float * pos_src_x = new float[src_nr_points];
	float * pos_src_y = new float[src_nr_points];
	float * pos_src_z = new float[src_nr_points];
	
	float * pos_dst_x = new float[dst_nr_points];
	float * pos_dst_y = new float[dst_nr_points];
	float * pos_dst_z = new float[dst_nr_points];
	
	Eigen::Matrix4f transformationMat = Eigen::Matrix4f::Identity();
	
	for(int i = 0; i < src_nr_points;i++)
	{
		pos_src_x[i] = src_keypoints[i]->point->x;
		pos_src_y[i] = src_keypoints[i]->point->y;
		pos_src_z[i] = src_keypoints[i]->point->z;
	}
	for(int i = 0; i < dst_nr_points;i++)
	{
		pos_dst_x[i] = dst_keypoints[i]->point->x;
		pos_dst_y[i] = dst_keypoints[i]->point->y;
		pos_dst_z[i] = dst_keypoints[i]->point->z;
	}
	
	int nr_best = 0;
	pcl::TransformationFromCorrespondences tfc;
	for(int i = 0; i < nr_iter; i++)
	{
		int src_a,src_b,src_c,dst_a,dst_b,dst_c;
		int counter = 0;
		src_a = rand() % src_nr_points;
		src_b = rand() % src_nr_points;
		src_c = rand() % src_nr_points;
		while(possibleMatches[src_a]->size() == 0 && counter < 500)										{counter++;src_a = rand() % src_nr_points;}
		while(possibleMatches[src_b]->size() == 0 || src_a == src_b  && counter < 500)					{counter++;src_b = rand() % src_nr_points;}
		while(possibleMatches[src_c]->size() == 0 || src_c == src_a || src_c == src_b && counter < 500)	{counter++;src_c = rand() % src_nr_points;}
		if(counter >= 500){continue;}
		dst_a = possibleMatches[src_a]->at(rand()%possibleMatches[src_a]->size());
		dst_b = possibleMatches[src_b]->at(rand()%possibleMatches[src_b]->size());
		dst_c = possibleMatches[src_c]->at(rand()%possibleMatches[src_c]->size());
		
		tfc.reset();
		tfc.add(src_keypoints.at(src_a)->point->pos,dst_keypoints.at(dst_a)->point->pos);
		tfc.add(src_keypoints.at(src_b)->point->pos,dst_keypoints.at(dst_b)->point->pos);
		tfc.add(src_keypoints.at(src_c)->point->pos,dst_keypoints.at(dst_c)->point->pos);
		transformationMat = tfc.getTransformation().matrix();
		
		float mat00 = transformationMat(0,0);
		float mat01 = transformationMat(0,1);
		float mat02 = transformationMat(0,2);
		float mat03 = transformationMat(0,3);
		float mat10 = transformationMat(1,0);
		float mat11 = transformationMat(1,1);
		float mat12 = transformationMat(1,2);
		float mat13 = transformationMat(1,3);
		float mat20 = transformationMat(2,0);
		float mat21 = transformationMat(2,1);
		float mat22 = transformationMat(2,2);
		float mat23 = transformationMat(2,3);
		
		std::vector<int> src_good;
		std::vector<int> dst_good;
		
		for(int src_j = 0; src_j < src_nr_points; src_j++)
		{
			float x = pos_src_x[src_j];
			float y = pos_src_y[src_j];
			float z = pos_src_z[src_j];
			float tmp_x = x*mat00+y*mat01+z*mat02+mat03;
			float tmp_y = x*mat10+y*mat11+z*mat12+mat13;
			float tmp_z = x*mat20+y*mat21+z*mat22+mat23;
			float min = 100;
			int closest = -1;
			for(int k = 0; k < possibleMatches[src_j]->size(); k++){
				int dst_j = possibleMatches[src_j]->at(k);
				
				float dx = pos_dst_x[dst_j] - tmp_x;
				float dy = pos_dst_y[dst_j] - tmp_y;
				float dz = pos_dst_z[dst_j] - tmp_z;
				float dist = sqrt(dx*dx + dy*dy + dz*dz);
				if(dist < min){
					min = dist;
					closest = dst_j;
				}
			}

			if(min < distance_threshold){
				//printf("%i : %i -> %f\n",src_j,closest,min);
				src_good.push_back(src_j);
				dst_good.push_back(closest);
			}
		}
		if(src_good.size() > nr_best){
			nr_best = src_good.size();
			tfc.reset();
			for(int j = 0; j < src_good.size(); j++){
				tfc.add(src_keypoints.at(src_good.at(j))->point->pos,dst_keypoints.at(dst_good.at(j))->point->pos);
			}
			transformation->transformationMatrix = tfc.getTransformation().matrix();
		}

	}
	
	for(int i = 0; i < src_nr_points;i++)
	{
		delete possibleMatches[i];
	}
	delete[] possibleMatches;
	delete[] pos_src_x;
	delete[] pos_src_y;
	delete[] pos_src_z;
	
	delete[] pos_dst_x;
	delete[] pos_dst_y;
	delete[] pos_dst_z;
	return transformation;
}
