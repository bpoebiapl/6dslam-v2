#include "Map3Dbow.h"
#include "Frame_input.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

Map3Dbow::Map3Dbow(string file_path){
	path=file_path;
}
Map3Dbow::Map3Dbow(){
	path="output/bowTest2_%i.feature.surf";
}
Map3Dbow::~Map3Dbow(){}
void Map3Dbow::addFrame(Frame_input * fi){addFrame(new RGBDFrame(fi,extractor,segmentation));}
void Map3Dbow::addFrame(RGBDFrame * frame){
	printf("Map3Dbow::addFrame(RGBDFrame * frame)\n");
	frames.push_back(frame);
}
void Map3Dbow::addTransformation(Transformation * transformation){}
void Map3Dbow::estimate(){
	printf("estimate\n");
	vector<FeatureDescriptor *> descriptors;
	for(int i  = 0; i < frames.size(); i++){
		KeyPointSet * current = frames.at(i)->keypoints;
		for(int j = 0; j < current->valid_key_points.size(); j++){descriptors.push_back(current->valid_key_points.at(j)->descriptor);}
		for(int j = 0; j < current->invalid_key_points.size(); j++){descriptors.push_back(current->invalid_key_points.at(j)->descriptor);}	
	}
	vector<FeatureDescriptor * > * bags = kmeans(descriptors, 1, 10, 500);
	
	for(int i = 0; i < bags->size(); i++){
		char buff[250];
		sprintf(buff,path.c_str(),i);
		bags->at(i)->store(string(buff));
		bags->at(i)->print();
		//new OrbFeatureDescriptor(string(buff));
	}
	printf("estimate done\n");
}
void Map3Dbow::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

void Map3Dbow::visualize(){}

vector<FeatureDescriptor * > * Map3Dbow::kmeans(vector<FeatureDescriptor *> input, int nr_restarts, int iterations, int nr_clusters)
{
	printf("doing kmeans with %i features\n",input.size());
	float best_sum = -1;
	vector<FeatureDescriptor * > * best_centers = new vector<FeatureDescriptor * >();
	for(int restart = 0; restart < nr_restarts; restart++)
	{
		printf("--------------------------------------------------------------------------------\n");
		vector<FeatureDescriptor * > * centers = new vector<FeatureDescriptor * >();
		for(int j = 0; j < nr_clusters; j++){centers->push_back(input.at(rand()%input.size())->clone());}
		float sum;
		
		for(int iter = 0; iter < iterations; iter++)
		{
			vector< vector<FeatureDescriptor * > * > * centers_data = new vector< vector <FeatureDescriptor * > * >();
			for(int j = 0; j < nr_clusters; j++){centers_data->push_back(new vector<FeatureDescriptor *>());}
			
			sum = 0;
			for(int i = 0; i < input.size(); i++)
			{
				FeatureDescriptor * current = input.at(i);
				float best = 99999;
				int best_id = -1;
				for(int j = 0; j < nr_clusters; j++)
				{
					float dist = current->distance(centers->at(j));
					
					if(best > dist){
						best = dist;
						best_id = j;
					}
				}
				sum+=best*best;
				centers_data->at(best_id)->push_back(current);
			}
			
			sum /= float(input.size());
			printf("errorsum: %f\n",sum);
			
			for(int j = 0; j < nr_clusters; j++)
			{
				if(centers_data->at(j)->size() > 0){
					centers->at(j)->update(centers_data->at(j));
				}
			}
			
			for(int j = 0; j < nr_clusters; j++){delete centers_data->back(); centers_data->pop_back();}
			delete centers_data;
		}
		
		if(best_sum == -1){
			best_centers = centers;
			best_sum = sum;
		}else if(sum < best_sum){
			//for(int i = 0; i < nr_clusters; i++){delete centers[i];}
			//delete[] centers;
			best_centers = centers;
			best_sum = sum;
		}else{
			for(int i = 0; i < nr_clusters; i++){delete centers->at(i);}
			delete centers;
		}
	}
	return best_centers;
}

