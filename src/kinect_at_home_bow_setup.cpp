#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <pthread.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include "Map/Map3D.h"
#include "graph/graph.h"
#include "FeatureDescriptor/FeatureDescriptor.h"
#include "core/core.h"
#include "FeatureExtractor/FeatureExtractor.h"
#include "mygeometry/mygeometry.h"
#include "FrameMatcher/FrameMatcher.h"
#include "TransformationFilter/TransformationFilter.h"
#include "RGBDSegmentation/RGBDSegmentation.h"

using namespace std;
using namespace Eigen;

vector< Frame_input * > * getFrameInput(string path,int start, int max, Calibration * calibration){
	vector< Frame_input * > * all_inp = new vector< Frame_input * >();
	char fpath [150];
	for(int i = start; i <= start+max; i+=1){
		Frame_input * fi =  new Frame_input();
		sprintf(fpath,"%s/rgb_frames/%05i.png",path.c_str(),i);
		printf("----------------------%i---------------------\n",i);
		printf("%s\n",fpath);
		fi->rgb_path 			= string(fpath);
		sprintf(fpath,"%s/depth_frames/%05i.png",path.c_str(),i);
		printf("%s\n",fpath);
		fi->depth_path			= string(fpath);
		fi->calibration			= calibration;
		all_inp->push_back(fi);
	}
	return all_inp;
}

int main(int argc, char **argv)
{
	printf("--------------------START--------------------\n");
	string bow_path = "output/bowTest7_%i.feature.surf";
	//string bow_path = "output/bowTest_%i.feature.ORB";
	
	vector<FeatureDescriptor * > words;
	
	for(int i = 0; i < 1; i++){
		char buff[250];
		sprintf(buff,bow_path.c_str(),i);
		words.push_back(new SurfFeatureDescriptor64(string(buff)));
	}
	
	Calibration * calib0 = new Calibration();
	calib0->fx			= 525.0;
	calib0->fy			= 525.0;
	calib0->cx			= 319.5;
	calib0->cy			= 239.5;
	calib0->ds			= 3*1;
	calib0->scale		= 5000;
	calib0->words 		= words;
	
	OrbExtractor * orb = new OrbExtractor();
	orb->nr_features = 1000;
	orb->calibration = calib0;
	
	SurfExtractor * surf = new SurfExtractor();
	surf->calibration = calib0;
	surf->thres *= 1.0;
	
	vector< Frame_input * > * all_input = getFrameInput("/home/johane/LibraryRecsForJohan/6e079fee-6c5f-42ec-b1d5-cb01cea5dedd",001, 4081,calib0);
	
	vector<Map3D *> maps;
	
	
	Map3D * map1;
/*
	map1 = new Map3Dbow("bow_output/library_10_1_10_%i.feature.orb", 10,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = orb;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_100_1_10_%i.feature.orb", 100,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = orb;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_500_1_10_%i.feature.orb", 500,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = orb;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_1000_1_10_%i.feature.orb", 1000,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = orb;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_5000_1_10_%i.feature.orb", 5000,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = orb;
	maps.push_back(map1);
*/	
	map1 = new Map3Dbow("bow_output/library_10_1_10_%i.feature.surf", 10,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = surf;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_100_1_10_%i.feature.surf", 100,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = surf;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_500_1_10_%i.feature.surf", 500,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = surf;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_1000_1_10_%i.feature.surf", 1000,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = surf;
	maps.push_back(map1);
	
	map1 = new Map3Dbow("bow_output/library_5000_1_10_%i.feature.surf", 5000,1, 10);
	map1->segmentation = new RGBDSegmentationDummy();
	map1->segmentation->calibration = calib0;
	map1->extractor = surf;
	maps.push_back(map1);
	
	for(int m = 0; m < maps.size(); m++){
		Map3D * map = maps.at(m);
		for(int i = 0; i < all_input->size(); i+=20){printf("i:%i\n",i);map->addFrame(all_input->at(i));}
		map->estimate();
	}

	return 0;
}
