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
#include "Map/Map3Dbase.h"
#include "Map/Map3Dbow.h"
#include "Map/Map3DbaseGraph.h"
#include "Map/Map3DPlanesGraph.h"
#include "graph/graph.h"
#include "FeatureDescriptor/FeatureDescriptor.h"
#include "core/core.h"
#include "FeatureExtractor/FeatureExtractor.h"
#include "mygeometry/mygeometry.h"
#include "FrameMatcher/FrameMatcher.h"
#include "RGBDSegmentation/RGBDSegmentation.h"

using namespace std;
using namespace Eigen;

vector< RGBDFrame * > * frames;

vector< Frame_input * > * getFrameInput(string path,int start, int max, Calibration * calibration){
	vector< Frame_input * > * all_input = new vector< Frame_input * >();
	char fpath [150];
	for(int i = start; i <= start+max; i+=1){
		Frame_input * fi =  new Frame_input();
		sprintf(fpath,"%s/rgb_frames/%05i.png",path.c_str(),i);
		//printf("%s\n",fpath);
		fi->rgb_path 			= string(fpath);
		sprintf(fpath,"%s/depth_frames/%05i.png",path.c_str(),i);
		//printf("%s\n",fpath);
		fi->depth_path			= string(fpath);
		fi->calibration			= calibration;
		all_input->push_back(fi);
	}
	return all_input;
}

int main(int argc, char **argv)
{
	printf("--------------------START--------------------\n");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0.5f, 0.5f, 0.5f);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->setBackgroundColor (0, 0, 0);
	
	vector<FeatureDescriptor * > words;
	for(int i = 0; i < 300; i++){
		char buff[50];
		sprintf(buff,"output/bowTest_%i.feature.ORB",i);
		words.push_back(new OrbFeatureDescriptor(string(buff)));
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
	orb->nr_features = 1300;
	orb->calibration = calib0;
	
	//Map3D * map = new Map3DbaseGraph();
	Map3D * map = new Map3DPlanesGraph();
	AICK * aick = new AICK();
	aick->max_points = 300;
	aick->distance_threshold = 0.0075f;
	map->matcher = aick;
	map->segmentation = new RGBDSegmentationBase();
	map->segmentation->calibration = calib0;
	map->extractor = orb;
	map->setVisualization(viewer);

	//vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",1250, 1200,calib0);
	vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",1250, 1200,calib0);
	for(int i = 0; i < all_input->size(); i+=3){
		printf("adding %i\n",i);
		map->addFrame(all_input->at(i));
	}
	printf("time to estimate\n");
	map->estimate();
	map->visualize();
	
	//segmenter = new RGBDSegmentation(calib0);
	//AICK * aick = new AICK();
	//aick->distance_threshold = 0.01;
	
	//test(all_frames,aick);
	printf("---------------------END---------------------\n");
	return 0;
}
