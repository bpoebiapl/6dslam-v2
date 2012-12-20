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
#include "graph/graph.h"
#include "FeatureDescriptor/FeatureDescriptor.h"
#include "core/core.h"
#include "FeatureExtractor/FeatureExtractor.h"
#include "mygeometry/mygeometry.h"
#include "FrameMatcher/FrameMatcher.h"

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
/*
bool goToNext = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
	printf("key pressed\n");
	goToNext = true;

}

void test(vector< RGBDFrame * > * all_frames, FrameMatcher * matcher)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = 1000000;
	cloud->height   = 1;
	cloud->points.resize (cloud->width * cloud->height);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	Eigen::Matrix4f transformationMat = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f current = transformationMat;
	Eigen::Matrix4f prev = transformationMat;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0.5f, 0.5f, 0.5f);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	
	int display = 0;
	int start = 0;
	int step = 2;
	int counter = 0;
	for(int i = step+start; i < (int)all_frames->size(); i+=step){
		printf("------------------------------------------------------------------------------------------\n");
		printf("i:%i\n",i);
		Transformation * trans = matcher->getTransformation(all_frames->at(i-step), all_frames->at(i));
		current = (trans->transformationMatrix).inverse();
		cout<<current<<endl;
		Eigen::Matrix4f current_diff = current-prev;
		float diff = 0;//
		for(int i = 0; i < 3; i++){for(int j = 0; j < 3; j++){diff += current_diff(i,j)*current_diff(i,j);}}
		float poss_diff = sqrt(current_diff(0,3)*current_diff(0,3)+current_diff(1,3)*current_diff(1,3)+current_diff(2,3)*current_diff(2,3));
		printf("diff: %f || poss_diff: %f || weight: %f\n",diff,poss_diff,trans->weight);
		transformationMat*=current;
		prev = current;
		
		pcl::transformPointCloud (*(all_frames->at(i)->xyz_), *tmp_cloud, transformationMat);
		int randr = rand()%256;
		int randg = rand()%256;
		int randb = rand()%256;
		for(int j = 0; j < tmp_cloud->width*tmp_cloud->height; j++)
		{
			cloud->points[counter+j].x = tmp_cloud->points[j].x;
			cloud->points[counter+j].y = tmp_cloud->points[j].y;
			cloud->points[counter+j].z = tmp_cloud->points[j].z;
			cloud->points[counter+j].r = randr;
			cloud->points[counter+j].g = randg;
			cloud->points[counter+j].b = randb;

		}
		counter+=tmp_cloud->width*tmp_cloud->height;
		cloud->width    = counter;
		
		if(display%5 == 0){
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
			viewer->removePointCloud("sample cloud");
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
		
			usleep(100);
			goToNext = false;
			while (!goToNext){
				viewer->spinOnce (100);
				boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			}
		}
		display++;
	}
}
*/
int main(int argc, char **argv)
{
	printf("--------------------START--------------------\n");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0.5f, 0.5f, 0.5f);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->setBackgroundColor (0, 0, 0);
	
	Calibration * calib0 = new Calibration();
	calib0->fx			= 525.0;
	calib0->fy			= 525.0;
	calib0->cx			= 319.5;
	calib0->cy			= 239.5;
	calib0->ds			= 3*1;
	calib0->scale		= 5000;
	
	OrbExtractor * orb = new OrbExtractor();
	orb->nr_features = 1300;
	orb->calibration = calib0;
	
	//Map3D * map = new Map3DbaseGraph();
	Map3D * map = new Map3Dbow();
	AICK * aick = new AICK();
	aick->max_points = 1300;
	aick->distance_threshold = 0.0075f;
	map->matcher = aick;
	map->extractor = orb;
	map->setVisualization(viewer);

	vector< Frame_input * > * all_input = getFrameInput("/home/johane/johan_cvap_run",1250, 1200,calib0);
	for(int i = 0; i < all_input->size(); i+=6){
		printf("adding %i\n",i);
		map->addFrame(all_input->at(i));
	}
	
	map->estimate();
	map->visualize();
	
	//segmenter = new RGBDSegmentation(calib0);
	//AICK * aick = new AICK();
	//aick->distance_threshold = 0.01;
	
	//test(all_frames,aick);
	printf("---------------------END---------------------\n");
	return 0;
}
