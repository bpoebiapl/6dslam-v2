#include "Map3Dbase.h"
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

bool comparison_Map3Dbase (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

using namespace std;

Map3Dbase::Map3Dbase(){}
Map3Dbase::~Map3Dbase(){}
void Map3Dbase::addFrame(Frame_input * fi){addFrame(new RGBDFrame(fi,extractor,segmentation));}
void Map3Dbase::addFrame(RGBDFrame * frame){
	printf("Map3Dbase::addFrame(RGBDFrame * frame)\n");
	if(frames.size() > 0){
		transformations.push_back(matcher->getTransformation(frame, frames.back()));
	}
	for(int i = 0; i < frames.size(); i++){
		float d = frames.at(i)->image_descriptor->distance(frame->image_descriptor);
		//printf("d: %f\n",d);
	}
	frames.push_back(frame);
}
void Map3Dbase::addTransformation(Transformation * transformation){}
void Map3Dbase::estimate(){
	printf("estimate\n");
	sort(transformations.begin(),transformations.end(),comparison_Map3Dbase);
	printf("sorted\n");
	poses.push_back(Matrix4f::Identity());
	for(int i = 0; i < transformations.size(); i++){
		printf("id:%i ---------> %i <--> %i : %f\n",i,transformations.at(i)->src->id,transformations.at(i)->dst->id,transformations.at(i)->weight);
		poses.push_back(poses.back()*transformations.at(i)->transformationMatrix);
	}
	printf("estimate done\n");
}
void Map3Dbase::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

//bool goToNext = false;
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void){goToNext = true;}

void Map3Dbase::visualize(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = 0;
	cloud->height   = 1;
	for(int i = 0; i < poses.size(); i+=10){cloud->width+=frames.at(i)->xyz_->width;}
	cloud->points.resize (cloud->width);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width    = 0;
	for(int i = 0; i < poses.size(); i+=10){
		//*(frames.at(i)->xyz_);
		//*tmp_cloud;
		//poses.at(i);
		pcl::transformPointCloud (*(frames.at(i)->xyz_), *tmp_cloud, poses.at(i));
		int randr = 0;rand()%256;
		int randg = 255;rand()%256;
		int randb = 0;rand()%256;
		for(int j = 0; j < tmp_cloud->width*tmp_cloud->height; j++)
		{
			cloud->points[cloud->width].x = tmp_cloud->points[j].x;
			cloud->points[cloud->width].y = tmp_cloud->points[j].y;
			cloud->points[cloud->width].z = tmp_cloud->points[j].z;
			cloud->points[cloud->width].r = randr;
			cloud->points[cloud->width].g = randg;
			cloud->points[cloud->width].b = randb;
			cloud->width++;
		}
	}
	
	//viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->removePointCloud("sample cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
		
	usleep(100);
	while (true){
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	
}

