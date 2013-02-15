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
/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = poses.size();
	cloud->height   = 1;
	cloud->points.resize (cloud->width);
	cloud->width = 0;
	for(int i = 0; i < poses.size(); i+=1){
		int randr = 0;rand()%256;
		int randg = 255;rand()%256;
		int randb = 0;rand()%256;
		Eigen::Matrix4f m1 = poses.at(i);
		cloud->points[cloud->width].x = m1(0,3);
		cloud->points[cloud->width].y = m1(1,3);
		cloud->points[cloud->width].z = m1(2,3);
		printf("%f %f %f\n",cloud->points[cloud->width].x,cloud->points[cloud->width].y,cloud->points[cloud->width].z);
		cloud->points[cloud->width].r = randr;
		cloud->points[cloud->width].g = randg;
		cloud->points[cloud->width].b = randb;
		cloud->width++;
	}
	
	for(int i = 0; i < transformations.size(); i+=1){
		char buf[100];
		sprintf(buf,"line%i.%i",transformations.at(i)->src->id,transformations.at(i)->dst->id);
		//viewer->addLine<pcl::PointXYZRGB> (cloud->points[transformations.at(i)->src->id],cloud->points[transformations.at(i)->dst->id], buf);
	}
	
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = 0;
	cloud->height   = 1;
	for(int i = 0; i < poses.size(); i+=40){cloud->width+=frames.at(i)->xyz_->width;}
	cloud->points.resize (cloud->width);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width    = 0;
	for(int i = 0; i < poses.size(); i+=40){
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
	*/
	bool render_full = true;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if(!render_full){
		cloud->width    = poses.size();
		cloud->height   = 1;
		cloud->points.resize (cloud->width);
		cloud->width = 0;
		for(int i = 0; i < poses.size(); i+=1){
			int randr = 0;rand()%256;
			int randg = 255;rand()%256;
			int randb = 0;rand()%256;
			Eigen::Matrix4f m1 = poses.at(i);
			cloud->points[cloud->width].x = m1(0,3);
			cloud->points[cloud->width].y = m1(1,3);
			cloud->points[cloud->width].z = m1(2,3);
			printf("%f %f %f\n",cloud->points[cloud->width].x,cloud->points[cloud->width].y,cloud->points[cloud->width].z);
			cloud->points[cloud->width].r = randr;
			cloud->points[cloud->width].g = randg;
			cloud->points[cloud->width].b = randb;
			cloud->width++;
		}
	}else{
		int step = poses.size()/100;
		if(step<1){step = 1;}
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		tmpcloud->width    = 640*480;
		tmpcloud->height   = 1;
		tmpcloud->points.resize (tmpcloud->width);
	
		cloud->width    = 0;
		cloud->height   = 1;
		//cloud->points.resize (640*480*200);
	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
		for(int i = 0; i < frames.size(); i+=step){
			RGBDFrame * frame = frames.at(i);
			tmpcloud->width = 0;
			IplImage* rgb_img 	= cvLoadImage(frame->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
			IplImage* depth_img = cvLoadImage(frame->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		
			float d_scaleing	= frame->input->calibration->ds/frame->input->calibration->scale;
			float centerX		= frame->input->calibration->cx;
			float centerY		= frame->input->calibration->cy;
			float invFocalX		= 1.0f/frame->input->calibration->fx;
			float invFocalY		= 1.0f/frame->input->calibration->fy;
		
			char * rgb_data		= (char *)rgb_img->imageData;
			unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
		
			int pixelstep = 2;
			for(int w = pixelstep; w < 640; w+=pixelstep){
				for(int h = pixelstep; h < 480; h+=pixelstep){
					int ind = 640*h+w;
					float x = 0;
					float y = 0;
					float z = float(depth_data[ind]) * d_scaleing;
				
					int r = char(rgb_data[3*ind+2]);
					int g = char(rgb_data[3*ind+1]);
					int b = char(rgb_data[3*ind+0]);
				
					if(r < 0){r = 255+r;}
					if(g < 0){g = 255+g;}
					if(b < 0){b = 255+b;}

					if(z > 0 && z < 1.25f){
						x = (w - centerX) * z * invFocalX;
					   	y = (h - centerY) * z * invFocalY;
					   	
					   	tmpcloud->points[tmpcloud->width].x = x;
						tmpcloud->points[tmpcloud->width].y = y;
						tmpcloud->points[tmpcloud->width].z = z;
						tmpcloud->points[tmpcloud->width].r = float(r);
						tmpcloud->points[tmpcloud->width].g = float(g);
						tmpcloud->points[tmpcloud->width].b = float(b);
						tmpcloud->width++;
					}
				}
			}
		
			cvReleaseImage( &rgb_img );
			cvReleaseImage( &depth_img );
		
			pcl::transformPointCloud (*tmpcloud, *tmpcloud2, poses.at(i));
		
			cloud->points.resize (cloud->width+tmpcloud2->width);
		
			for(int j = 0; j < tmpcloud2->width*tmpcloud2->height; j++)
			{
				cloud->points[cloud->width].x = tmpcloud2->points[j].x;
				cloud->points[cloud->width].y = tmpcloud2->points[j].y;
				cloud->points[cloud->width].z = tmpcloud2->points[j].z;
				cloud->points[cloud->width].r = tmpcloud2->points[j].r;
				cloud->points[cloud->width].g = tmpcloud2->points[j].g;
				cloud->points[cloud->width].b = tmpcloud2->points[j].b;
				cloud->width++;
			}
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

