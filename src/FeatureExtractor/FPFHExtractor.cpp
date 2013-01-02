#include "FPFHExtractor.h"
#include <ctime>
#include "FPFHFeatureDescriptor.h"
#include <algorithm>

FPFHExtractor::FPFHExtractor(){
	feature_radius = 0.15;
	voxel_grid_size = 0.02f;
}

FPFHExtractor::~FPFHExtractor(){}
using namespace std;

KeyPointSet * FPFHExtractor::getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud){	
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr search_method 	= pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> vox_grid;
	vox_grid.setInputCloud (input_cloud);
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>); 
	vox_grid.filter (*tempCloud);
	
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud (tempCloud);
	fpfh_est.setInputNormals (tempCloud);
	fpfh_est.setSearchMethod (search_method);
	fpfh_est.setRadiusSearch (feature_radius);
	fpfh_est.compute (*features);
	KeyPointSet * keypoints = new KeyPointSet();
	for(int i = 0; i < tempCloud->width * tempCloud->height; i++){
		KeyPoint * kp = new KeyPoint();
		kp->stabilety = 1;
		kp->descriptor = new FPFHFeatureDescriptor();
		((FPFHFeatureDescriptor *)kp->descriptor)->feature = features->points[i];
		kp->point = new Point(tempCloud->points[i].x,tempCloud->points[i].y,tempCloud->points[i].z,-1,-1);
		kp->r = tempCloud->points[i].r;
		kp->g = tempCloud->points[i].g;
		kp->b = tempCloud->points[i].b;
		kp->valid = true;
		kp->index_number = keypoints->valid_key_points.size();
		keypoints->valid_key_points.push_back(kp);
	}

	return keypoints;
}

KeyPointSet * FPFHExtractor::getKeyPointSet(IplImage * rgb_img,IplImage * depth_img){return new KeyPointSet();}
