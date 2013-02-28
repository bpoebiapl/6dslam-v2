#include "RGBDSegmentationPCL.h"
#include "mygeometry/mygeometry.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

RGBDSegmentationPCL::RGBDSegmentationPCL(){}
RGBDSegmentationPCL::~RGBDSegmentationPCL(){}
vector<Plane * > * RGBDSegmentationPCL::segment(IplImage * rgb_img,IplImage * depth_img){
	struct timeval start, end;
	gettimeofday(&start, NULL);
	
	vector<Plane * > * planes = new vector<Plane * >();
	
	int width = rgb_img->width;
	int height = rgb_img->height;

	float d_scaleing	= calibration->ds/calibration->scale;
	float centerX		= calibration->cx;
	float centerY		= calibration->cy;
	float invFocalX	= 1.0f/calibration->fx;
    float invFocalY	= 1.0f/calibration->fy;
	
	char * rgb_data		= (char *)rgb_img->imageData;
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width  = width;
	cloud->height = height;
	cloud->points.resize (cloud->width * cloud->height);

	for(int i = 0; i < width; i++){
		for(int j = 0; j < height; j++){
			int ind = width*j+i;
			
			float tmp_z = float(depth_data[ind]) * d_scaleing;
			float tmp_x = 0;
			float tmp_y = 0;

			if(tmp_z > 0){
				tmp_x = (i - centerX) * tmp_z * invFocalX;
		       	tmp_y = (j - centerY) * tmp_z * invFocalY;
			}
			cloud->points[ind].x = tmp_x;
			cloud->points[ind].y = tmp_y;
			cloud->points[ind].z = tmp_z;
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud		(cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits	(0.00001, 100000);
	pass.filter				(*tmp_cloud);
	
	const float voxel_grid_size = 0.002f;//12f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud (tmp_cloud);
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
	vox_grid.filter (*tempCloud);
	
	printf("points for segmentation: %i\n",tempCloud->width*tempCloud->height);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (tempCloud->makeShared());//cloud.makeShared ());
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0){
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return planes;
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

	Plane * p = new Plane();
	p->normal_x = coefficients->values[0];
	p->normal_y = coefficients->values[1];
	p->normal_z = coefficients->values[2];

	p->point_x = coefficients->values[0]*coefficients->values[3];
	p->point_y = coefficients->values[1]*coefficients->values[3];
	p->point_z = coefficients->values[2]*coefficients->values[3];
	
	planes->push_back(p);

	gettimeofday(&end, NULL);
	float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	printf("Segment cost: %f\n",time);
	return planes;
}
