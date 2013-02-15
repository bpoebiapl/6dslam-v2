
#include "Transformation.h"

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/core/structure_only_solver.h"
#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>
#include <fstream>

void Transformation::show(){
	int max_points = 300;
	IplImage* rgb_img_src 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_src = (char *)rgb_img_src->imageData;
	IplImage* rgb_img_dst 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_dst = (char *)rgb_img_dst->imageData;
		
	int width = rgb_img_src->width;
	int height = rgb_img_src->height;
		
	IplImage* img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
	char * data = (char *)img_combine->imageData;
		
	vector<KeyPoint * > src_keypoints;//	= src->keypoints->valid_key_points;
	int nr_loop_src = src->keypoints->valid_key_points.size();
	if(nr_loop_src > max_points){nr_loop_src = max_points;}
	int nr_loop_dst = dst->keypoints->valid_key_points.size();
	if(nr_loop_dst > max_points){nr_loop_dst = max_points;}
	
	for(int i = 0; i < nr_loop_src; i++)
	{
		src_keypoints.push_back(src->keypoints->valid_key_points.at(i));
	}

	vector<KeyPoint * > dst_keypoints;//	= dst->keypoints->valid_key_points;
	for(int i = 0; i < nr_loop_dst; i++)
	{
		dst_keypoints.push_back(dst->keypoints->valid_key_points.at(i));
	}

	int src_nr_points = src_keypoints.size();
	int dst_nr_points = dst_keypoints.size();

	int index = 0;
		
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			
			int ind = 3*(640*j+i);
			data[3 * (j * (2*width) + (width+i)) + 0] = data_dst[ind +0];
			data[3 * (j * (2*width) + (width+i)) + 1] = data_dst[ind +1];
			data[3 * (j * (2*width) + (width+i)) + 2] = data_dst[ind +2];

			data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +0];
			data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +1];
			data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
			
		}
	}
	
	for(int i = 0; i < dst_nr_points;i++){
		cvCircle(img_combine,cvPoint(dst_keypoints.at(i)->point->w + width	, dst_keypoints.at(i)->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);}
	for(int i = 0; i < src_nr_points;i++){
		cvCircle(img_combine,cvPoint(src_keypoints.at(i)->point->w			, src_keypoints.at(i)->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);}
	
	//cvNamedWindow("src image", CV_WINDOW_AUTOSIZE );
	//cvShowImage("src image", rgb_img_src);
		
	//cvNamedWindow("dst image", CV_WINDOW_AUTOSIZE );
	//cvShowImage("dst image", rgb_img_dst);
	for(int i = 0; i < matches.size();i++){
		KeyPoint * src_kp = matches.at(i).first;
		KeyPoint * dst_kp = matches.at(i).second;
		cvCircle(img_combine,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		cvCircle(img_combine,cvPoint(src_kp->point->w			, src_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		cvLine(img_combine,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
	}
		
	cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
	cvShowImage("combined image", img_combine);
		
	cvWaitKey(0);
	cvReleaseImage( &rgb_img_src );
	cvReleaseImage( &rgb_img_dst );
}

void Transformation::show(pcl::visualization::CloudViewer * viewer){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	src_cloud->width    = src->cloud->width;
	src_cloud->height   = src->cloud->height;
	src_cloud->points.resize (src_cloud->width * src_cloud->height);
	
	int point_index = 0;
	
	for(int h = 0; h < src_cloud->height; h++)
	{
		for(int w = 0; w < src_cloud->width; w++)
		{
			src_cloud->points[point_index].x = src->cloud->points[point_index].x;
			src_cloud->points[point_index].y = src->cloud->points[point_index].y;
			src_cloud->points[point_index].z = src->cloud->points[point_index].z;
			src_cloud->points[point_index].r = 255.0f;//src->cloud->points[point_index].r;
			src_cloud->points[point_index].g = 0.0f;//src->cloud->points[point_index].g;
			src_cloud->points[point_index].b = 0.0f;//src->cloud->points[point_index].b;
			
			point_index++;
		}
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	dst_cloud->width    = dst->cloud->width;
	dst_cloud->height   = dst->cloud->height;
	dst_cloud->points.resize (dst_cloud->width * dst_cloud->height);
	point_index = 0;
	
	for(int h = 0; h < dst_cloud->height; h++)
	{
		for(int w = 0; w < dst_cloud->width; w++)
		{
			dst_cloud->points[point_index].x = dst->cloud->points[point_index].x;
			dst_cloud->points[point_index].y = dst->cloud->points[point_index].y;
			dst_cloud->points[point_index].z = dst->cloud->points[point_index].z;
			dst_cloud->points[point_index].r = 0.0f;//dst->cloud->points[point_index].r;
			dst_cloud->points[point_index].g = 0.0f;//dst->cloud->points[point_index].g;
			dst_cloud->points[point_index].b = 255.0f;//dst->cloud->points[point_index].b;
			
			point_index++;
		}
	}
		
	std::string text_cloud = "src";
	viewer->showCloud(src_cloud->makeShared(),text_cloud);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*src_cloud, *transformed_cloud, transformationMatrix);
	
	text_cloud = "dst";
	viewer->showCloud(dst_cloud->makeShared(),text_cloud);
	
	text_cloud = "final";
	point_index = 0;
	for(int h = 0; h < transformed_cloud->height; h++)
	{
		for(int w = 0; w < transformed_cloud->width; w++)
		{
			transformed_cloud->points[point_index].r = 0.0f;//dst->cloud->points[point_index].r;
			transformed_cloud->points[point_index].g = 255.0f;//dst->cloud->points[point_index].g;
			transformed_cloud->points[point_index].b = 0.0f;//dst->cloud->points[point_index].b;
			
			point_index++;
		}
	}
	viewer->showCloud(transformed_cloud->makeShared(),text_cloud);
	
	while (!viewer->wasStopped ()){usleep(10000);}
};
