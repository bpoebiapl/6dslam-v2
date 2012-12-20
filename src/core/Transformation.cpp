
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
