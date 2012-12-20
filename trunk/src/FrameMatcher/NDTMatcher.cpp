#include "NDTMatcher.h"
#include <pcl/registration/ndt.h>

using namespace std;

NDTMatcher::NDTMatcher()
{
	name = "NDTMatcher";
	printf("new NDTMatcher\n");
	nr_iters = 15;
	leafsize = 0.02;
	stepSize = 0.09;
	resolution = 0.1;
	epsilon = 1e-6;
}

NDTMatcher::~NDTMatcher()
{
	printf("delete NDTMatcher\n");
}

void NDTMatcher::update(){}

Transformation * NDTMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_src (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_dst (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize	(leafsize, leafsize, leafsize);
	
	approximate_voxel_filter.setInputCloud	(src->xyz_);
	approximate_voxel_filter.filter			(*filtered_src);
	approximate_voxel_filter.setInputCloud	(dst->xyz_);
	approximate_voxel_filter.filter			(*filtered_dst);
	std::cout << "Filtered cloud contains " << filtered_src->size () << " points from" << src->xyz_->size() << std::endl;
	std::cout << "Filtered cloud contains " << filtered_dst->size () << " points from" << dst->xyz_->size() << std::endl;
	
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(epsilon);
	ndt.setStepSize				(stepSize);
	ndt.setResolution			(resolution);
	ndt.setMaximumIterations 	(nr_iters);
	ndt.setInputCloud			(filtered_src);
	ndt.setInputTarget 			(filtered_dst);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align (*output_cloud, Eigen::Matrix4f::Identity());
	
	Transformation * transformation = new Transformation();
	transformation->src = src;
	transformation->dst = dst;
	transformation->transformationMatrix = ndt.getFinalTransformation();
	transformation->weight = 1/ndt.getFitnessScore();
	return transformation;
/*	
	
	
	
	cout<<"icp: \n"<<init_guess<<endl;
	cout<<"ndt: \n"<<transformation->transformationMatrix<<endl;
	cout<<"diff:\n"<<transformation->transformationMatrix*init_guess.inverse()<<endl;

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	src_cloud->width    = src->xyz_->width;
	src_cloud->height   = src->xyz_->height;
	src_cloud->points.resize (src_cloud->width * src_cloud->height);
	
	int point_index = 0;
	
	for(int h = 0; h < src_cloud->height; h++)
	{
		for(int w = 0; w < src_cloud->width; w++)
		{
			src_cloud->points[point_index].x = src->xyz_->points[point_index].x;
			src_cloud->points[point_index].y = src->xyz_->points[point_index].y;
			src_cloud->points[point_index].z = src->xyz_->points[point_index].z;
			src_cloud->points[point_index].r = 255.0f;//src->cloud->points[point_index].r;
			src_cloud->points[point_index].g = 0.0f;//src->cloud->points[point_index].g;
			src_cloud->points[point_index].b = 0.0f;//src->cloud->points[point_index].b;
			
			point_index++;
		}
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	dst_cloud->width    = dst->xyz_->width;
	dst_cloud->height   = dst->xyz_->height;
	dst_cloud->points.resize (dst_cloud->width * dst_cloud->height);
	point_index = 0;
	
	for(int h = 0; h < dst_cloud->height; h++)
	{
		for(int w = 0; w < dst_cloud->width; w++)
		{
			dst_cloud->points[point_index].x = dst->xyz_->points[point_index].x;
			dst_cloud->points[point_index].y = dst->xyz_->points[point_index].y;
			dst_cloud->points[point_index].z = dst->xyz_->points[point_index].z;
			dst_cloud->points[point_index].r = 0.0f;//dst->cloud->points[point_index].r;
			dst_cloud->points[point_index].g = 255.0f;//dst->cloud->points[point_index].g;
			dst_cloud->points[point_index].b = 0.0f;//dst->cloud->points[point_index].b;
			
			point_index++;
		}
	}
printf("transformation->weight = %f\n",transformation->weight);
Eigen::Matrix3f rotation = transformation->transformationMatrix.block<3,3>(0, 0);
Eigen::Vector3f translation = transformation->transformationMatrix.block<3,1>(0, 3);

printf ("\n");
printf ("    | %6.6f %6.6f %6.6f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
printf ("R = | %6.6f %6.6f %6.6f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
printf ("    | %6.6f %6.6f %6.6f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
printf ("\n");
printf ("t = < %0.6f, %0.6f, %0.6f >\n", translation (0), translation (1), translation (2));
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*dst_cloud, *transformed_cloud, transformation->transformationMatrix);

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(src_cloud->makeShared(),std::string("src"));
	//viewer.showCloud(dst_cloud->makeShared(),text_cloud);
	viewer.showCloud(transformed_cloud->makeShared(),std::string("dst"));
	
	while (!viewer.wasStopped ()){usleep(10000);}

	printf("done...\n");
	exit(0);
	*/
}
