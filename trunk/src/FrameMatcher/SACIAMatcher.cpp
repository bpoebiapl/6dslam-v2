#include "SACIAMatcher.h"
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

struct OrbPointType
{
	float descriptor[32];
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

inline std::ostream& operator << (std::ostream& os, const OrbPointType& p){
	for (int i = 0; i < 32; ++i)
		os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 31 ? ", " : ")");
	return (os);
}

POINT_CLOUD_REGISTER_POINT_STRUCT(OrbPointType,(float, descriptor, descriptor))

struct Surf64PointType
{
	float histogram[64];
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

inline std::ostream& operator << (std::ostream& os, const Surf64PointType& p){
	for (int i = 0; i < 64; ++i)
		os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 63 ? ", " : ")");
	return (os);
}

POINT_CLOUD_REGISTER_POINT_STRUCT(Surf64PointType,(float, histogram, histogram))

struct Surf128PointType
{
	float descriptor[128];
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

inline std::ostream& operator << (std::ostream& os, const Surf128PointType& p){
	for (int i = 0; i < 128; ++i)
		os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 127 ? ", " : ")");
	return (os);
}

POINT_CLOUD_REGISTER_POINT_STRUCT(Surf128PointType,(float, descriptor, descriptor))

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/ia_ransac.h>

//http://pointclouds.org/documentation/tutorials/template_alignment.php
    
using namespace std;

SACIAMatcher::SACIAMatcher()
{
	name = "SACIAMatcher";
	printf("new SACIAMatcher\n");
	SearchMethod::Ptr smxyz (new SearchMethod);
	search_method_xyz_ = smxyz;
	minSampleDistance = 0.05f;
	maxCorrespondenceDistance = 0.01f;
}

SACIAMatcher::~SACIAMatcher(){printf("delete SACIAMatcher\n");}

void SACIAMatcher::update(){}

Transformation * SACIAMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst)
{
	//printf("sacia start\n");

	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
	Transformation * transformation = new Transformation();
	transformation->src = src;
	transformation->dst = dst;
	pcl::PointCloud<pcl::PointXYZ> registration_output;
	
	sac_ia_.setMinSampleDistance (minSampleDistance);
	sac_ia_.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	sac_ia_.setMaximumIterations (500);
/*
	sac_ia_.setInputTarget(src->xyz_);
	sac_ia_.setTargetFeatures(src->features_);
	int ind = 0;
	for(int i = 0; i < src->features_->width; i++){
		for(int j = 0; j < src->features_->height; j++){
			//printf("%i:%i -> ",i,j);
			//std::cout << src->features_->points[ind] <<std::endl;
			//ind++; 
		}
	}
	sac_ia_.setInputCloud (dst->xyz_);
	sac_ia_.setSourceFeatures (dst->features_);

	printf("pre align\n");
	sac_ia_.align(registration_output);
	printf("post align\n");
	

	transformation->transformationMatrix = sac_ia_.getFinalTransformation();
	transformation->weight = 1/sac_ia_.getFitnessScore();
	
	Eigen::Matrix3f rotation = transformation->transformationMatrix.block<3,3>(0, 0);
	Eigen::Vector3f translation = transformation->transformationMatrix.block<3,1>(0, 3);

	printf ("\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	printf ("\n");
	printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	printf ("-----------------------------------------------------------------------------\n");

	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, OrbPointType> orb_sac_ia_;
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, Surf64PointType> surf64_sac_ia_;
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, Surf128PointType> surf128_sac_ia_;
	//vector<KeyPoint * > valid_key_points = src->keypoints.valid_key_points;
*/
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	src_xyz_cloud->width    = src->keypoints->valid_key_points.size();
	src_xyz_cloud->height   = 1;
	src_xyz_cloud->points.resize (src_xyz_cloud->width * src_xyz_cloud->height);

	for(int i = 0; i < src->keypoints->valid_key_points.size(); i++)
	{
		src_xyz_cloud->points[i].x = src->keypoints->valid_key_points.at(i)->point->x;
		src_xyz_cloud->points[i].y = src->keypoints->valid_key_points.at(i)->point->y;
		src_xyz_cloud->points[i].z = src->keypoints->valid_key_points.at(i)->point->z;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	dst_xyz_cloud->width    = dst->keypoints->valid_key_points.size();
	dst_xyz_cloud->height   = 1;
	dst_xyz_cloud->points.resize (dst_xyz_cloud->width * dst_xyz_cloud->height);

	for(int i = 0; i < dst->keypoints->valid_key_points.size(); i++)
	{
		dst_xyz_cloud->points[i].x = dst->keypoints->valid_key_points.at(i)->point->x;
		dst_xyz_cloud->points[i].y = dst->keypoints->valid_key_points.at(i)->point->y;
		dst_xyz_cloud->points[i].z = dst->keypoints->valid_key_points.at(i)->point->z;
	}

	//pcl::PointCloud<pcl::PointXYZ> registration_output;
	DescriptorType type = src->keypoints->valid_key_points.at(0)->descriptor->type;
	if(type == surf64){
		pcl::PointCloud<Surf64PointType>::Ptr src_features (new pcl::PointCloud<Surf64PointType>);
		//pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features (new pcl::PointCloud<pcl::FPFHSignature33>);
		src_features->width    = src->keypoints->valid_key_points.size();
		src_features->height   = 1;
		src_features->points.resize (src_features->width);
		for(unsigned int i = 0; i < src->keypoints->valid_key_points.size(); i++)
		{
			Surf64PointType tmp = src_features->points[i];
			//pcl::FPFHSignature33 tmp = src_features->points[i];
			float * d1 = tmp.histogram;
			float * d2 = ((SurfFeatureDescriptor64 *)src->keypoints->valid_key_points.at(i)->descriptor)->descriptor;
			for(int j = 0; j < 33; j++){d1[j] = d2[j];}
			//printf("%i -> ",i);std::cout << tmp <<std::endl;
			src_features->points[i] = tmp;
		}
	
		pcl::PointCloud<Surf64PointType>::Ptr dst_features (new pcl::PointCloud<Surf64PointType>);
		//pcl::PointCloud<pcl::FPFHSignature33>::Ptr dst_features (new pcl::PointCloud<pcl::FPFHSignature33>);
		dst_features->width    = dst->keypoints->valid_key_points.size();
		dst_features->height   = 1;
		dst_features->points.resize (dst_features->width);
		for(unsigned int i = 0; i < dst->keypoints->valid_key_points.size(); i++)
		{
			Surf64PointType tmp = dst_features->points[i];
			//pcl::FPFHSignature33 tmp = dst_features->points[i];
			float * d1 = tmp.histogram;
			float * d2 = ((SurfFeatureDescriptor64 *)dst->keypoints->valid_key_points.at(i)->descriptor)->descriptor;
			for(int j = 0; j < 33; j++){d1[j] = d2[j];}
			dst_features->points[i] =tmp;
		}
		
		/*
		sac_ia_.setInputCloud(dst_xyz_cloud);
		sac_ia_.setInputTarget(src_xyz_cloud);
     	sac_ia_.setSourceFeatures(src_features);
     	sac_ia_.setTargetFeatures(dst_features);
		sac_ia_.align(registration_output);
		
		transformation->transformationMatrix = sac_ia_.getFinalTransformation();
		transformation->weight = 1/sac_ia_.getFitnessScore();
		*/
	}
/*
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
			dst_cloud->points[point_index].g = 255.0f;//dst->cloud->points[point_index].g;
			dst_cloud->points[point_index].b = 0.0f;//dst->cloud->points[point_index].b;
			
			point_index++;
		}
	}

printf("transformation->weight = %f\n",transformation->weight);
rotation = transformation->transformationMatrix.block<3,3>(0, 0);
translation = transformation->transformationMatrix.block<3,1>(0, 3);

printf ("\n");
printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
printf ("\n");
printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud (*dst_cloud, *transformed_cloud, transformation->transformationMatrix);

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	
	std::string text_cloud = "src";
	viewer.showCloud(src_cloud->makeShared(),text_cloud);
	
	text_cloud = "dst";
	//viewer.showCloud(dst_cloud->makeShared(),text_cloud);
	viewer.showCloud(transformed_cloud->makeShared(),text_cloud);
	
	while (!viewer.wasStopped ()){usleep(10000);}

	printf("done...\n");
	exit(0);
*/
	return transformation;
}
