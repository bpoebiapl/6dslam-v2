#ifndef SACIAMatcher_H_
#define SACIAMatcher_H_
//OpenCV
#include "cv.h"
#include "highgui.h"
#include <string>
#include <iostream>
#include <stdio.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/Histogram.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include "FeatureDescriptor.h"

using namespace std;

class SACIAMatcher: public FrameMatcher
{
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
		typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
		typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
		typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;
		
		PointCloudXYZ::Ptr xyz_;
		SurfaceNormals::Ptr normals_;
		LocalFeatures::Ptr features_;
		SearchMethod::Ptr search_method_xyz_;
		float minSampleDistance;
		float maxCorrespondenceDistance;
	
		SACIAMatcher();
		~SACIAMatcher();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		void update();
};

#endif
