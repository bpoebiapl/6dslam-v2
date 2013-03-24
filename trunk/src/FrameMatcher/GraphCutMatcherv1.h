#ifndef GraphCutMatcherv1_H_
#define GraphCutMatcherv1_H_
//OpenCV
#include "cv.h"
#include "highgui.h"
#include <string>
#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/thread/thread.hpp>

#include "FrameMatcher.h"

using namespace std;

class GraphCutMatcherv1: public FrameMatcher
{
	public:
		//GraphCutMatcherv1();
		float feature_limit;
		float point_to_point_zero;
		float point_to_plane_zero;
		float plane_to_plane_zero;
		GraphCutMatcherv1();
		~GraphCutMatcherv1();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
};

#endif
