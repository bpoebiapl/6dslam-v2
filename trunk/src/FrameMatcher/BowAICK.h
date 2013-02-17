#ifndef BowAICK_H_
#define BowAICK_H_
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

class BowAICK: public FrameMatcher
{
	public:
		int nr_iter;
		float feature_scale;
		float distance_threshold;
		float feature_threshold;
		float bow_threshold;
		float shrinking;
		float stabilety_threshold;
		int max_points;
		BowAICK();
		BowAICK(int max_points_);
		~BowAICK();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		float getAlpha(int iteration);
};

#endif
