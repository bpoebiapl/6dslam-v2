#ifndef DistanceNetMatcherv5_H_
#define DistanceNetMatcherv5_H_
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

class DistanceNetMatcherv5: public FrameMatcher
{
	public:
		int max_points;
		int nr_iter;
		float std_dist;
		int lookup_size;
		float bounds;
		float probIgnore;

		float * lookup;
		float lookup_step;
		
		bool movement_prior;
		float movementPerFrame;
				
		bool feature_prior;
		float feature_smoothing;
		
		DistanceNetMatcherv5(int iter_, int max_points_, float std_dist_, float bounds_, bool movement_prior_, float movementPerFrame_, bool feature_prior_, float feature_smoothing_);
		
		DistanceNetMatcherv5();
		~DistanceNetMatcherv5();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
};

#endif
