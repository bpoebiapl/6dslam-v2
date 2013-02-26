#ifndef DistanceNetMatcherv2_H_
#define DistanceNetMatcherv2_H_
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

struct linkData {
	float distance;
	int id1;
	int id2;
};

struct linkPair {
	linkData * src_link;
	linkData * dst_link;
	float value;
	float * p_src;
	float * p_dst;
	float * c_i;
	float * c_j;
};



using namespace std;

class DistanceNetMatcherv2: public FrameMatcher
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
		
		//float p1_rejection;
		//float p2_rejection;
		//float likelihood_rejection;
		
		bool movement_prior;
		float movementPerFrame;
				
		bool feature_prior;
		float feature_smoothing;
		
		DistanceNetMatcherv2(int iter_, int max_points_, float std_dist_, float bounds_, bool movement_prior_, float movementPerFrame_, bool feature_prior_, float feature_smoothing_);
		
		DistanceNetMatcherv2();
		~DistanceNetMatcherv2();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
};

#endif
