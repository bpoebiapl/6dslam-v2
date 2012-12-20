#ifndef BasicIcpMatcher_H_
#define BasicIcpMatcher_H_
//OpenCV
#include "cv.h"
#include "highgui.h"
#include <string>
#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>

#include "FrameMatcher.h"

using namespace std;

class BasicIcpMatcher: public FrameMatcher
{
	public:
		int nr_iters;
		float correspondenceDistance;
		float rejectionThreshold;
		//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		BasicIcpMatcher();
		~BasicIcpMatcher();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		void update();
};

#endif
