#ifndef BasicGIcpMatcher_H_
#define BasicGIcpMatcher_H_
//OpenCV
#include "cv.h"
#include "highgui.h"
#include <string>
#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>

#include "FrameMatcher.h"

using namespace std;

class BasicGIcpMatcher: public FrameMatcher
{
	public:
		int nr_iters;
		float correspondenceDistance;
		float rejectionThreshold;
		//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		BasicGIcpMatcher();
		~BasicGIcpMatcher();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		void update();
};

#endif
