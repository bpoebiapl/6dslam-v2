#ifndef NDTMatcher_H_
#define NDTMatcher_H_
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
#include <pcl/filters/approximate_voxel_grid.h>

#include "FrameMatcher.h"

using namespace std;

class NDTMatcher: public FrameMatcher
{
	public:
		int nr_iters;
		float leafsize;
		float stepSize;
		float resolution;
		float epsilon;
		NDTMatcher();
		~NDTMatcher();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		void update();
};

#endif
