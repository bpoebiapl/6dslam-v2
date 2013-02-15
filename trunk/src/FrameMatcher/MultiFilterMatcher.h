#ifndef MultiFilterMatcher_H_
#define MultiFilterMatcher_H_
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
#include "TransformationFilter/TransformationFilter.h"

using namespace std;

class MultiFilterMatcher: public FrameMatcher
{
	public:
		vector<TransformationFilter *> filters;
		
		void addFilter(TransformationFilter * f);
		MultiFilterMatcher();
		~MultiFilterMatcher();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		void update();
};

#endif
