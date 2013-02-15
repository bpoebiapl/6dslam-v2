#ifndef FilterMatcher_H_
#define FilterMatcher_H_
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

class FilterMatcher: public FrameMatcher
{
	public:
		FrameMatcher * fm;
		vector<TransformationFilter *> filters;
		vector<float> thresholds;
		
		void addFilter(TransformationFilter * f);
		FilterMatcher();
		~FilterMatcher();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		void update();
};

#endif
