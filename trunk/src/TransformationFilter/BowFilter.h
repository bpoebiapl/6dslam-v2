#ifndef BowFilter_H_
#define BowFilter_H_
//OpenCV
/*
#include "cv.h"
#include "highgui.h"

#include <string>
#include <iostream>
#include <stdio.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "RGBDFrame.h"
#include "Transformation.h"
*/
#include "TransformationFilter.h"

using namespace std;

class BowFilter: public TransformationFilter
{
	public:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		string name;
		float feature_threshold;
		int max_points;
		float stabilety_threshold;
		BowFilter();
		~BowFilter();
		Transformation * filterTransformation(Transformation * input);
		void print();
		void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
};

#include "BowFilter.h"

#endif
