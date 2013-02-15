#ifndef UpdateWeightFilterv2_H_
#define UpdateWeightFilterv2_H_
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

class UpdateWeightFilterv2: public TransformationFilter
{
	public:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		unsigned short * depth_data;
		string name;
		float limit;
		UpdateWeightFilterv2();
		~UpdateWeightFilterv2();
		Transformation * filterTransformation(Transformation * input);
		void print();
		void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
};

#include "UpdateWeightFilterv2.h"

#endif
