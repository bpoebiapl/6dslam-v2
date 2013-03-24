#ifndef FrameMatcher_H_
#define FrameMatcher_H_
//OpenCV
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

using namespace std;

//class RGBDFrame;
//class Transformation;

class FrameMatcher
{
	public:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		string name;
		virtual ~FrameMatcher();
		virtual Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		virtual void print();
		void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
};


#include "AICK.h"
#include "BowAICK.h"
#include "AICKv2.h"
#include "BasicGIcpMatcher.h"
#include "BasicIcpMatcher.h"
#include "dnet.h"
#include "NDTMatcher.h"
#include "RansacPCLCopyMatcher.h"
#include "RansacPCLCopySlowMatcher.h"
#include "SACIAMatcher.h"
#include "MultiLayerMatcher.h"
#include "MultiFilterMatcher.h"
#include "FilterMatcher.h"
#include "DistanceNetMatcher.h"
#include "DistanceNetMatcherv2.h"
#include "DistanceNetMatcherv3.h"
#include "DistanceNetMatcherv4.h"
#include "DistanceNetMatcherv5.h"
#include "GraphCutMatcherv1.h"

#endif
