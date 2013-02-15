#ifndef RGBDSegmentation_H_
#define RGBDSegmentation_H_

#include "../Frame_input.h"
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <string>
#include <iostream>
#include <stdio.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "mygeometry/mygeometry.h"

using namespace Eigen;
using namespace std;

struct segmentation_fit {
	//MyLinkedList<segment_pixel * > * segment;
	Matrix3f covMat;
	Matrix3f U;
	Vector3f S;
	Matrix3f V;
	Vector3f mean;
	int nr_valid;
	vector<int> * seg_w;
	vector<int> * seg_h;
};

class RGBDSegmentation
{
	public:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		Calibration * calibration;
		virtual ~RGBDSegmentation();
		virtual vector<Plane * > * segment(IplImage * rgb_img,IplImage * depth_img);
		void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
};

#include "RGBDSegmentationBase.h"
#include "RGBDSegmentationTest.h"
#include "RGBDSegmentationDummy.h"
#endif
