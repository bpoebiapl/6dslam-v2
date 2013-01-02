#ifndef FeatureExtractor_H_
#define FeatureExtractor_H_

//OpenCV
#include "cv.h"
#include "highgui.h"
#include "KeyPointSet.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include "../Frame_input.h"

//bool comparison_f (KeyPoint * i,KeyPoint * j) { return (i->stabilety>j->stabilety); }

class Calibration;
class FeatureExtractor
{
	public:
		Calibration * calibration;
		virtual KeyPointSet * getKeyPointSet(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud);
		virtual KeyPointSet * getKeyPointSet(IplImage * rgb_img,IplImage * depth_img);
		FeatureExtractor();
		virtual ~FeatureExtractor();
};

#include "FPFHExtractor.h"
#include "OrbExtractor.h"
#include "OwnFeatureExtractor.h"
#include "SurfExtractor.h"
#endif
