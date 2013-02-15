#ifndef RGBDFrame_H_
#define RGBDFrame_H_

//g2o
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/core/structure_only_solver.h"
#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

//OpenCV
#include "cv.h"
#include "highgui.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <stdio.h>
#include <string.h>

#include "KeyPointSet.h"
#include "Plane.h"
#include "Point.h"
#include "SurfExtractor.h"

#include "RGBDSegmentation.h"
#include "FloatHistogramFeatureDescriptor.h"

using namespace std;
class RGBDFrame
{

	public:

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	Frame_input * input;
	int id;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
	int width;
	int height;
	
	vector<segmentation_fit * > * segments;
	char ** segmentation;

	FeatureDescriptor * image_descriptor;
	
	KeyPointSet * keypoints;
	
	vector<Plane *> * planes;
	
	vector<float *> validation_points;
	
	Eigen::Matrix4f	matrix;
	g2o::VertexSE3 * g2oVertex;
	bool locked;
	
	void init_pointcloud(IplImage* rgb_img, IplImage* depth_img);
	void init_normals();
	void init_jointcloudnormals(IplImage* rgb_img, IplImage* depth_img);
	void init_segmentation(IplImage* rgb_img,IplImage* depth_img);
	void init_filter();
	RGBDFrame();
	RGBDFrame(Frame_input * fi, FeatureExtractor * extractor, RGBDSegmentation * segmenter);
	//RGBDFrame(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud);
	RGBDFrame(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud, vector<FeatureDescriptor *> * centers);
	~RGBDFrame();
	
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	//private:
	// Point cloud data
	PointCloudXYZ::Ptr xyz_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;

};
#endif
