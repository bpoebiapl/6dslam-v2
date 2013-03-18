
#include "Transformation.h"

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/core/structure_only_solver.h"
#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

//#include "edge_se3_pointxyz.h"
//#include "vertex_point_xyz.h"
#include "VertexPlane.h"
#include "EdgeSe3Plane.h"
#include "EdgeSe3Plane2.h"
#include "EdgePlane.h"
#include "VertexPoint.cpp"
#include "EdgeSe3PointXYZ.cpp"



#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>
#include <fstream>

void Transformation::show(){
	int max_points = 300;
	IplImage* rgb_img_src 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_src = (char *)rgb_img_src->imageData;
	IplImage* rgb_img_dst 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_dst = (char *)rgb_img_dst->imageData;
		
	int width = rgb_img_src->width;
	int height = rgb_img_src->height;
		
	IplImage* img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
	char * data = (char *)img_combine->imageData;
		
	vector<KeyPoint * > src_keypoints;//	= src->keypoints->valid_key_points;
	int nr_loop_src = src->keypoints->valid_key_points.size();
	if(nr_loop_src > max_points){nr_loop_src = max_points;}
	int nr_loop_dst = dst->keypoints->valid_key_points.size();
	if(nr_loop_dst > max_points){nr_loop_dst = max_points;}
	
	for(int i = 0; i < nr_loop_src; i++)
	{
		src_keypoints.push_back(src->keypoints->valid_key_points.at(i));
	}

	vector<KeyPoint * > dst_keypoints;//	= dst->keypoints->valid_key_points;
	for(int i = 0; i < nr_loop_dst; i++)
	{
		dst_keypoints.push_back(dst->keypoints->valid_key_points.at(i));
	}

	int src_nr_points = src_keypoints.size();
	int dst_nr_points = dst_keypoints.size();

	int index = 0;
		
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			
			int ind = 3*(640*j+i);
			data[3 * (j * (2*width) + (width+i)) + 0] = data_dst[ind +0];
			data[3 * (j * (2*width) + (width+i)) + 1] = data_dst[ind +1];
			data[3 * (j * (2*width) + (width+i)) + 2] = data_dst[ind +2];

			data[3 * (j * (2*width) + (i)) + 0] = data_src[ind +0];
			data[3 * (j * (2*width) + (i)) + 1] = data_src[ind +1];
			data[3 * (j * (2*width) + (i)) + 2] = data_src[ind +2];
			
		}
	}
	
	for(int i = 0; i < dst_nr_points;i++){
		cvCircle(img_combine,cvPoint(dst_keypoints.at(i)->point->w + width	, dst_keypoints.at(i)->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);}
	for(int i = 0; i < src_nr_points;i++){
		cvCircle(img_combine,cvPoint(src_keypoints.at(i)->point->w			, src_keypoints.at(i)->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);}
	
	//cvNamedWindow("src image", CV_WINDOW_AUTOSIZE );
	//cvShowImage("src image", rgb_img_src);
		
	//cvNamedWindow("dst image", CV_WINDOW_AUTOSIZE );
	//cvShowImage("dst image", rgb_img_dst);
	for(int i = 0; i < matches.size();i++){
		KeyPoint * src_kp = matches.at(i).first;
		KeyPoint * dst_kp = matches.at(i).second;
		cvCircle(img_combine,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		cvCircle(img_combine,cvPoint(src_kp->point->w			, src_kp->point->h), 5,cvScalar(0, 255, 0, 0),2, 8, 0);
		cvLine(img_combine,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
	}
		
	cvNamedWindow("combined image", CV_WINDOW_AUTOSIZE );
	cvShowImage("combined image", img_combine);
		
	cvWaitKey(0);
	cvReleaseImage( &rgb_img_src );
	cvReleaseImage( &rgb_img_dst );
}

void Transformation::show(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer){

	g2o::SparseOptimizer	graphoptimizer;
	graphoptimizer.clear();
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
	
	Eigen::Affine3f src_eigenTransform(Eigen::Matrix4f::Identity());
	Eigen::Quaternionf src_eigenRotation(src_eigenTransform.rotation());
	g2o::SE3Quat src_poseSE3(Eigen::Quaterniond(src_eigenRotation.w(), src_eigenRotation.x(), src_eigenRotation.y(), src_eigenRotation.z()),Eigen::Vector3d(src_eigenTransform(0,3), src_eigenTransform(1,3), src_eigenTransform(2,3)));
	g2o::VertexSE3 * src_vertexSE3 = new g2o::VertexSE3();
	src_vertexSE3->setId(0);
	src_vertexSE3->estimate() = src_poseSE3;
	graphoptimizer.addVertex(src_vertexSE3);
	
	Eigen::Affine3f dst_eigenTransform(Eigen::Matrix4f::Identity());
	Eigen::Quaternionf dst_eigenRotation(dst_eigenTransform.rotation());
	g2o::SE3Quat dst_poseSE3(Eigen::Quaterniond(dst_eigenRotation.w(), dst_eigenRotation.x(), dst_eigenRotation.y(), dst_eigenRotation.z()),Eigen::Vector3d(dst_eigenTransform(0,3), dst_eigenTransform(1,3), dst_eigenTransform(2,3)));
	g2o::VertexSE3 * dst_vertexSE3 = new g2o::VertexSE3();
	dst_vertexSE3->setId(1);
	dst_vertexSE3->estimate() = dst_poseSE3;
	graphoptimizer.addVertex(dst_vertexSE3);
	
	for(int i = 0; i < matches.size(); i++){
		KeyPoint * src_kp = matches.at(i).first;
		KeyPoint * dst_kp = matches.at(i).second;
		
		g2o::VertexPoint * match_vertex = new g2o::VertexPoint();
		match_vertex->setId(2+i);
		match_vertex->x = src_kp->point->x;
		match_vertex->y = src_kp->point->y;
		match_vertex->z = src_kp->point->z;
		graphoptimizer.addVertex(match_vertex);
		
		g2o::EdgeSe3PointXYZ * src_edge = new g2o::EdgeSe3PointXYZ();
		src_edge->vertices()[0] = src_vertexSE3;
		src_edge->vertices()[1] = match_vertex;
		src_edge->vp = match_vertex;
		src_edge->information() = Matrix4d::Identity();
		src_edge->setMeasurement(src_kp->point);
		graphoptimizer.addEdge(src_edge);
		
		g2o::EdgeSe3PointXYZ * dst_edge = new g2o::EdgeSe3PointXYZ();
		dst_edge->vertices()[0] = dst_vertexSE3;
		dst_edge->vertices()[1] = match_vertex;
		dst_edge->vp = match_vertex;
		dst_edge->information() = Matrix4d::Identity();
		dst_edge->setMeasurement(dst_kp->point);
		graphoptimizer.addEdge(dst_edge);
/*		
		g2o::tutorial::EdgeSE3PointXYZ * src_edge = new g2o::tutorial::EdgeSE3PointXYZ();
		src_edge->vertices()[0] = src_vertexSE3;
		src_edge->vertices()[1] = match_vertex;
		
		src_edge->information() = Matrix3d::Identity();
		//src_edge->_measurement[0] = src_kp->point->x;
		//src_edge->_measurement[1] = src_kp->point->y;
		//src_edge->_measurement[2] = src_kp->point->z;
		
		//graphoptimizer.addEdge(src_edge);
			
		g2o::tutorial::EdgeSE3PointXYZ * dst_edge = new g2o::tutorial::EdgeSE3PointXYZ();
		dst_edge->vertices()[0] = dst_vertexSE3;
		dst_edge->vertices()[1] = match_vertex;
		dst_edge->information() = Matrix3d::Identity();
		dst_edge->setMeasurement(dst_kp->point->pos);
		graphoptimizer.addEdge(dst_edge);
	*/
	/*
		Eigen::Affine3f eigenTransform(transformation->transformationMatrix);
		Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
		g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
		g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;
		edgeSE3->vertices()[0] = frames.at(transformation->src->id)->g2oVertex;
		edgeSE3->vertices()[1] = frames.at(transformation->dst->id)->g2oVertex;
		edgeSE3->setMeasurement(transfoSE3.inverse());
		edgeSE3->setInverseMeasurement(transfoSE3);
		Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
		mat.setIdentity(6,6);
		edgeSE3->information() = mat;
		graphoptimizer.addEdge(edgeSE3);
	*/
	
	}

	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(10000);
	
	Eigen::Matrix4f src_mat = (src_vertexSE3->estimate().to_homogenious_matrix()).cast<float>();
	Eigen::Matrix4f dst_mat = (dst_vertexSE3->estimate().to_homogenious_matrix()).cast<float>();
	
	Eigen::Matrix4f trans_mat_est = dst_mat.inverse()*src_mat;
	
	Eigen::Matrix4f inv = transformationMatrix.inverse();
	
	std::cout << "trans_mat_est\n" << trans_mat_est << std::endl;
	std::cout << "trans_mat_pcl_inv\n" << inv << std::endl;
	std::cout << "trans_mat_pcl\n" << transformationMatrix << std::endl;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpcloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

	IplImage* src_rgb_img 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	IplImage* src_depth_img = cvLoadImage(src->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		
	IplImage* dst_rgb_img 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	IplImage* dst_depth_img = cvLoadImage(dst->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
		
	float d_scaleing	= src->input->calibration->ds/src->input->calibration->scale;
	float centerX		= src->input->calibration->cx;
	float centerY		= src->input->calibration->cy;
	float invFocalX		= 1.0f/src->input->calibration->fx;
	float invFocalY		= 1.0f/src->input->calibration->fy;
		
	char * src_rgb_data		= (char *)src_rgb_img->imageData;
	char * dst_rgb_data		= (char *)dst_rgb_img->imageData;
	unsigned short * src_depth_data	= (unsigned short *)src_depth_img->imageData;
	unsigned short * dst_depth_data	= (unsigned short *)dst_depth_img->imageData;
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	src_cloud->width    = 0;
	src_cloud->height   = 1;
	src_cloud->points.resize (2*640*480);

	for(int w = 0; w < 640; w+=1){
		for(int h = 0; h < 480; h+=1){
			int ind = 640*h+w;
			float x = 0;
			float y = 0;
			float z = float(src_depth_data[ind]) * d_scaleing;
			
			int r = 0;//char(src_rgb_data[3*ind+2]);
			int g = 255;//char(src_rgb_data[3*ind+1]);
			int b = 0;//char(src_rgb_data[3*ind+0]);
			
			if(r < 0){r = 255+r;}
			if(g < 0){g = 255+g;}
			if(b < 0){b = 255+b;}
				
			if(z > 0 && z < 3.0f){
				x = (w - centerX) * z * invFocalX;
			   	y = (h - centerY) * z * invFocalY;
				   	
			   	src_cloud->points[src_cloud->width].x = x;
				src_cloud->points[src_cloud->width].y = y;
				src_cloud->points[src_cloud->width].z = z;
				src_cloud->points[src_cloud->width].r = float(r);
				src_cloud->points[src_cloud->width].g = float(g);
				src_cloud->points[src_cloud->width].b = float(b);
				src_cloud->width++;
			}
		}
	}
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	dst_cloud->width    = 0;
	dst_cloud->height   = 1;
	dst_cloud->points.resize (640*480);
	for(int w = 0; w < 640; w+=1){
		for(int h = 0; h < 480; h+=1){
			int ind = 640*h+w;
			float x = 0;
			float y = 0;
			float z = float(dst_depth_data[ind]) * d_scaleing;
			
			int r = 255;//char(dst_rgb_data[3*ind+2]);
			int g = 0;//char(dst_rgb_data[3*ind+1]);
			int b = 0;//char(dst_rgb_data[3*ind+0]);
			
			if(r < 0){r = 255+r;}
			if(g < 0){g = 255+g;}
			if(b < 0){b = 255+b;}
				
			if(z > 0 && z < 3.0f){
				x = (w - centerX) * z * invFocalX;
			   	y = (h - centerY) * z * invFocalY;
			   	
			   	dst_cloud->points[dst_cloud->width].x = x;
				dst_cloud->points[dst_cloud->width].y = y;
				dst_cloud->points[dst_cloud->width].z = z;
				dst_cloud->points[dst_cloud->width].r = float(r);
				dst_cloud->points[dst_cloud->width].g = float(g);
				dst_cloud->points[dst_cloud->width].b = float(b);
				dst_cloud->width++;
			}
		}
	}
	dst_cloud->points.resize (dst_cloud->width);
		
	cvReleaseImage( &src_rgb_img );
	cvReleaseImage( &src_depth_img );

	cvReleaseImage( &dst_rgb_img );
	cvReleaseImage( &dst_depth_img );
		
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
		

		
	pcl::transformPointCloud (*dst_cloud, *dst_cloud_tmp, inv);		
	src_cloud->points.resize (src_cloud->width+dst_cloud_tmp->width);
	for(int j = 0; j < dst_cloud_tmp->width; j++){
		src_cloud->points[src_cloud->width].x = dst_cloud_tmp->points[j].x;
		src_cloud->points[src_cloud->width].y = dst_cloud_tmp->points[j].y;
		src_cloud->points[src_cloud->width].z = dst_cloud_tmp->points[j].z;
		src_cloud->points[src_cloud->width].r = dst_cloud_tmp->points[j].r;
		src_cloud->points[src_cloud->width].g = dst_cloud_tmp->points[j].g;
		src_cloud->points[src_cloud->width].b = dst_cloud_tmp->points[j].b;
		src_cloud->width++;
	}

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "test");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(src_cloud);
	viewer->removePointCloud("test");
	viewer->addPointCloud<pcl::PointXYZRGB> (src_cloud, rgb, "test");
	usleep(100);
	for(int i = 0; i < 50; i++)///while (true)
	{
		printf("i:%i\n",i);
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	viewer->removePointCloud("test");
	
	/*
	viewer->showCloud(transformed_cloud->makeShared(),text_cloud);
	while (!viewer->wasStopped ()){usleep(10000);}
	*/
};
