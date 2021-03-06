#include "Map3DbaseGraph.h"
#include "Frame_input.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

bool comparison_Map3DbaseGraph (Transformation * i,Transformation * j) {
	if(i->src == j->src){return (i->weight<j->weight);}
	else{return (i->src->id<j->src->id);}
}

using namespace std;

Map3DbaseGraph::Map3DbaseGraph(){
	max_backing = 10;
	w_limit = 0.7;
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);
}

Map3DbaseGraph::~Map3DbaseGraph(){}

g2o::SparseOptimizer * Map3DbaseGraph::startNewGraph()
{
	g2o::SparseOptimizer * graph = new g2o::SparseOptimizer;
	graph->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graph->setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graph->setSolver(solver_ptr);
	return graph;
}

g2o::VertexSE3 * Map3DbaseGraph::getG2OVertex(RGBDFrame * frame)
{
	Eigen::Affine3f eigenTransform(frame->matrix);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
	g2o::SE3Quat poseSE3(
		Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
		Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	g2o::VertexSE3 *vertexSE3 = new g2o::VertexSE3();
	vertexSE3->setId(frame->id);
	vertexSE3->estimate() = poseSE3;
	return vertexSE3;
}

g2o::EdgeSE3 * Map3DbaseGraph::getG2OEdge(Transformation * transformation)
{
	Eigen::Affine3f eigenTransform(transformation->transformationMatrix);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
	g2o::SE3Quat transfoSE3(
			Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
			Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;	
	edgeSE3->vertices()[0] = graphoptimizer.vertex(transformation->src->id);
	edgeSE3->vertices()[1] = graphoptimizer.vertex(transformation->dst->id);
	edgeSE3->setMeasurement(transfoSE3.inverse());
	edgeSE3->setInverseMeasurement(transfoSE3);
	Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
	mat.setIdentity(6,6);
	edgeSE3->information() = mat;
	return edgeSE3;
}

void Map3DbaseGraph::addFrame(Frame_input * fi){addFrame(new RGBDFrame(fi,extractor,segmentation));}
void Map3DbaseGraph::addFrame(RGBDFrame * frame){
	Matrix4f pose = Matrix4f::Identity();

	//printf("--------------------------------%i--------------------------------\n",frames.size());
	float best = -1;
	Transformation * best_t = 0;
	
	IplImage* depth_img = cvLoadImage(frame->input->depth_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	unsigned short * depth_data	= (unsigned short *)depth_img->imageData;
	UpdateWeightFilter * update = new UpdateWeightFilter();
	update->depth_data = depth_data;
	
	for(int i = frames.size(); (i > 0) && ((frames.size() - i) < max_backing); i--){
		Transformation * t = matcher->getTransformation(frame, frames.at(i-1));
		Transformation * t_update = update->filterTransformation(t);
		t->weight = t_update->weight;
		delete t_update;
		if(t->weight > best){
			pose = poses.at(i-1)*t->transformationMatrix;
			best = t->weight;
			best_t = t;
		}
		if(t->weight > w_limit){transformations.push_back(t);}
	}
	
	if(best <= w_limit && best_t != 0){transformations.push_back(best_t);}
	
	poses.push_back(pose);
	frames.push_back(frame);

	int stop = frames.size()-max_backing-1;
	FeatureDescriptor * frame_desc = frame->image_descriptor;
	float img_threshold = 0.0045;
	
	for(int i = 0; i < stop; i++){
		RGBDFrame * test = frames.at(i);
		float d = test->image_descriptor->distance(frame_desc);
		if(d<img_threshold)
		{
			Transformation * t = loopclosure_matcher->getTransformation(frame, test);
			if(t->weight > 20){
				Transformation * t_update = update->filterTransformation(t);
				t->weight = t_update->weight;
				delete t_update;
				if(t->weight > w_limit)	{transformations.push_back(t);}
				else					{delete t;}
			}else{delete t;}
			
		}
	}
	delete update;
	cvReleaseImage( &depth_img );
	
}
void Map3DbaseGraph::addTransformation(Transformation * transformation){transformations.push_back(transformation);}
void Map3DbaseGraph::estimate(){
	printf("estimate\n");
	printf("nr frames: %i nr trans: %i\n",frames.size(),transformations.size());
	for(int i  = 0; i < frames.size(); i++){
		Eigen::Affine3f eigenTransform(poses.at(i));
		Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
		g2o::SE3Quat poseSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
		g2o::VertexSE3 * vertexSE3 = new g2o::VertexSE3();
		vertexSE3->setId(frames.at(i)->id);
		vertexSE3->estimate() = poseSE3;
		graphoptimizer.addVertex(vertexSE3);
		frames.at(i)->g2oVertex = vertexSE3;
		if(i == 0){vertexSE3->setFixed(true);}
	}
	
	for(int i  = 0; i < transformations.size(); i++){
		Transformation * transformation = transformations.at(i);
		Eigen::Affine3f eigenTransform(transformation->transformationMatrix);
		Eigen::Quaternionf eigenRotation(eigenTransform.rotation());
		g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
		g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;	
		edgeSE3->vertices()[0] = graphoptimizer.vertex(transformation->src->id);
		edgeSE3->vertices()[1] = graphoptimizer.vertex(transformation->dst->id);
		edgeSE3->setMeasurement(transfoSE3.inverse());
		edgeSE3->setInverseMeasurement(transfoSE3);
		Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
		mat.setIdentity(6,6);
		edgeSE3->information() = mat;
		graphoptimizer.addEdge(edgeSE3);
	}
	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(50);
	for(int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(frames.at(i)->id));
		poses.at(i) = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		//cout<<i<<endl<<poses.at(i)<<endl;
	}
	printf("estimate done\n");
}
void Map3DbaseGraph::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}

//bool goToNext = false;
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void){goToNext = true;}

void Map3DbaseGraph::visualize(){
/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = poses.size();
	cloud->height   = 1;
	cloud->points.resize (cloud->width);
	cloud->width = 0;
	for(int i = 0; i < poses.size(); i+=1){
		int randr = 0;rand()%256;
		int randg = 255;rand()%256;
		int randb = 0;rand()%256;
		Eigen::Matrix4f m1 = poses.at(i);
		cloud->points[cloud->width].x = m1(0,3);
		cloud->points[cloud->width].y = m1(1,3);
		cloud->points[cloud->width].z = m1(2,3);
		//printf("%f %f %f\n",cloud->points[cloud->width].x,cloud->points[cloud->width].y,cloud->points[cloud->width].z);
		cloud->points[cloud->width].r = randr;
		cloud->points[cloud->width].g = randg;
		cloud->points[cloud->width].b = randb;
		cloud->width++;
	}
*/

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width    = 0;
	cloud->height   = 1;
	for(int i = 0; i < poses.size(); i+=10){cloud->width+=frames.at(i)->xyz_->width;}
	cloud->points.resize (cloud->width);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width    = 0;
	for(int i = 0; i < poses.size(); i+=40){
		pcl::transformPointCloud (*(frames.at(i)->xyz_), *tmp_cloud, poses.at(i));
		int randr = 0;rand()%256;
		int randg = 255;rand()%256;
		int randb = 0;rand()%256;
		for(int j = 0; j < tmp_cloud->width*tmp_cloud->height; j++)
		{
			cloud->points[cloud->width].x = tmp_cloud->points[j].x;
			cloud->points[cloud->width].y = tmp_cloud->points[j].y;
			cloud->points[cloud->width].z = tmp_cloud->points[j].z;
			cloud->points[cloud->width].r = randr;
			cloud->points[cloud->width].g = randg;
			cloud->points[cloud->width].b = randb;
			cloud->width++;
		}
	}
	
	//viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->removePointCloud("sample cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
		
	usleep(100);
	while (true){
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	
}

