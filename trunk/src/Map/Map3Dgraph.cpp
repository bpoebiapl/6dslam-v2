
//Other
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/core/structure_only_solver.h"
#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

#include "Map3D.h"

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <Eigen/Core>
#include <fstream>

using namespace std;

Map3D::Map3D()
{
	
	graphoptimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graphoptimizer.setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graphoptimizer.setSolver(solver_ptr);

	matcher = 0;
	extractor = 0;
	
	printf("-->created Map3D<--\n");
}

Map3D::~Map3D(){}

void Map3D::addFrame(RGBDFrame * frame)
{
	Eigen::Affine3d eigenTransform(Eigen::Matrix4d::Identity());
	g2o::SE3Quat poseSE3(Eigen::Quaterniond(eigenTransform.rotation()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	g2o::VertexSE3 * vertexSE3 = new g2o::VertexSE3();
	vertexSE3->setId(frame->id);
	vertexSE3->estimate() = poseSE3;
	if(frame->id == 0){vertexSE3->setFixed(true);}
	graphoptimizer.addVertex(vertexSE3);
	frame->g2oVertex = vertexSE3;

	int nr_fails = 0;
	int nr_good = 0;
	for(int i = frames.size() - 1; i >= 0 && nr_fails < 10; i--){
		Transformation * trans = matcher->getTransformation(frame, frames.at(i));
		printf("%i <--> %i: match ->%f\n",frame->id,frames.at(i)->id,trans->weight);
		if(trans->weight >= 8)
		{
			frame->transformations.push_back(trans);
			frames.at(i)->transformations.push_back(trans);
			addTransformation(trans);
			nr_good++;
		}else{nr_fails++;}
	
	if(nr_good == 0){
		Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
		Eigen::Affine3d eigenTransform(transformationMatrix);
		g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenTransform.rotation()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
		g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;	
		edgeSE3->vertices()[0] = graphoptimizer.vertex(frame->id);
		edgeSE3->vertices()[1] = graphoptimizer.vertex(frame->id-1);
		edgeSE3->setMeasurement(transfoSE3.inverse());
		edgeSE3->setInverseMeasurement(transfoSE3);
		Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
		mat.setIdentity(6,6);
		edgeSE3->information() = mat;
		graphoptimizer.addEdge(edgeSE3);};
	}

	frames.push_back(frame);
	//estimate();
	
}

g2o::SparseOptimizer * Map3D::startNewGraph()
{
	g2o::SparseOptimizer * graph = new g2o::SparseOptimizer;
	graph->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	graph->setVerbose(false);
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&graphoptimizer,linearSolver);
	graph->setSolver(solver_ptr);
	return graph;
}

g2o::VertexSE3 * Map3D::getG2OVertex(RGBDFrame * frame)
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

g2o::EdgeSE3 * Map3D::getG2OEdge(Transformation * transformation)
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


void Map3D::addTransformation(Transformation * transformation)
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	for(int i = 0; i < 4 ; i++)
	{
		for(int j = 0; j < 4 ; j++)
		{
			transformationMatrix(i,j) = transformation->transformationMatrix(i,j); 
		}
	}
	

	Eigen::Affine3d eigenTransform(transformationMatrix);
	g2o::SE3Quat transfoSE3(Eigen::Quaterniond(eigenTransform.rotation()),Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
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

void Map3D::estimate()
{
	graphoptimizer.initializeOptimization();
	graphoptimizer.setVerbose(true);
	graphoptimizer.optimize(50);
	printf("map_x = [ 0");
	for(int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(i));
		Eigen::Matrix4f matrix = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		printf(" %f",matrix(0,3));
	}
	printf("];\n");
	
	printf("map_y = [");
	for(int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(i));
		Eigen::Matrix4f matrix = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		printf(" %f",matrix(1,3));
	}
	printf("];\n");
	
	printf("map_z = [");
	for(int i  = 0; i < frames.size(); i++){
		g2o::VertexSE3 * vertexSE3_src = (g2o::VertexSE3*)(graphoptimizer.vertex(i));
		Eigen::Matrix4f matrix = (vertexSE3_src->estimate().to_homogenious_matrix()).cast<float>();
		printf(" %f",matrix(2,3));
	}
	printf("];\n");
}

void Map3D::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){
	viewer = view;
	//matcher->setVisualization(view);
}

