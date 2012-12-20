#ifndef Map3D_H_
#define Map3D_H_

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

//other
#include <stdio.h>
#include <string.h>
#include "RGBDFrame.h"

#include "FrameMatcher.h"
#include "FeatureExtractor.h"

#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>

#include "Transformation.h" 

//#include "RGBDSegmentation.h"

using namespace std;
class RGBDFrame;
class Transformation;
class FrameMatcher;

class Map3D
{
	public:
	
	vector<RGBDFrame *> frames;
	FrameMatcher * matcher;
	FeatureExtractor * extractor;
	//RGBDSegmentation * seg;
	
	Map3D();
	~Map3D(); 
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	void addFrame(RGBDFrame * frame);
	void addTransformation(Transformation * transformation);
	
	g2o::VertexSE3 * getG2OVertex(RGBDFrame * frame);
	g2o::EdgeSE3 * getG2OEdge(Transformation * transformation);
	g2o::SparseOptimizer * startNewGraph();
	
	void estimate();
	void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
	
	private:
	g2o::SparseOptimizer	graphoptimizer;
};
#endif
