#ifndef Map3DbaseGraph_H_
#define Map3DbaseGraph_H_

//other
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>

#include "RGBDFrame.h"
#include "Transformation.h" 

#include "FrameMatcher.h"
#include "FeatureExtractor.h"

#include "Map3D.h"

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

using namespace std;

class Map3DbaseGraph: public Map3D
{
	public:

	int max_backing;
	float w_limit;

	Map3DbaseGraph();
	~Map3DbaseGraph(); 
	
	void addFrame(Frame_input * fi);
	void addFrame(RGBDFrame * frame);
	void addTransformation(Transformation * transformation);
	
	void estimate();
	void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
	void visualize();
	
	g2o::VertexSE3 * getG2OVertex(RGBDFrame * frame);
	g2o::EdgeSE3 * getG2OEdge(Transformation * transformation);
	g2o::SparseOptimizer * startNewGraph();
	
	private:
	g2o::SparseOptimizer	graphoptimizer;
};
#endif
