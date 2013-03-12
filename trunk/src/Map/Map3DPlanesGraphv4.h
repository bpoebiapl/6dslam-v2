#ifndef Map3DPlanesGraphv4_H_
#define Map3DPlanesGraphv4_H_

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

#include "GraphForCut.h"

using namespace std;

class Map3DPlanesGraphv4: public Map3D
{
	public:

	float img_threshold;
	int estimateIter;
	int max_backing;
	
	float w_limit_close;
	float w_limit_loop;
	float match_limit_close;
	float match_limit_loop;
	
	float smoothing;
	bool render_full;
	
	int plane_segment_id_counter;
	map< Plane *, int > map_id_plane;
	vector< vector< pair <RGBDFrame * , Plane * > > > mergedPlanes;

	vector< vector<Transformation *> * > * transformations_mat;
	
	Map3DPlanesGraphv4();
	~Map3DPlanesGraphv4(); 
	
	void addFrame(Frame_input * fi);
	void addFrame(RGBDFrame * frame);
	void addTransformation(Transformation * transformation);
	vector< GraphEdge > * match_planes(RGBDFrame * src, RGBDFrame * dst, Eigen::Matrix4f trans);
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
