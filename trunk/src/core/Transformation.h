#ifndef Transformation_H_
#define Transformation_H_
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

#include <pcl/visualization/cloud_viewer.h>

//other
#include <utility>
#include "RGBDFrame.h"
using namespace std;

class Transformation {
	public:
		RGBDFrame * src;
		RGBDFrame * dst;
		vector< pair <KeyPoint * ,KeyPoint * > > matches;
		Eigen::Matrix4f transformationMatrix;
		double weight;
		int level;
		g2o::EdgeSE3 * g2oEdge;
		
		void show(pcl::visualization::CloudViewer * viewer);
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
