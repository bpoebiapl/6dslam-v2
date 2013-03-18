#ifndef EdgeSe3PointXYZ_H
#define EdgeSe3PointXYZ_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/math_groups/se3quat.h"

#include "vertex_se3_quat.h"

namespace g2o {

  using namespace Eigen;
  
 class EdgeSe3PointXYZ : public BaseBinaryEdge<4, Eigen::Matrix4d, VertexSE3, VertexPoint>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSe3PointXYZ();

	VertexPoint *  vp;

	double x;
	double y;
	double z;

    void computeError(){
		const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);

		Matrix4d mat = v1->estimate().to_homogenious_matrix();
		
		double vx = vp->x;
		double vy = vp->y;
		double vz = vp->z;
		
		double tmp_x = vx*mat(0,0) + vy*mat(0,1) + vz*mat(0,2)+mat(0,3);
		double tmp_y = vx*mat(1,0) + vy*mat(1,1) + vz*mat(1,2)+mat(1,3);
		double tmp_z = vx*mat(2,0) + vy*mat(2,1) + vz*mat(2,2)+mat(2,3);

		_error[0] = 1.0f*fabs(tmp_x-x);
		_error[1] = 1.0f*fabs(tmp_y-y);
		_error[2] = 1.0f*fabs(tmp_z-z);
		_error[3] = 0;//0.01f*fabs(tmp);
    }

	void setMeasurement(Point * p){
		x = p->x;
		y = p->y;
		z = p->z;
	}

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
};
}

#endif
