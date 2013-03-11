#ifndef EdgeSe3Plane2_H
#define EdgeSe3Plane2_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/math_groups/se3quat.h"

#include "vertex_se3_quat.h"

namespace g2o {

  using namespace Eigen;
  
 class EdgeSe3Plane2 : public BaseBinaryEdge<4, Eigen::Matrix4d, VertexSE3, VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSe3Plane2();
    double nx;
    double ny;
    double nz;
    double px;
    double py;
    double pz;
    void computeError()
    {
		const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
		const VertexPlane* v2 = dynamic_cast<const VertexPlane*>(_vertices[1]);
		Matrix4d mat2 = v1->estimate().to_homogenious_matrix();
		Matrix4d mat = mat2.inverse();

		double rx = v2->rx;
		double ry = v2->ry;
		double rz = v2->rz;
		
		double p2x = v2->px;
		double p2y = v2->py;
		double p2z = v2->pz;

		double tmp_nx = nx*mat2(0,0) + ny*mat2(0,1) + nz*mat2(0,2);
		double tmp_ny = nx*mat2(1,0) + ny*mat2(1,1) + nz*mat2(1,2);
		double tmp_nz = nx*mat2(2,0) + ny*mat2(2,1) + nz*mat2(2,2);
		
		double tmp_px = px*mat2(0,0) + py*mat2(0,1) + pz*mat2(0,2)+mat2(0,3);
		double tmp_py = px*mat2(1,0) + py*mat2(1,1) + pz*mat2(1,2)+mat2(1,3);
		double tmp_pz = px*mat2(2,0) + py*mat2(2,1) + pz*mat2(2,2)+mat2(2,3);
		
		double tmp = rx*(p2x-tmp_px) + ry*(p2y-tmp_py) + rz*(p2z-tmp_pz);
		_error[0] = 1.0f*fabs(tmp_nx-rx);
		_error[1] = 1.0f*fabs(tmp_ny-ry);
		_error[2] = 1.0f*fabs(tmp_nz-rz);
		_error[3] = 0;//0.01f*fabs(tmp);
    }

	void setMeasurement(Plane * p){
		nx = p->normal_x;
		ny = p->normal_y;
		nz = p->normal_z;
		px = p->point_x;
		py = p->point_y;
		pz = p->point_z;
	}
	
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
};
}

#endif
