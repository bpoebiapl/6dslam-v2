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
    double d;
    void computeError()
    {
		const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
		const VertexPlane* v2 = dynamic_cast<const VertexPlane*>(_vertices[1]);
		Matrix4d mat2 = v1->estimate().to_homogenious_matrix();
		Matrix4d mat = mat2.inverse();
		
		Vector4d pos = Vector4d(0,0,0,1);
		
		double rx = v2->rx;
		double ry = v2->ry;
		double rz = v2->rz;
		
		double px = v2->px;
		double py = v2->py;
		double pz = v2->pz;
		
		double x = mat2(0,3);
		double y = mat2(1,3);
		double z = mat2(2,3);
		
		double tmp_nx = rx*mat(0,0) + ry*mat(0,1) + rz*mat(0,2);
		double tmp_ny = rx*mat(1,0) + ry*mat(1,1) + rz*mat(1,2);
		double tmp_nz = rx*mat(2,0) + ry*mat(2,1) + rz*mat(2,2);
		
		double tmp_d = rx*(px-x) + ry*(py-y) + rz*(pz-z);
		float diff = fabs(tmp_d-d);
		diff-= 0.015f;
		if(diff < 0){diff = 0;}
		diff = diff*diff*diff;
		
		//if((v2->id == 0) && (rand()%100 == 0)){printf("%i [%.5f,%.5f]\n",v2->id,tmp_d,d);}
		
		_error[0] = 10000.0f*fabs(tmp_nx-nx);
		_error[1] = 10000.0f*fabs(tmp_ny-ny);
		_error[2] = 10000.0f*fabs(tmp_nz-nz);
		_error[3] = diff;
    }

	void setMeasurement(Plane * p){
		nx = p->normal_x;
		ny = p->normal_y;
		nz = p->normal_z;
		d = p->distance(0,0,0);
	}
	
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
};
}

#endif
