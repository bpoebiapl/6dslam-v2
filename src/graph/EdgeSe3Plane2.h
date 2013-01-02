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
		Matrix4d mat = v1->estimate().to_homogenious_matrix().inverse();
		double rx = v2->rx;
		double ry = v2->ry;
		double rz = v2->rz;
		
		double tmp_nx = rx*mat(0,0) + ry*mat(0,1) + rz*mat(0,2);
		double tmp_ny = rx*mat(1,0) + ry*mat(1,1) + rz*mat(1,2);
		double tmp_nz = rx*mat(2,0) + ry*mat(2,1) + rz*mat(2,2);
		
		//printf("%i: [%.5f,%.5f,%.5f] -> [%.5f,%.5f,%.5f]\n",v2->id,nx,ny,nz,tmp_nx,tmp_ny,tmp_nz);
		
		_error[0] = fabs(tmp_nx-nx);
		_error[1] = fabs(tmp_ny-ny);
		_error[2] = fabs(tmp_nz-nz);
		_error[3] = 0;
		
    }
    virtual void setMeasurement(Plane * p){
    	nx = p->normal_x;
    	ny = p->normal_y;
    	nz = p->normal_z;
    	d = p->distance(0,0,0);
    }
/*
    virtual void setMeasurement(const SE3Quat& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* d){
      Vector7d v;
      v.setZero();
      for (int i=0; i<6; i++)
	v[i]=d[i];
      _measurement.fromVector(v);
      _inverseMeasurement = _measurement.inverse();
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Vector7d v=_measurement.toVector();
      for (int i=0; i<7; i++)
	d[i]=v[i];
      return true;
    }
    
    virtual int measurementDimension() const {return 7;}
*/
    // virtual bool setMeasurementFromState() ;

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    // virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
};
} // end namespace

#endif
