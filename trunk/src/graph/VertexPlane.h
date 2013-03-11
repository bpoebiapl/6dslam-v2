
#ifndef VertexPlane_H
#define VertexPlane_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/math_groups/se3quat.h"

namespace g2o {
class VertexPlane : public BaseVertex<6, SE3Quat>
{
  public:
  	float rx;
  	float ry;
  	float rz;
  	
  	float px;
  	float py;
  	float pz;
  	
  	float d;
  	int id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPlane();

    virtual void setToOrigin() {_estimate = SE3Quat() ;}

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;


    virtual bool setEstimateData(const double* est){
      Vector7d v;
      for (int i=0; i<7; i++)
	v[i]=est[i];
      _estimate.fromVector(v);
      return true;
    }

    virtual bool getEstimateData(double* est) const{
      Vector7d v=_estimate.toVector();
      for (int i=0; i<7; i++)
	est[i] = v[i];
      return true;
    }

    virtual int estimateDimension() const {return 7;}

    virtual bool setMinimalEstimateData(const double* est){
      Map<const Vector6d> v(est);
      _estimate.fromMinimalVector(v);
      return true;
    }

    virtual bool getMinimalEstimateData(double* est) const{
      Map<Vector6d> v(est);
      v = _estimate.toMinimalVector();
      return true;
    }

    virtual int minimalEstimateDimension() const {return 6;}

    virtual void oplus(double* update)
    {
    	/*

		d 	= rx*px + ry*py + rz*pz;
		*/
		rx += update[0];
		ry += update[1];
		rz += update[2];
		
		px += update[3];
		py += update[4];
		pz += update[5];
		
		double div = sqrt(rx*rx+ry*ry+rz*rz);
		rx /= div;
		ry /= div;
		rz /= div;
		
		/*
		d += update[3];
		px = rx*d;
		py = ry*d;
		pz = rz*d;
		*/
	
    }
};
}

#endif
