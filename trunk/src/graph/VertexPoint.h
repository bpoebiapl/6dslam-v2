
#ifndef VertexPoint_H
#define VertexPoint_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/math_groups/se3quat.h"

namespace g2o {
class VertexPoint : public BaseVertex<6, SE3Quat>
{
  public:
  	double x;
  	double y;
  	double z;
  	
  	int id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPoint();

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
		if(!std::isnan(update[0])){x += update[0];}
		if(!std::isnan(update[1])){y += update[1];}
		if(!std::isnan(update[2])){z += update[2];}
		//printf("Update: %10.10f %10.10f %10.10f\n",update[0],update[1],update[2]);
		//printf("oplus: %f %f %f\n",x,y,z);
    }
};
}

#endif
