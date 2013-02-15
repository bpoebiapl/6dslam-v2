#ifndef EdgePlane_H
#define EdgePlane_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/math_groups/se3quat.h"

#include "vertex_se3_quat.h"

namespace g2o {

  using namespace Eigen;
  
 class EdgePlane : public BaseBinaryEdge<1, double, VertexPlane, VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePlane();
    double angle;

    void computeError()
    {
		const VertexPlane* v1 = dynamic_cast<const VertexPlane*>(_vertices[0]);
		const VertexPlane* v2 = dynamic_cast<const VertexPlane*>(_vertices[1]);

		double rx1 = v1->rx;
		double ry1 = v1->ry;
		double rz1 = v1->rz;
		
		double rx2 = v2->rx;
		double ry2 = v2->ry;
		double rz2 = v2->rz;
		
		_error[0] = rx1*rx2+ry1*ry2+rz1*rz2;		
    }
    
    virtual void setMeasurement(double a){angle = a;}
    
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
