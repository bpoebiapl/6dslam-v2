#ifndef EdgeSe3Plane_H
#define EdgeSe3Plane_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/math_groups/se3quat.h"

#include "vertex_se3_quat.h"

namespace g2o {

  using namespace Eigen;
  
 class EdgeSe3Plane : public BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSe3Plane();
    void computeError()
    {
      const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
      SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
      _error.head<3>() = delta.translation();
      // The analytic Jacobians assume the error in this special form (w beeing positive)
      if (delta.rotation().w() < 0.)
        _error.tail<3>() =  - delta.rotation().vec();
      else
        _error.tail<3>() =  delta.rotation().vec();
    }

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

    // virtual bool setMeasurementFromState() ;

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    // virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
};
} // end namespace

#endif
