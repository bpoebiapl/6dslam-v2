
#include "edge_se3_quat.h"
#include "g2o/core/factory.h"

#include <iostream>
#include "EdgeSe3Plane2.h"

namespace g2o {

  using namespace std;

  EdgeSe3Plane2::EdgeSe3Plane2() : BaseBinaryEdge<4, Eigen::Matrix4d , VertexSE3, VertexPlane>(){}

/*
  bool EdgeSe3Plane2::setMeasurementFromState(){
  
    const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
    const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
    _measurement = (v1->estimate().inverse()*v2->estimate());
    _inverseMeasurement = _measurement.inverse();
    
    return true;
  }
*/
  bool EdgeSe3Plane2::read(std::istream& is)
  {
  	/*
    for (int i=0; i<7; i++)
      is >> measurement()[i];
    measurement().rotation().normalize();
    inverseMeasurement() = measurement().inverse();
    
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i) = information()(i,j);
      }
*/
    return true;
  }

  bool EdgeSe3Plane2::write(std::ostream& os) const
  {
  /*
    for (int i=0; i<7; i++)
      os << measurement()[i] << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
        os << " " <<  information()(i,j);
      }
     */
    return os.good();
  }
}
