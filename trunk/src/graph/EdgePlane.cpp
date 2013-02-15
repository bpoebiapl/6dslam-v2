
#include "edge_se3_quat.h"
#include "g2o/core/factory.h"

#include <iostream>
#include "EdgePlane.h"

namespace g2o {

  using namespace std;

  EdgePlane::EdgePlane() : BaseBinaryEdge<1, double , VertexPlane, VertexPlane>(){}

/*
  bool EdgePlane::setMeasurementFromState(){
  
    const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
    const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
    _measurement = (v1->estimate().inverse()*v2->estimate());
    _inverseMeasurement = _measurement.inverse();
    
    return true;
  }
*/
  bool EdgePlane::read(std::istream& is)
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

  bool EdgePlane::write(std::ostream& os) const
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
