#include "VertexPlane.h"
#include "g2o/core/factory.h"

#include <iostream>

namespace g2o {
int VertexPlaneCounter = 0;
	VertexPlane::VertexPlane() : BaseVertex<6, SE3Quat>(){
		rx=0;
		ry=0;
		rz=1;
		d=0;
		px = 0;
		py = 0;
		pz = 0;
		id=VertexPlaneCounter++;
	}

  bool VertexPlane::read(std::istream& is)
  {
  /*
    for (int i=0; i<4; i++)
      is  >> estimate()[i];
    estimate().rotation().normalize();
    */
    return true;
  }

  bool VertexPlane::write(std::ostream& os) const
  {
  /*
    for (int i=0; i<4; i++)
      os << estimate()[i] << " ";
  */
    return os.good();
  }
}
