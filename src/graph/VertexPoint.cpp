#include "VertexPoint.h"
#include "g2o/core/factory.h"

#include <iostream>

namespace g2o {
int VertexPointCounter = 0;
	VertexPoint::VertexPoint() : BaseVertex<6, SE3Quat>(){
		x=0;
		y=1;
		z=2;
		id=VertexPointCounter++;
	}

  bool VertexPoint::read(std::istream& is)
  {
  /*
    for (int i=0; i<4; i++)
      is  >> estimate()[i];
    estimate().rotation().normalize();
    */
    return true;
  }

  bool VertexPoint::write(std::ostream& os) const
  {
  /*
    for (int i=0; i<4; i++)
      os << estimate()[i] << " ";
  */
    return os.good();
  }
}
