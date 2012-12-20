#include "VertexPoint3D.h"
#include <typeinfo>

namespace g2o {

	VertexPoint3D::VertexPoint3D() : BaseVertex<3, Vector3f>(){_estimate.setZero();}

	bool VertexPoint3D::read(std::istream& is)
	{
		is >> estimate()[0] >> estimate()[1] >> estimate()[2];
		return true;
	}

	bool VertexPoint3D::write(std::ostream& os) const
	{
		os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2);
		return os.good();
	}

}
