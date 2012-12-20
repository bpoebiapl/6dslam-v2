#include "VertexPlane.h"
#include <typeinfo>

namespace g2o {

	VertexPlane::VertexPlane() : BaseVertex<4, Vector4f>(){_estimate.setZero();}

	bool VertexPlane::read(std::istream& is)
	{
		is >> estimate()[0] >> estimate()[1] >> estimate()[2] >> estimate()[3];
		return true;
	}

	bool VertexPlane::write(std::ostream& os) const
	{
		os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2) << " " << estimate()(3);
		return os.good();
	}

}
