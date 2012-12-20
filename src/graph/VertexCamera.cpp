#include "VertexCamera.h"
#include <typeinfo>

namespace g2o {

	VertexCamera::VertexCamera() : BaseVertex<6, Vector6f>(){_estimate.setZero();}

	bool VertexCamera::read(std::istream& is){
		is >> estimate()[0] >> estimate()[1] >> estimate()[2] >> estimate()[3] >> estimate()[4] >> estimate()[5];
		return true;
	}

	bool VertexCamera::write(std::ostream& os) const
	{
		os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2) << " " << estimate()(3) << estimate()(4) << " " << estimate()(5);
		return os.good();
	}
}
