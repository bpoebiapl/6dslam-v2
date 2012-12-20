#include "EdgeCameraPlane.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace g2o {
	EdgeCameraPlane::EdgeCameraPlane() : BaseBinaryEdge<2, Vector2d, VertexCamera, VertexPlane>(){}

	bool EdgeCameraPlane::read(std::istream& is)
	{
		//is >> measurement()[0] >> measurement()[1];
		//inverseMeasurement() = measurement() * -1;
		//is >> information()(0,0) >> information()(0,1) >> information()(1,1);
		//information()(1,0) = information()(0,1);
		return true;
	}

	bool EdgeCameraPlane::write(std::ostream& os) const
	{
		//os << measurement()[0] << " " << measurement()[1] << " ";
		//os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
		//return os.good();
		return true;
	}

}
