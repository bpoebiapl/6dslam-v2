#include "edge_se3_point3d.h"
	EdgeSE3Point3D::EdgeSE3Point3D() : BaseBinaryEdge<3, Vector3d, VertexSE3, VertexPoint3D>(){}
	bool EdgeSE3Point3D::read(std::istream& is){
		//is >> measurement()[0] >> measurement()[1];
		//inverseMeasurement() = measurement() * -1;
		//is >> information()(0,0) >> information()(0,1) >> information()(1,1);
		//information()(1,0) = information()(0,1);
		return true;
	}

	bool EdgeSE2PointXY::write(std::ostream& os) const{
		//os << measurement()[0] << " " << measurement()[1] << " ";
		//os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
		return os.good();
	}
