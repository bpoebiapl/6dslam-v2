#ifndef edge_se3_pointxyz_H
#define edge_se3_pointxyz_H

#include "vertex_se3_quat.h"
#include "vertex_point_xyz.h"
#include "g2o/core/base_binary_edge.h"
namespace g2o {
	namespace tutorial {
	class EdgeSE3PointXYZ : public BaseBinaryEdge<3, Eigen::Vector3d, VertexSE3, VertexPointXYZ>{
		public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSE3PointXYZ();

		void computeError(){
			//const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
			//const VertexPointXYZ* l2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
			//_error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
			//_error[0] = 0;
			//_error[1] = 0;
			//_error[2] = 0;
		}
		void setMeasurement(double x, double y, double z){}
		/*
		void setMeasurement(double x, double y, double z){
			_measurement[0] = x;
			_measurement[1] = y;
			_measurement[2] = z;
			_inverseMeasurement = _measurement.inverse();
		}
		*/
		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;
	};
}
}
#endif
