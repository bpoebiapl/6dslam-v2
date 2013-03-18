#ifndef edge_se3_point3d_H
#define edge_se3_point3d_H

#include "vertex_se3.h"
#include "VertexPoint3D.h"
#include "g2o/core/base_binary_edge.h"
	class EdgeSE3Point3D : public BaseBinaryEdge<3, Eigen::Vector3d, VertexSE3, VertexPoint3D>{
		public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSE3Point3D();

		void computeError(){
			const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
			const VertexPoint3D* l2 = static_cast<const VertexPoint3D*>(_vertices[1]);
			_error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
		}
		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;
	};
#endif
