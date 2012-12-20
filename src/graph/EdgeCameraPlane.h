#ifndef EdgeCameraPlane_H
#define EdgeCameraPlane_H

#include "VertexPlane.h"
#include "VertexCamera.h"
#include "VertexPoint3D.h"
#include "g2o/core/base_binary_edge.h"

namespace g2o {
	class EdgeCameraPlane : public BaseBinaryEdge<2, Eigen::Vector2d, VertexCamera, VertexPlane>
	{
		public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeCameraPlane();
		void computeError()
		{
			//const VertexCamera * v1 = static_cast<const VertexCamera *>(_vertices[0]);
			//const VertexPlane  * l2 = static_cast<const VertexPlane  *>(_vertices[1]);
			//_error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

	};
} // end namespace

#endif
