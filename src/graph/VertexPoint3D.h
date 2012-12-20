#ifndef VertexPoint3D_H
#define VertexPoint3D_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {
	class VertexPoint3D : public BaseVertex<3, Eigen::Vector3f>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		VertexPoint3D();

		virtual void setToOrigin() {
			_estimate.setZero();
		}

		virtual void oplus(double* update)
		{
			_estimate[0] += update[0];
			_estimate[1] += update[1];
			_estimate[2] += update[2];
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

	};
}

#endif
