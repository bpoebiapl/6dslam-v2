#ifndef VertexCamera_H
#define VertexCamera_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {
	using namespace Eigen;
	typedef Matrix<float, 6, 1> Vector6f;
	class VertexCamera : public BaseVertex<6, Vector6f>
	{
		public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		VertexCamera();

		virtual void setToOrigin() {
			_estimate.setZero();
		}

		virtual void oplus(double* update)
		{
			_estimate[0] += update[0];
			_estimate[1] += update[1];
			_estimate[2] += update[2];
			_estimate[3] += update[3];
			_estimate[4] += update[4];
			_estimate[5] += update[5];
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

	};
}

#endif
