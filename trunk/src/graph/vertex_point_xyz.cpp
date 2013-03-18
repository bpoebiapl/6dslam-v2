#include "vertex_point_xyz.h"
#include <typeinfo>

namespace g2o {
    VertexPointXYZ::VertexPointXYZ() :
      BaseVertex<3, Vector3d>()
    {
      _estimate.setZero();
    }

    bool VertexPointXYZ::read(std::istream& is)
    {
      is >> estimate()[0] >> estimate()[1] >> estimate()[2];
      return true;
    }

    bool VertexPointXYZ::write(std::ostream& os) const
    {
      os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2);
      return os.good();
    }
} // end namespace
