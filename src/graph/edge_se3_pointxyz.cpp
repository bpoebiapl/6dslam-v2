#include "edge_se3_pointxyz.h"

namespace g2o {
  namespace tutorial {

    EdgeSE3PointXYZ::EdgeSE3PointXYZ() :
      BaseBinaryEdge<3, Vector3d, VertexSE3, VertexPointXYZ>()
    {
    }

    bool EdgeSE3PointXYZ::read(std::istream& is)
    {
      //is >> measurement()[0] >> measurement()[1];
      //inverseMeasurement() = measurement() * -1;
      //is >> information()(0,0) >> information()(0,1) >> information()(1,1);
      //information()(1,0) = information()(0,1);
      return true;
    }

    bool EdgeSE3PointXYZ::write(std::ostream& os) const
    {
      //os << measurement()[0] << " " << measurement()[1] << " ";
      //os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
      return os.good();
    }

} // end namespace
} // end namespace
