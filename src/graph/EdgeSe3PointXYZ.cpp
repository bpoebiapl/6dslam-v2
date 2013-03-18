
#include "edge_se3_quat.h"
#include "g2o/core/factory.h"

#include <iostream>
#include "EdgeSe3PointXYZ.h"

namespace g2o {

  using namespace std;

  EdgeSe3PointXYZ::EdgeSe3PointXYZ() : BaseBinaryEdge<4, Eigen::Matrix4d , VertexSE3, VertexPoint>(){}

  bool EdgeSe3PointXYZ::read(std::istream& is){return true;}

  bool EdgeSe3PointXYZ::write(std::ostream& os) const{return os.good();}
}
