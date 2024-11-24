#ifndef SEEREP_CORE_MSGS_TIMESTAMP_FRAME_POINTS_H_
#define SEEREP_CORE_MSGS_TIMESTAMP_FRAME_POINTS_H_

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include "timestamp.h"

namespace seerep_core_msgs
{

using ExactKernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGPoint_3 = CGAL::Point_3<ExactKernel>;
using SurfaceMesh = CGAL::Surface_mesh<ExactKernel::Point_3>;

struct TimestampFramePoints
{
  seerep_core_msgs::Timestamp timestamp;
  SurfaceMesh mesh;
  std::string frame_id;
};
}  // namespace seerep_core_msgs

#endif
