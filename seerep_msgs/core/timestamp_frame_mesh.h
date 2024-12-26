#ifndef SEEREP_CORE_MSGS_TIMESTAMP_FRAME_MESH_H_
#define SEEREP_CORE_MSGS_TIMESTAMP_FRAME_MESH_H_

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include "timestamp.h"

namespace seerep_core_msgs
{

using ExactKernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGPoint_3 = CGAL::Point_3<ExactKernel>;
using SurfaceMesh = CGAL::Surface_mesh<ExactKernel::Point_3>;

struct TimestampFrameMesh
{
  seerep_core_msgs::Timestamp timestamp;
  std::string frame_id;
  SurfaceMesh mesh;
};
}  // namespace seerep_core_msgs

#endif
