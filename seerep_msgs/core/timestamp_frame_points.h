#ifndef SEEREP_CORE_MSGS_TIMESTAMP_FRAME_POINTS_H_
#define SEEREP_CORE_MSGS_TIMESTAMP_FRAME_POINTS_H_

#include "aabb.h"
#include "timestamp.h"

namespace seerep_core_msgs
{
struct TimestampFramePoints
{
  seerep_core_msgs::Timestamp timestamp;
  std::array<seerep_core_msgs::Point, 5> points;
  std::string frame_id;
};
}  // namespace seerep_core_msgs

#endif
