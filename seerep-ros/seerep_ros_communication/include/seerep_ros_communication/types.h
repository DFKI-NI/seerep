#ifndef SEEREP_GRPC_ROS_TYPES_H_
#define SEEREP_GRPC_ROS_TYPES_H_

// std
#include <string>
#include <algorithm>

// seerep
#include <seerep-com/transfer-sensor-msgs.grpc.pb.h>
#include <seerep-com/meta-operations.grpc.pb.h>
#include <seerep-com/image-service.grpc.pb.h>
#include <seerep-com/point-cloud-service.grpc.pb.h>
#include <seerep-com/tf-service.grpc.pb.h>

#include <seerep_ros_conversions_pb/conversions.h>

// Supported ROS message types
#define SEEREP_MESSAGE_TYPES(M)                                                                                        \
  SEEREP_M(std_msgs, Header), SEEREP_M(sensor_msgs, PointCloud2), SEEREP_M(sensor_msgs, Image),                        \
      SEEREP_M(geometry_msgs, Point), SEEREP_M(geometry_msgs, Quaternion), SEEREP_M(geometry_msgs, Pose),              \
      SEEREP_M(geometry_msgs, PoseStamped), SEEREP_M(tf2_msgs, TFMessage)

namespace seerep_grpc_ros
{
// Create enum types
#define SEEREP_M(p, t) p##_##t
enum SEEREP_MESSAGE_TYPE
{
  SEEREP_MESSAGE_TYPES(M),
  NumTypes
};
#undef SEEREP_M

#define PPCAT_NX(A, B) A##_##B
#define PPCAT(A, B) PPCAT_NX(A, B)
#define STRINGIZE_NX(A) #A
#define STRINGIZE(A) STRINGIZE_NX(A)

// Create the strings
#define STR_CONCAT (STR1 STR2
#define SEEREP_M(p, t) STRINGIZE(PPCAT(p, t))
std::string MessageTypeNames[] = { SEEREP_MESSAGE_TYPES(M) };
#undef SEEREP_M

using StubTransferPtr = std::unique_ptr<seerep::TransferSensorMsgs::Stub>;
using StubMetaPtr = std::unique_ptr<seerep::MetaOperations::Stub>;
using StubImagePtr = std::unique_ptr<seerep::ImageService::Stub>;
using StubPointCloudPtr = std::unique_ptr<seerep::PointCloudService::Stub>;
using StubTfPtr = std::unique_ptr<seerep::TfService::Stub>;

template <typename Type>
void send(StubTransferPtr& stub, grpc::ClientContext* c, seerep::ServerResponse* r, Type msg)
{
}
template <typename Type>
void send(StubImagePtr& stub, grpc::ClientContext* c, seerep::ServerResponse* r, Type msg)
{
}
template <typename Type>
void send(StubPointCloudPtr& stub, grpc::ClientContext* c, seerep::ServerResponse* r, Type msg)
{
}
template <typename Type>
void send(StubTfPtr& stub, grpc::ClientContext* c, seerep::ServerResponse* r, Type msg)
{
}

std::string names()
{
  std::string s;
  for (auto name : MessageTypeNames)
    s += name + ", ";
  return s;
}

SEEREP_MESSAGE_TYPE type(std::string name)
{
  std::replace(name.begin(), name.end(), '/', '_');
  return SEEREP_MESSAGE_TYPE(std::find(MessageTypeNames, MessageTypeNames + NumTypes, name) - MessageTypeNames);
}

std::string name(SEEREP_MESSAGE_TYPE type)
{
  return (type < NumTypes) ? MessageTypeNames[type] : "";
}

}  // namespace seerep_grpc_ros

#endif  // SEEREP_GRPC_ROS_TYPES_H_
