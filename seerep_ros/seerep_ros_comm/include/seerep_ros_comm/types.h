#ifndef SEEREP_ROS_COMM_TYPES_H_
#define SEEREP_ROS_COMM_TYPES_H_

// std
#include <algorithm>
#include <string>

// seerep-pb
#include <seerep_com/image_service.grpc.pb.h>
#include <seerep_com/meta_operations.grpc.pb.h>
#include <seerep_com/point_cloud_service.grpc.pb.h>
#include <seerep_com/tf_service.grpc.pb.h>

// seerep-fb
#include <seerep_com/image_service.grpc.fb.h>
#include <seerep_com/meta_operations.grpc.fb.h>
#include <seerep_com/tf_service.grpc.fb.h>

// seerep_com
#include <seerep_ros_conversions_pb/conversions.h>

// Supported ROS message types
#define SEEREP_MESSAGE_TYPES(M)                                                \
  SEEREP_M(std_msgs, Header), SEEREP_M(sensor_msgs, PointCloud2),              \
      SEEREP_M(sensor_msgs, Image), SEEREP_M(geometry_msgs, Point),            \
      SEEREP_M(geometry_msgs, Quaternion), SEEREP_M(geometry_msgs, Pose),      \
      SEEREP_M(geometry_msgs, PoseStamped), SEEREP_M(tf2_msgs, TFMessage)

namespace seerep_ros_comm
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

using StubMetaPbPtr = std::unique_ptr<seerep::pb::MetaOperations::Stub>;
using StubImagePbPtr = std::unique_ptr<seerep::pb::ImageService::Stub>;
using StubPointCloudPbPtr =
    std::unique_ptr<seerep::pb::PointCloudService::Stub>;
using StubTfPbPtr = std::unique_ptr<seerep::pb::TfService::Stub>;

template <typename Type>
void send(StubImagePbPtr& stub, grpc::ClientContext* c,
          seerep::pb::ServerResponse* r, Type msg)
{
}
template <typename Type>
void send(StubPointCloudPbPtr& stub, grpc::ClientContext* c,
          seerep::pb::ServerResponse* r, Type msg)
{
}
template <typename Type>
void send(StubTfPbPtr& stub, grpc::ClientContext* c,
          seerep::pb::ServerResponse* r, Type msg)
{
}

using StubMetaFbPtr = std::unique_ptr<seerep::fb::MetaOperations::Stub>;
using StubImageFbPtr = std::unique_ptr<seerep::fb::ImageService::Stub>;
using StubTfFbPtr = std::unique_ptr<seerep::fb::TfService::Stub>;

template <typename Type>
void send(StubImageFbPtr& stub, grpc::ClientContext* c,
          seerep::pb::ServerResponse* r, Type msg)
{
}
template <typename Type>
void send(StubTfFbPtr& stub, grpc::ClientContext* c,
          seerep::pb::ServerResponse* r, Type msg)
{
}

std::string names()
{
  std::string s;
  for (auto name : MessageTypeNames)
  {
    s += name + ", ";
  }
  return s;
}

SEEREP_MESSAGE_TYPE type(std::string name)
{
  std::replace(name.begin(), name.end(), '/', '_');
  return SEEREP_MESSAGE_TYPE(
      std::find(MessageTypeNames, MessageTypeNames + NumTypes, name) -
      MessageTypeNames);
}

std::string name(SEEREP_MESSAGE_TYPE type)
{
  return (type < NumTypes) ? MessageTypeNames[type] : "";
}

}  // namespace seerep_ros_comm

#endif  // SEEREP_ROS_COMM_TYPES_H_
