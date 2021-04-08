#ifndef SEEREP_GRPC_ROS_SERVER_H_
#define SEEREP_GRPC_ROS_SERVER_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// ag
#include <seerep_msgs/transfer_sensor_msgs.grpc.pb.h>
#include <seerep_ros_conversions/conversions.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace seerep_grpc_ros
{
class ReceiveSensorMsgs final : public ag::TransferSensorMsgs::Service
{
public:
  ReceiveSensorMsgs();
  grpc::Status TransferPointCloud2(grpc::ServerContext* context, const ag::PointCloud2* point_cloud_2, ag::ServerResponse* response);
  grpc::Status TransferImage(grpc::ServerContext* context, const ag::Image* image, ag::ServerResponse* response);
  grpc::Status TransferHeader(grpc::ServerContext* context, const ag::Header* header, ag::ServerResponse* response);
};

std::shared_ptr<grpc::Server> createServer(const std::string& server_address, seerep_grpc_ros::ReceiveSensorMsgs* receive_sensor_msgs);

} /* namespace seerep_grpc_ros */

#endif // SEEREP_GRPC_ROS_SERVER_H_
