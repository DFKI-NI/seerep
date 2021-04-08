#include "ag_grpc_ros/server.h"

namespace ag_grpc_ros
{
ReceiveSensorMsgs::ReceiveSensorMsgs() {}

grpc::Status ReceiveSensorMsgs::TransferPointCloud2(
    grpc::ServerContext* context,
    const ag::PointCloud2* msg,
    ag::ServerResponse* response)
{
  sensor_msgs::PointCloud2 cloud = ag_proto_ros::toROS(*msg);
  ROS_INFO_STREAM("Incoming PointCloud2 message" << std::endl << cloud);
  response->set_message("okidoki");
  response->set_transmission_state(ag::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferHeader(
    grpc::ServerContext* context,
    const ag::Header* msg,
    ag::ServerResponse* response)
{
  std_msgs::Header header = ag_proto_ros::toROS(*msg);
  ROS_INFO_STREAM("Incoming Header message" << std::endl << header);
  response->set_message("okidoki");
  response->set_transmission_state(ag::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferImage(
    grpc::ServerContext* context,
    const ag::Image* msg,
    ag::ServerResponse* response)
{
  sensor_msgs::Image image = ag_proto_ros::toROS(*msg);
  ROS_INFO_STREAM("Incoming Image message" << std::endl << image);
  response->set_message("okidoki");
  response->set_transmission_state(ag::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

std::shared_ptr<grpc::Server> createServer(
    const std::string& server_address,
    ag_grpc_ros::ReceiveSensorMsgs* receive_sensor_msgs)
{
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(receive_sensor_msgs);
  return std::shared_ptr<grpc::Server>(server_builder.BuildAndStart());
}

} /* namespace ag_grpc_ros */


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ag_grpc_ros_server");
  ros::NodeHandle private_nh("~");

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  ag_grpc_ros::ReceiveSensorMsgs receive_sensor_msgs_service;
  std::shared_ptr<grpc::Server> server = ag_grpc_ros::createServer(server_address, &receive_sensor_msgs_service);

  ROS_INFO_STREAM("Server listening on " << server_address);

  ros::spin();
  server->Shutdown();

  return EXIT_SUCCESS;
}
