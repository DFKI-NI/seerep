#include "seerep-server/server.h"

namespace seerep_server
{
ReceiveSensorMsgs::ReceiveSensorMsgs(std::string datafolder) : projectOverview(datafolder), datafolder(datafolder)
{
}

grpc::Status ReceiveSensorMsgs::TransferHeader(grpc::ServerContext* context, const seerep::Header* header,
                                               seerep::ServerResponse* response)
{
  std::cout << "received header... " << std::endl;
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferImage(grpc::ServerContext* context, const seerep::Image* image,
                                              seerep::ServerResponse* response)
{
  std::cout << "received image... " << std::endl;
  // hdf5_io.writeImage("test_id", *image);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPointCloud2(grpc::ServerContext* context,
                                                    const seerep::PointCloud2* point_cloud_2,
                                                    seerep::ServerResponse* response)
{
  std::cout << "received point clouds... " << std::endl;
  // TODO implement hdf5_io function
  // hdf5_io.writePointCloud2("test_id", *point_cloud_2);

  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid = gen(projectOverview.newProject("testproject"));

  projectOverview.addPointCloud(*point_cloud_2, uuid);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPointCloud2Labeled(grpc::ServerContext* context,
                                                           const seerep::PointCloud2Labeled* point_cloud_2_labeled,
                                                           seerep::ServerResponse* response)
{
  std::cout << "received point clouds... " << std::endl;
  // TODO implement hdf5_io function
  // hdf5_io.writePointCloud2("test_id", *point_cloud_2);
  boost::uuids::uuid uuid;
  projectOverview.addPointCloudLabeled(*point_cloud_2_labeled, uuid);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::GetPointCloud2(grpc::ServerContext* context, const seerep::Boundingbox* request,
                                               grpc::ServerWriter<seerep::PointCloud2>* writer)
{
  std::cout << "sending point cloud in bounding box min(" << request->point_min().x() << "/" << request->point_min().y()
            << "/" << request->point_min().z() << "), max(" << request->point_max().x() << "/"
            << request->point_max().y() << "/" << request->point_max().z() << ")" << std::endl;

  std::vector<std::vector<std::optional<seerep::PointCloud2>>> pointclouds = projectOverview.getPointCloud(*request);
  if (!pointclouds.empty())
  {
    std::cout << "Found pointclouds in " << pointclouds.size() << " projects that match the query" << std::endl;

    for (const std::vector<std::optional<seerep::PointCloud2>>& resultPerProject : pointclouds)
    {
      std::cout << "Found " << resultPerProject.size() << " pointclouds in this projects that match the query"
                << std::endl;
      for (const std::optional<seerep::PointCloud2>& pc : resultPerProject)
      {
        writer->Write(pc.value());
      }
    }
  }
  else
  {
    std::cout << "Found NOTHING that matches the query" << std::endl;
  }
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPoint(grpc::ServerContext* context, const seerep::Point* point,
                                              seerep::ServerResponse* response)
{
  std::cout << "received point... " << std::endl;
  // hdf5_io.writePoint("test_id", *point);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                                   seerep::ServerResponse* response)
{
  std::cout << "received quaternion... " << std::endl;
  // hdf5_io.writeQuaternion("test_id", *quaternion);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPose(grpc::ServerContext* context, const seerep::Pose* pose,
                                             seerep::ServerResponse* response)
{
  std::cout << "received pose... " << std::endl;
  // hdf5_io.writePose("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                                    seerep::ServerResponse* response)
{
  std::cout << "received pose_stamped... " << std::endl;
  // hdf5_io.writePoseStamped("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::CreateProject(grpc::ServerContext* context, const seerep::ProjectCreation* request,
                                              seerep::ProjectCreated* response)
{
  std::cout << "create new project... " << std::endl;
  response->set_uuid(projectOverview.newProject(request->name()));

  return grpc::Status::OK;
}

std::shared_ptr<grpc::Server> createServer(const std::string& server_address,
                                           seerep_server::ReceiveSensorMsgs* receive_sensor_msgs)
{
  std::cout << "Create the server..." << std::endl;
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(receive_sensor_msgs);
  return std::shared_ptr<grpc::Server>(server_builder.BuildAndStart());
}

} /* namespace seerep_server */

int main(int argc, char** argv)
{
  std::string datafolder;
  if (argc == 2)
  {
    datafolder = argv[1];
    // append '/' if not path does not end with it
    if (!datafolder.empty() && datafolder.back() != '/')
    {
      datafolder += '/';
    }
  }
  else
  {
    datafolder = std::filesystem::current_path();
  }
  std::cout << "The used data folder is: " << datafolder << std::endl;
  std::string server_address = "localhost:9090";
  // HighFive::File hdf5_file("test.h5", HighFive::File::ReadWrite | HighFive::File::Create);
  // HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  seerep_server::ReceiveSensorMsgs receive_sensor_msgs_service(datafolder);
  std::shared_ptr<grpc::Server> server = seerep_server::createServer(server_address, &receive_sensor_msgs_service);
  std::cout << "serving on \"" << server_address << "\"..." << std::endl;
  server->Wait();
  return EXIT_SUCCESS;
}
