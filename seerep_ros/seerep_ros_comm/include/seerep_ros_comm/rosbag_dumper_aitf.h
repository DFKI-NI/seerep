#ifndef SEEREP_ROS_COMM_ROSBAG_DUMPER_AITF_H_
#define SEEREP_ROS_COMM_ROSBAG_DUMPER_AITF_H_

#include <seerep_hdf5_core/hdf5_core_cameraintrinsics.h>
#include <seerep_hdf5_core/hdf5_core_general.h>
#include <seerep_hdf5_core/hdf5_core_image.h>
#include <seerep_hdf5_ros/hdf5_ros.h>

#include <filesystem>
#include <fstream>

// ros
#include <ros/console.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_ros_comm
{
class RosbagDumperAitf
{
public:
  RosbagDumperAitf(const std::string& bagPath, const std::string& hdf5FilePath,
                   const std::string& projectFrameId,
                   const std::string& projectName,
                   const std::string& topicImage, const std::string& topicPc2,
                   const std::string& topicCameraIntrinsics,
                   const std::string& topicTf, const std::string& topicTfStatic,
                   double maxViewingDistance);
  ~RosbagDumperAitf();

private:
  std::string getCameraIntrinsic(const std::string& topicCameraIntrinsics,
                                 double maxViewingDistance);
  void iterateAndDumpImages(const std::string& topicImage,
                            const std::string& cameraIntrinsicsUuid);
  void iterateAndDumpPc2(const std::string& topicPc2);
  void iterateAndDumpTf(const std::string& topicTf,
                        const std::string& topicTfStatic);
  void iterateAndDumpTf(const std::string& topicTf, const bool isStatic);

  /* composition to the seerep_hdf5_ros interface */
  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> hdf5Ros;

  rosbag::Bag bag;
};

}  // namespace seerep_ros_comm

#endif  // SEEREP_ROS_COMM_ROSBAG_DUMPER_AITF_H_
