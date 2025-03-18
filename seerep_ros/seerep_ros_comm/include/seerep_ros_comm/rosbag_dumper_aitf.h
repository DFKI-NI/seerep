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

// open cv
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

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
                   const std::vector<std::string>& topicsImage,
                   const std::vector<std::string>& topicsPc2,
                   const std::vector<std::string>& topicsCameraIntrinsics,
                   const std::string& topicTf, const std::string& topicTfStatic,
                   std::vector<double> maxViewingDistances);
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

  sensor_msgs::Image::ConstPtr convertCompressedImageToImage(
      const sensor_msgs::CompressedImage::ConstPtr& msg);

  /* composition to the seerep_hdf5_ros interface */
  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> hdf5Ros;

  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> ioCoreGeneral;

  rosbag::Bag bag;
};

}  // namespace seerep_ros_comm

#endif  // SEEREP_ROS_COMM_ROSBAG_DUMPER_AITF_H_
