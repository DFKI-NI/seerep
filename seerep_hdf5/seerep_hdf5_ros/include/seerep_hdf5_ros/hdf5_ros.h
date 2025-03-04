#ifndef SEEREP_HDF5_ROS_H_
#define SEEREP_HDF5_ROS_H_

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <highfive/H5File.hpp>
#include <mutex>
#include <string>

#include "geometry_msgs/TransformStamped.h"
#include "seerep_hdf5_core/hdf5_core_general.h"
#include "seerep_hdf5_core/hdf5_core_point_cloud.h"
#include "seerep_hdf5_core/hdf5_core_tf.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/Header.h"
#include "tf2_msgs/TFMessage.h"

namespace seerep_hdf5_ros
{

class Hdf5Ros
{
public:
  Hdf5Ros() = delete;

  /**
   * @brief Intialize the hdf5 file with the minimal required information.
   *
   * TODO: Add geo-refercing and versioning information.
   *
   * @param fileName Full path for the hdf5 file.
   * @param projectName Name of the seerep project.
   * @param projectFrameId Root frame of the project.
   */
  Hdf5Ros(const std::string& fileName, const std::string& projectName,
          const std::string& projectFrameId);

  /**
   * @brief Destructor to flush the file.
   */
  ~Hdf5Ros();

  /**
   * @brief Store an image message to hdf5.
   *
   * @param image Image message to dump.
   * @param uuid UUID of the corresponding camera info message.
   */
  void dump(const sensor_msgs::Image& image, const std::string& uuid);

  /**
   * @brief Store a compressed image message to hdf5.
   *
   * Note: Currently only used for benchmarking. Seerep currently does
   * not support compressed images.
   *
   * @param image Compressed image message to store.
   */
  void dump(const sensor_msgs::CompressedImage& image);
  std::string dump(const sensor_msgs::CameraInfo& cameraInfo,
                   float maxViewingDistance = 0.0);

  /**
   * @briefStore a point cloud message to hdf5.
   *
   * @param pcl Point cloud message to store.
   */
  void dump(const sensor_msgs::PointCloud2& pcl);

  /**
   * @brief Store a tf message to hdf5.
   *
   * @param tfMessage TF message to store.
   */
  void dump(const tf2_msgs::TFMessage& tfMessage, bool is_static);

  /**
   * @brief Store a transform message to hdf5.
   *
   * @param tf Transform message to store.
   */
  void dump(const geometry_msgs::TransformStamped& tf);

private:
  std::shared_ptr<std::mutex> mutex_;
  std::shared_ptr<HighFive::File> file_;

  /* composition to the hdf5 core to re-use data type independent implementations */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> hdf5Core_;

  std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> hdf5CoreTf_;

  /**
   * @brief Store a message header to hdf5.
   *
   * @param groupPath Path to the hdf5 group, to which the header should be added.
   * @param header Header message to store.
   */
  void dump(const std::string& groupPath, const std_msgs::Header& header);

  /**
   * @brief Store a region of interest message to hdf5.
   *
   * @param cameraInfoPath Path to the camerainfo the roi message should be added.
   * @param roi Region of interest message to store.
   */
  void dump(const std::string& cameraInfoPath,
            const sensor_msgs::RegionOfInterest& roi);

  /**
   * @brief Compute BoundingBox of whole pointcloud.
   *
   * @param pcl pointcloud2 msg to analyze.
   */
  std::pair<seerep_core_msgs::Point, seerep_core_msgs::Point>
  computeBoundingBox(const sensor_msgs::PointCloud2& pcl);
};

}  // namespace seerep_hdf5_ros

#endif /* SEEREP_HDF5_ROS_H_ */
