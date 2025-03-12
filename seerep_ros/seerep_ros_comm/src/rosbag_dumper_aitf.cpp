#include "seerep_ros_comm/rosbag_dumper_aitf.h"

namespace seerep_ros_comm
{
RosbagDumperAitf::RosbagDumperAitf(
    const std::string& bagPath, const std::string& hdf5FilePath,
    const std::string& projectFrameId, const std::string& projectName,
    const std::vector<std::string>& topicsImage,
    const std::vector<std::string>& topicsPc2,
    const std::vector<std::string>& topicsCameraIntrinsics,
    const std::string& topicTf, const std::string& topicTfStatic,
    std::vector<double> maxViewingDistances)
{
  hdf5Ros = std::make_unique<seerep_hdf5_ros::Hdf5Ros>(
      hdf5FilePath, projectFrameId, projectName);

  bag.open(bagPath);

  iterateAndDumpTf(topicTf, topicTfStatic);

  // Iterate over images and camera info topics

  size_t index = 0;
  for (const auto& topicImage : topicsImage)
  {
    std::string cameraIntrinsicsUuid = getCameraIntrinsic(
        topicsCameraIntrinsics[index], maxViewingDistances[index]);

    iterateAndDumpImages(topicImage, cameraIntrinsicsUuid);
    ++index;
  }
  // Iterate over pc2 topcis
  for (const auto& topicPc2 : topicsPc2)
  {
    iterateAndDumpPc2(topicPc2);
  }
}

RosbagDumperAitf::~RosbagDumperAitf()
{
  bag.close();
}

std::string
RosbagDumperAitf::getCameraIntrinsic(const std::string& topicCameraIntrinsics,
                                     double maxViewingDistance)
{
  for (const rosbag::MessageInstance& m :
       rosbag::View(bag, rosbag::TopicQuery(topicCameraIntrinsics)))
  {
    sensor_msgs::CameraInfo::ConstPtr msg =
        m.instantiate<sensor_msgs::CameraInfo>();
    if (msg != nullptr)
    {
      return hdf5Ros->dump(*msg, maxViewingDistance);
    }
  }
  return "";
}

void RosbagDumperAitf::iterateAndDumpImages(
    const std::string& topicImage, const std::string& cameraIntrinsicsUuid)
{
  for (const rosbag::MessageInstance& m :
       rosbag::View(bag, rosbag::TopicQuery(topicImage)))
  {
    sensor_msgs::Image::ConstPtr raw_msg;
    if (m.getDataType() == "sensor_msgs/CompressedImage")
    {
      {
        sensor_msgs::CompressedImage::ConstPtr msg =
            m.instantiate<sensor_msgs::CompressedImage>();
        raw_msg = convertCompressedImageToImage(msg);
      }
    }
    else
    {
      raw_msg = m.instantiate<sensor_msgs::Image>();
    }
    if (raw_msg != nullptr)
    {
      hdf5Ros->dump(*raw_msg, cameraIntrinsicsUuid);
    }
    else
    {
      ROS_ERROR_STREAM("nullptr while iterating images");
    }
  }
}

void RosbagDumperAitf::iterateAndDumpPc2(const std::string& topicPc2)
{
  for (const rosbag::MessageInstance& m :
       rosbag::View(bag, rosbag::TopicQuery(topicPc2)))
  {
    sensor_msgs::PointCloud2::ConstPtr msg =
        m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != nullptr)
    {
      hdf5Ros->dump(*msg);
    }
    else
    {
      ROS_ERROR_STREAM("nullptr while iterating images");
    }
  }
}

void RosbagDumperAitf::iterateAndDumpTf(const std::string& topicTf,
                                        const std::string& topicTfStatic)
{
  iterateAndDumpTf(topicTf, false);
  iterateAndDumpTf(topicTfStatic, true);
}

void RosbagDumperAitf::iterateAndDumpTf(const std::string& topic,
                                        const bool isStatic)
{
  rosbag::View view(bag, rosbag::TopicQuery(topic));

  for (const rosbag::MessageInstance& m :
       rosbag::View(bag, rosbag::TopicQuery(topic)))
  {
    tf2_msgs::TFMessage::ConstPtr msg = m.instantiate<tf2_msgs::TFMessage>();
    if (msg != nullptr)
    {
      hdf5Ros->dump(*msg, isStatic);
    }
    else
    {
      ROS_ERROR_STREAM("nullptr while iterating tf");
    }
  }
}

sensor_msgs::Image::ConstPtr RosbagDumperAitf::convertCompressedImageToImage(
    const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  // toCV
  cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

  // header
  std_msgs::Header header = msg->header;
  // to image
  sensor_msgs::Image::ConstPtr img_msg =
      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

  return img_msg;
}

}  // namespace seerep_ros_comm

std::string getHDF5FilePath(ros::NodeHandle privateNh, std::string& projectUuid)
{
  std::string hdf5FolderPath;
  if (!privateNh.getParam("hdf5FolderPath", hdf5FolderPath))
  {
    ROS_WARN_STREAM(
        "Use the \"hdf5FolderPath\" parameter to specify the HDF5 file!");
    return "";
  }

  if (privateNh.getParam("projectUuid", projectUuid))
  {
    try
    {
      boost::uuids::string_generator gen;
      // if this throws no exception, the UUID is valid
      gen(projectUuid);
    }
    catch (std::runtime_error const& e)
    {
      // mainly catching "invalid uuid string"
      ROS_ERROR_STREAM(e.what());
      projectUuid =
          boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      ROS_WARN_STREAM(
          "The provided UUID is invalid! Generating a a new one. (" +
          projectUuid + ".h5)");
    }
  }
  else
  {
    projectUuid =
        boost::lexical_cast<std::string>(boost::uuids::random_generator()());
    ROS_WARN_STREAM("Use the \"projectUuid\" parameter to specify the HDF5 "
                    "file! Generating a a new one. (" +
                    projectUuid + ".h5)");
  }

  return hdf5FolderPath + "/" + projectUuid + ".h5";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_rosbagDumperAitf");
  ros::NodeHandle privateNh("~");

  std::string bagPath, projectFrameId, projectName, topicTf, topicTfStatic;
  std::vector<std::string> topicsImage, topicsPc2, topicsCameraIntrinsics;
  std::vector<double> maxViewingDistances;

  std::string projectUuid;
  const std::string hdf5FilePath = getHDF5FilePath(privateNh, projectUuid);

  if (privateNh.getParam("bagPath", bagPath) &&
      privateNh.getParam("projectFrameId", projectFrameId) &&
      privateNh.getParam("projectName", projectName) &&
      privateNh.getParam("topicImage", topicsImage) &&
      privateNh.getParam("topicPc2", topicsPc2) &&
      privateNh.getParam("topicCameraIntrinsics", topicsCameraIntrinsics) &&
      privateNh.getParam("topicTf", topicTf) &&
      privateNh.getParam("topicTfStatic", topicTfStatic) &&
      privateNh.getParam("maxViewingDistance", maxViewingDistances))
  {
    ROS_INFO_STREAM("hdf5FilePath: " << hdf5FilePath);
    ROS_INFO_STREAM("bagPath: " << bagPath);
    ROS_INFO_STREAM("projectFrameId: " << projectFrameId);
    ROS_INFO_STREAM("projectName: " << projectName);
    for (auto topicImage : topicsImage)
    {
      ROS_INFO_STREAM("topicImage: " << topicImage);
    }
    for (auto topicPc2 : topicsPc2)
    {
      ROS_INFO_STREAM("topicPc2: " << topicPc2);
    }
    for (auto topicCameraIntrinsics : topicsCameraIntrinsics)
    {
      ROS_INFO_STREAM("topicCameraIntrinsics: " << topicCameraIntrinsics);
    }
    ROS_INFO_STREAM("topicTf: " << topicTf);
    ROS_INFO_STREAM("topicTfStatic: " << topicTfStatic);
    for (double maxViewingDistance : maxViewingDistances)
    {
      ROS_INFO_STREAM("maxViewingDistance: " << maxViewingDistance);
    }
    if (maxViewingDistances.size() == topicsCameraIntrinsics.size() &&
        topicsImage.size() == topicsCameraIntrinsics.size())
    {
      seerep_ros_comm::RosbagDumperAitf rosbagDumperAitf(
          bagPath, hdf5FilePath, projectFrameId, projectName, topicsImage,
          topicsPc2, topicsCameraIntrinsics, topicTf, topicTfStatic,
          maxViewingDistances);
    }
    else
    {
      ROS_ERROR_STREAM("Check length of image related params!");
    }
  }
  else
  {
    ROS_ERROR_STREAM("Not all mandatory parameters are set!");
  }

  return EXIT_SUCCESS;
}
