#include "seerep_ros_comm/rostopic_dumper.h"

namespace seerep_ros_examples
{

bool topicAvailable(const std::string& topic, ros::master::TopicInfo& info)
{
  ros::master::V_TopicInfo topicInfo;
  ros::master::getTopics(topicInfo);
  auto findIter = std::find_if(topicInfo.begin(), topicInfo.end(),
                               [&topic](const ros::master::TopicInfo& info) {
                                 return info.name == topic;
                               });
  if (findIter != topicInfo.end())
  {
    info.name = findIter->name;
    info.datatype = findIter->datatype;
    return true;
  }
  return false;
}

std::string getCameraInfoTopic(const std::string& imageTopic)
{
  std::string cameraInfoTopic;
  size_t pos = imageTopic.rfind("/");
  if (pos != std::string::npos)
  {
    cameraInfoTopic = imageTopic.substr(0, pos) + "/camera_info";
  }
  return cameraInfoTopic;
}

Hdf5Node::Hdf5Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_{ nh }, pnh_{ pnh }
{
  const std::string path = getParameter<std::string>("path");
  const std::string projectName = getParameter<std::string>("project_name");
  const std::string rootFrameId =
      getParameter<std::string>("project_root_frame");
  topics_ = getParameter<std::vector<std::string>>("topics");
  const std::string filename =
      boost::lexical_cast<std::string>(boost::uuids::random_generator()()) +
      ".h5";
  hdf5Ros_ = std::make_unique<seerep_hdf5_ros::Hdf5Ros>(
      path + "/" + filename, rootFrameId, projectName);
}

void Hdf5Node::subscribe()
{
  for (const std::string& topic : topics_)
  {
    ros::master::TopicInfo info;
    if (!topicAvailable(topic, info))
    {
      ROS_WARN_STREAM("Topic " << topic << " not available");
    }

    ROS_INFO_STREAM("Subscribing to topic " << topic);
    ROS_INFO_STREAM("Topic datatype: " << info.datatype);

    if (info.datatype == "sensor_msgs/Image")
    {
      /*
       * Current workaround is to store the first camera info message and use it
       * for all following images. If the camera info is  subject to change, the
       * camera and camera info topic should be time synchronized.
       * This could be achieved by using message filters.
       * http://wiki.ros.org/message_filters
       */
      auto firstCameraInfo =
          ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
              getCameraInfoTopic(topic), nh_, ros::Duration(3.0));

      cameraInfoUuid_ = hdf5Ros_->dump(
          *firstCameraInfo, getParameter<float>("max_viewing_distance"));

      ros::Subscriber subImage = nh_.subscribe<sensor_msgs::Image>(
          topic, 0, &Hdf5Node::callback, this);
      subscribers_[topic] = subImage;
    }
    else if (info.datatype == "sensor_msgs/PointCloud2")
    {
      ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
          topic, 0, &Hdf5Node::callback, this);
      subscribers_[topic] = sub;
    }
    else if (topic == "/tf")
    {
      ros::Subscriber sub = nh_.subscribe<tf2_msgs::TFMessage>(
          topic, 0, &Hdf5Node::callback, this);
      subscribers_[topic] = sub;
    }
    else if (topic == "/tf_static")
    {
      ros::Subscriber sub = nh_.subscribe<tf2_msgs::TFMessage>(
          topic, 0, &Hdf5Node::callback_static, this);
      subscribers_[topic] = sub;
    }
    else
    {
      ROS_WARN_STREAM("Type " << info.datatype << " not supported");
    }
  }
}

void Hdf5Node::callback(const sensor_msgs::ImageConstPtr& image)
{
  hdf5Ros_->dump(*image, cameraInfoUuid_);
}

void Hdf5Node::callback(const sensor_msgs::PointCloud2ConstPtr& pointCloud)
{
  hdf5Ros_->dump(*pointCloud);
}

void Hdf5Node::callback(const tf2_msgs::TFMessageConstPtr& tfMessage)
{
  hdf5Ros_->dump(*tfMessage, false);
}

void Hdf5Node::callback_static(const tf2_msgs::TFMessageConstPtr& tfMessage)
{
  hdf5Ros_->dump(*tfMessage, true);
}

} /* Namespace seerep_ros_examples */

int main(int argc, char** argv)
{
  const std::string& node_name = "hdf5-node";
  ros::init(argc, argv, node_name);

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  seerep_ros_examples::Hdf5Node node(nh, pnh);
  node.subscribe();

  ros::spin();
}
