#ifndef HDF5_NODE_H
#define HDF5_NODE_H

#include <ros/master.h>
#include <ros/ros.h>
#include <seerep_hdf5_ros/hdf5_ros.h>

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <chrono>
#include <optional>
#include <thread>

namespace seerep_ros_examples
{

std::string getCameraInfoTopic(const std::string& imageTopic);
bool topicAvailable(const std::string& topic, ros::master::TopicInfo& info);

class Hdf5Node
{
public:
  Hdf5Node() = delete;

  Hdf5Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  void subscribe();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  /* store the topics to dump */
  std::vector<std::string> topics_;

  /* keeps the subsribers alive outside of their initial function scope */
  std::map<std::string, ros::Subscriber> subscribers_;

  /*
   * Current workaround is to store the first camera info message and use it
   * for all following images. If the camera info is  subject to change, the
   * camera and camera info topic should be time synchronized.
   * This could be achieved by using message filters.
   * http://wiki.ros.org/message_filters
   */
  std::string cameraInfoUuid_;

  /* composition to the seerep_hdf5_ros interface */
  std::unique_ptr<seerep_hdf5_ros::Hdf5Ros> hdf5Ros_;

  /* @brief Get a parameter from the parameter server.
   *
   * @tparam T Type of the parameter.
   * @param name Nname of the parameter.
   * @return Vvalue of the parameter.
   * @throws std::runtime_error if the parameter is missing.
   */
  template <typename T>
  T getParameter(const std::string& name);

  /*
    Callbacks for the subscribers, some require additional logic, thus it is
    not possible to unify them with a template.
  */
  void callback(const sensor_msgs::Image::ConstPtr& msg);
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void callback(const tf2_msgs::TFMessage::ConstPtr& msg);
  void callback_static(const tf2_msgs::TFMessage::ConstPtr& msg);
};

template <typename T>
T Hdf5Node::getParameter(const std::string& name)
{
  T value;
  if (!pnh_.getParam(name, value))
  {
    throw std::runtime_error("Paramter '" + name + "' is missing");
  }
  return value;
}

} /* Namespace seerep_ros_examples */

#endif /* HDF5_NODE_H_ */
