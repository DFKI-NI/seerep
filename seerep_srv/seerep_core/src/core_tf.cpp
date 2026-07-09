#include "seerep_core/core_tf.h"

namespace seerep_core
{
// construct tfbuffer with INT_MAX so that it holds ALL tfs added
CoreTf::CoreTf(std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> hdf5_io)
  : m_hdf5_io(hdf5_io), m_tfBuffer(ros::DURATION_MAX)
{
  recreateDatasets();
}
CoreTf::~CoreTf()
{
}

void CoreTf::recreateDatasets()
{
  std::vector<std::string> tfs =
      m_hdf5_io->getGroupDatasets(seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF);
  loadTfs(tfs, false);

  std::vector<std::string> tfs_static = m_hdf5_io->getGroupDatasets(
      seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF_STATIC);
  loadTfs(tfs_static, true);
}

void CoreTf::recreateBufferAndDatasets()
{
  // cannot recreate the tf buffer object, because its protected by a mutex
  m_tfBuffer.clear();
  this->recreateDatasets();
}

void CoreTf::loadTfs(std::vector<std::string> tfs, const bool isStatic)
{
  for (auto const& name : tfs)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "found " << name << " in HDF5 file.";

    try
    {
      std::optional<std::vector<geometry_msgs::TransformStamped>> transforms =
          m_hdf5_io->readTransformStamped(name, isStatic);
      if (transforms)
      {
        for (auto& transform : transforms.value())
        {
          addToTfBuffer(transform, isStatic);
        }
      }
    }
    catch (const std::runtime_error& e)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << e.what();
    }
  }
}

std::optional<geometry_msgs::TransformStamped>
CoreTf::getData(const int64_t& timesecs, const int64_t& timenanos,
                const std::string& targetFrame, const std::string& sourceFrame)
{
  try
  {
    return m_tfBuffer.lookupTransform(targetFrame, sourceFrame,
                                      ros::Time(timesecs, timenanos));
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << e.what();
    return std::nullopt;
  }
}

void CoreTf::addDataset(const geometry_msgs::TransformStamped& transform,
                        const bool isStatic)
{
  addToTfBuffer(transform, isStatic);
}

// TODO optimise!
seerep_core_msgs::AABB CoreTf::transformAABB(seerep_core_msgs::AABB aabb,
                                             const std::string& sourceFrame,
                                             const std::string& targetFrame,
                                             const int64_t& timeSecs,
                                             const int64_t& timeNanos)
{
  if (targetFrame != sourceFrame)
  {
    auto tf = m_tfBuffer.lookupTransform(m_hdf5_io->tf2_frame_id(targetFrame),
                                         m_hdf5_io->tf2_frame_id(sourceFrame),
                                         ros::Time(timeSecs, timeNanos));
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(tf.transform.translation.x,
                                     tf.transform.translation.y,
                                     tf.transform.translation.z));
    transform.setRotation(
        tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w));

    std::vector<float> x{ bg::get<bg::min_corner, 0>(aabb),
                          bg::get<bg::max_corner, 0>(aabb) };
    std::vector<float> y{ bg::get<bg::min_corner, 1>(aabb),
                          bg::get<bg::max_corner, 1>(aabb) };
    std::vector<float> z{ bg::get<bg::min_corner, 2>(aabb),
                          bg::get<bg::max_corner, 2>(aabb) };

    float xmin, ymin, zmin, xmax, ymax, zmax;
    getAABBinNewFrame(transform, x, y, z, xmin, ymin, zmin, xmax, ymax, zmax);

    bg::set<bg::min_corner, 0>(aabb, xmin);
    bg::set<bg::min_corner, 1>(aabb, ymin);
    bg::set<bg::min_corner, 2>(aabb, zmin);

    bg::set<bg::max_corner, 0>(aabb, xmax);
    bg::set<bg::max_corner, 1>(aabb, ymax);
    bg::set<bg::max_corner, 2>(aabb, zmax);
  }
  return aabb;
}

bool CoreTf::canTransform(const std::string& sourceFrame,
                          const std::string& targetFrame,
                          const int64_t& timeSecs, const int64_t& timeNanos)
{
  return m_tfBuffer.canTransform(m_hdf5_io->tf2_frame_id(targetFrame),
                                 m_hdf5_io->tf2_frame_id(sourceFrame),
                                 ros::Time(timeSecs, timeNanos));
}

std::vector<CGPoint_3>
CoreTf::transform(const std::string& sourceFrame,
                  const std::string& targetFrame, const int64_t& timeSecs,
                  const int64_t& timeNanos,
                  const std::vector<std::reference_wrapper<CGPoint_3>>& points)
{
  if (sourceFrame == targetFrame)
  {
    return std::vector<CGPoint_3>{ points.begin(), points.end() };
  }

  std::vector<CGPoint_3> transformed_points;
  if (this->canTransform(sourceFrame, targetFrame, timeSecs, timeNanos))
  {
    auto tf =
        m_tfBuffer.lookupTransform(targetFrame, sourceFrame,
                                   ros::Time(static_cast<uint32_t>(timeSecs),
                                             static_cast<uint32_t>(timeNanos)));
    auto rot = tf.transform.rotation;
    auto trans = tf.transform.translation;
    tf2::Quaternion q{ rot.x, rot.y, rot.z, rot.w };
    tf2::Vector3 o{ trans.x, trans.y, trans.z };
    tf2::Transform tf2_tf{ q, o };

    for (auto&& p : points)
    {
      tf2::Vector3 vec{ p.get().x().exact().convert_to<double>(),
                        p.get().y().exact().convert_to<double>(),
                        p.get().z().exact().convert_to<double>() };

      tf2::Vector3 vec_target = tf2_tf * vec;

      CGPoint_3 p_target{ vec_target.getX(), vec_target.getY(),
                          vec_target.getZ() };

      transformed_points.push_back(p_target);
    }
  }
  else
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "Could not find transform" << sourceFrame << " -> " << targetFrame
        << " in tf buffer at time " << (timeSecs * 10 ^ 9) << timeNanos
        << "ns.";
  }
  return transformed_points;
}

std::vector<std::string> CoreTf::getFrames()
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
      << m_tfBuffer.allFramesAsString();

  return std::vector<std::string>{ m_tfBuffer.allFramesAsYAML() };
}

void CoreTf::addToTfBuffer(geometry_msgs::TransformStamped transform,
                           const bool isStatic)
{
  /* Make sure frame_ids are tf2 compatible */
  transform.header.frame_id =
      m_hdf5_io->tf2_frame_id(transform.header.frame_id);
  transform.child_frame_id = m_hdf5_io->tf2_frame_id(transform.child_frame_id);

  if (!m_tfBuffer.setTransform(transform, "fromHDF5", isStatic))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << "Could not add transform to tfBuffer";
  }
}

void CoreTf::getAABBinNewFrame(const tf2::Transform& transform,
                               const std::vector<float>& x,
                               const std::vector<float>& y,
                               const std::vector<float>& z, float& xmin,
                               float& ymin, float& zmin, float& xmax,
                               float& ymax, float& zmax)
{
  std::vector<float> xTransformed, yTransformed, zTransformed;

  for (float xAct : x)
  {
    for (float yAct : y)
    {
      for (float zAct : z)
      {
        tf2::Vector3 v(xAct, yAct, zAct);
        tf2::Vector3 vTransformed = transform * v;
        xTransformed.push_back(vTransformed.getX());
        yTransformed.push_back(vTransformed.getY());
        zTransformed.push_back(vTransformed.getZ());
      }
    }
  }
  xmin = *std::min_element(xTransformed.begin(), xTransformed.end());
  ymin = *std::min_element(yTransformed.begin(), yTransformed.end());
  zmin = *std::min_element(zTransformed.begin(), zTransformed.end());
  xmax = *std::max_element(xTransformed.begin(), xTransformed.end());
  ymax = *std::max_element(yTransformed.begin(), yTransformed.end());
  zmax = *std::max_element(zTransformed.begin(), zTransformed.end());
}

} /* namespace seerep_core */
