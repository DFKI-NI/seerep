#ifndef SEEREP_CORE_CORE_TF_H_
#define SEEREP_CORE_CORE_TF_H_

#include <functional>
#include <limits>
#include <optional>

// seerep-msgs
#include <seerep_msgs/aabb.h>
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_tf.h>

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// seerep_hdf5_core
#include <seerep_hdf5_core/hdf5_core_tf.h>

// ros tf2
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/buffer_core.h>
#include <tf2/transform_datatypes.h>

// CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core
{

using ExactKernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGPoint_3 = CGAL::Point_3<ExactKernel>;

/**
 * @brief This is the class handling the TF buffer
 *
 * TFs are stored in a tf2::BufferCore. TFs can be added to this buffer and
 * queries from one frame to another can be queried for a specific point in
 * time. AABBs and spatial queries can also be transformed directly in the map
 * frame of the project via public methods.
 */
class CoreTf
{
public:
  /**
   * @brief Constructs the object handling the TF buffer
   * @param hdf5_io pointer to the HDF5 io object for TFs
   */
  CoreTf(std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> hdf5_io);
  ~CoreTf();
  /**
   * @brief Returns the queried transformation between the two given frames at
   * the given point in time
   * @param timesecs queried transformation time; seconds
   * @param timenanos queried transformation time; nanoseconds
   * @param targetFrame the target frame of the transformation
   * @param sourceFrame the source frame of the transformation
   * @return the queried TF if it exists
   */
  std::optional<geometry_msgs::TransformStamped>
  getData(const int64_t& timesecs, const int64_t& timenanos,
          const std::string& targetFrame, const std::string& sourceFrame);
  /**
   * @brief Adds a tf to the tf buffer
   * @param tf the TransformStamped to be added to the buffer
   */
  void addDataset(const geometry_msgs::TransformStamped& tf,
                  const bool isStatic = false);

  /**
   * @brief Transforms an AABB into the target frame
   * @param aabb the AABB which should be transformed
   * @param sourceFrame the frame of the AABB before the transformation
   * @param targetFrame the frame of the AABB after the transformation
   * @param timeSecs the point in time to be used for the transformation; seconds
   * @param timeNanos the point in time to be used for the transformation; nanoseconds
   * @return the AABB transformed into the target frame
   */
  seerep_core_msgs::AABB transformAABB(seerep_core_msgs::AABB aabb,
                                       const std::string& sourceFrame,
                                       const std::string& targetFrame,
                                       const int64_t& timeSecs,
                                       const int64_t& timeNanos);
  /**
   * @brief Check if a transformation from the source to the target frame is available
   * at the defined point in time
   * @param sourceFrame the source frame of the transformation
   * @param targetFrame the target frame of the transformation
   * @param timeSecs the point in time to be used for the transformation; seconds
   * @param timeNanos the point in time to be used for the transformation; nanoseconds
   * @return true if transformation is available; false otherwise
   */
  bool canTransform(const std::string& sourceFrame,
                    const std::string& targetFrame, const int64_t& timeSecs,
                    const int64_t& timeNanos);

  /**
   * @brief transform points into another frame
   *
   * @param sourceFrame the frame the points are in
   * @param targetFrame the frame to transform the points to
   * @param points the points to transform
   *
   * @return the transformed points
   */
  std::vector<CGPoint_3>
  transform(const std::string& sourceFrame, const std::string& targetFrame,
            const int64_t& timeSecs, const int64_t& timeNanos,
            const std::vector<std::reference_wrapper<CGPoint_3>>& points);

  /**
   * @brief Returns a vector of all frames stored in the TF tree by the TF buffer
   * @return vector of frame names
   */
  std::vector<std::string> getFrames();

  /**
   * @brief cleans the current tf buffer and loads the TFs into the buffer from
   * the HDF5 file
   */
  void recreateBufferAndDatasets();

private:
  /**
   * @brief loads the TFs into the buffer from the HDF5 file
   */
  void recreateDatasets();

  void loadTfs(const std::vector<std::string> tfs, const bool isStatic);
  /**
   * @brief adds a transformation to the TF buffer
   * @param transform the transformation to be added to the buffer
   */
  void addToTfBuffer(geometry_msgs::TransformStamped transform,
                     const bool isStatic);

  /**
   * @brief Transforms the AABB based on the min/max coordinates into another
   * frame. Transforms all 8 vertices of the AABB into the new frame and gets
   * the new AABB based on all 8 transformed vertices
   *
   * @param transform the transform to the new frame
   * @param x the x coordinates of the old min/max vertices
   * @param y the y coordinates of the old min/max vertices
   * @param z the z coordinates of the old min/max vertices
   * @param xmin the min x value of the new AABB
   * @param ymin the min y value of the new AABB
   * @param zmin the min z value of the new AABB
   * @param xmax the max x value of the new AABB
   * @param ymax the max y value of the new AABB
   * @param zmax the max z value of the new AABB
   */
  void getAABBinNewFrame(const tf2::Transform& transform,
                         const std::vector<float>& x,
                         const std::vector<float>& y,
                         const std::vector<float>& z, float& xmin, float& ymin,
                         float& zmin, float& xmax, float& ymax, float& zmax);

  /** @brief shared pointer to the object handling the HDF5 io for TFs */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> m_hdf5_io;
  /** @brief the TF buffer */
  tf2::BufferCore m_tfBuffer;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_TF_H_
