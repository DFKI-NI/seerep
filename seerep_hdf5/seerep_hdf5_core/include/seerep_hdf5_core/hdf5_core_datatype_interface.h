#ifndef SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_

// seerep-msgs
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/timestamp_frame_points.h>

// std
#include <boost/uuid/uuid.hpp>
#include <optional>

namespace seerep_hdf5_core
{

class Hdf5CoreDatatypeInterface
{
public:
  virtual std::optional<seerep_core_msgs::DatasetIndexable>
  readDataset(const boost::uuids::uuid& uuid) = 0;
  virtual std::optional<seerep_core_msgs::DatasetIndexable>
  readDataset(const std::string& uuid) = 0;

  virtual std::vector<std::string> getDatasetUuids() = 0;

  /**
   * @brief a option to pass points down to check for spatially, the first
   * return value are the points which should get checked, the second in which
   * frame they should be checked
   *
   * @param uuid_entry optionally specify the uuid of a specific data entry to
   * perform additional operations
   *
   * @return the points which should get checked (if empty this should be a
   * noop) and the frame_id in which they need to get transformed from
   */
  virtual std::optional<seerep_core_msgs::TimestampFramePoints>
  getPolygonConstraintPoints(const boost::uuids::uuid& uuid_entry) = 0;
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_ */
