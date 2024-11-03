#ifndef SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_

// seerep-msgs
#include <seerep_msgs/dataset_indexable.h>

// std
#include <boost/uuid/uuid.hpp>
#include <optional>

namespace seerep_hdf5_core
{
typedef std::optional<std::pair<std::string, std::vector<seerep_core_msgs::Point>>>
    frame_to_points_mapping;

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
   * @return the points which should get checked (if empty this should be a
   * noop) and the frame_id in which they need to get transformed from
   */
  virtual frame_to_points_mapping getPolygonConstraintPoints() = 0;
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_DATATYPE_INTERFACE_H_ */
