#ifndef SEEREP_HDF5_CORE_HDF5_CORE_POINT_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_POINT_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5_core
#include "hdf5_core_datatype_interface.h"
#include "hdf5_core_general.h"

// seerep_msgs
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/timestamp_frame_points.h>

// std
#include <boost/geometry.hpp>
#include <optional>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_hdf5_core
{
class Hdf5CorePoint : public Hdf5CoreGeneral, public Hdf5CoreDatatypeInterface
{
public:
  Hdf5CorePoint(std::shared_ptr<HighFive::File>& file,
                std::shared_ptr<std::mutex>& write_mtx);

  std::optional<seerep_core_msgs::DatasetIndexable>
  readDataset(const boost::uuids::uuid& uuid);
  std::optional<seerep_core_msgs::DatasetIndexable>
  readDataset(const std::string& uuid);

  std::vector<std::string> getDatasetUuids();

  /**
   * @brief THIS IS CURRENTLY A NOOP
   *
   * @return a empty vector and string
   */
  std::optional<seerep_core_msgs::TimestampFramePoints>
  getPolygonConstraintPoints(const boost::uuids::uuid& uuid_entry);

public:
  inline static const std::string RAWDATA = "rawdata";

  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_POINT = "points";
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_POINT_H_ */
