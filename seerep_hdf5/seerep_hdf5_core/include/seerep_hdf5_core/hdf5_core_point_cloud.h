#ifndef SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5_core
#include "hdf5_core_datatype_interface.h"
#include "hdf5_core_general.h"

// seerep-msgs
#include <seerep_msgs/dataset_indexable.h>

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
class Hdf5CorePointCloud : public Hdf5CoreGeneral,
                           public Hdf5CoreDatatypeInterface
{
public:
  Hdf5CorePointCloud(std::shared_ptr<HighFive::File>& file,
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
  frame_to_points_mapping getPolygonConstraintPoints();

public:
  // image / pointcloud attribute keys
  inline static const std::string HEIGHT = "height";
  inline static const std::string WIDTH = "width";
  inline static const std::string ENCODING = "encoding";
  inline static const std::string IS_BIGENDIAN = "is_bigendian";
  inline static const std::string ROW_STEP = "row_step";
  inline static const std::string POINT_STEP = "point_step";
  inline static const std::string IS_DENSE = "is_dense";

  // pointcloud fields attribute keys
  inline static const std::string FIELD_NAME = "field_name";
  inline static const std::string FIELD_OFFSET = "field_offset";
  inline static const std::string FIELD_DATATYPE = "field_datatype";
  inline static const std::string FIELD_COUNT = "field_count";

  // make private again after fixing io calls of pointcloud.cpp and pointcloud-overview.cpp
  inline static const std::string BOUNDINGBOX = "boundingbox";
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_POINTCLOUD = "pointclouds";
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_ */
