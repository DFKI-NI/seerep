#ifndef SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

// seerep_hdf5_core
#include "hdf5_core_datatype_interface.h"
#include "hdf5_core_general.h"

// seerep-msgs
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/timestamp_frame_mesh.h>

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

using ExactKernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGPoint_3 = ExactKernel::Point_3;
using CGSurfaceMesh = CGAL::Surface_mesh<CGPoint_3>;
using CGVertexIndex = CGSurfaceMesh::Vertex_index;

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
   * @brief Get a timestamp-frameid-mesh struct which will be subject to
   *  encapsulation checks of a query polygon
   *
   * The mesh is created from the AABB bounding a pointcloud datatype entry
   *
   * @param uuid_entry of the pointcloud datatype entry to fetch the data from
   *
   * @return a struct containing the timestamp, frame_id of the data and the mesh itself
   */
  std::optional<seerep_core_msgs::TimestampFrameMesh>
  getPolygonConstraintMesh(const boost::uuids::uuid& uuid_entry);

  /**
   * @brief create mesh from a given AABB
   *
   * @param bb_coords the coordinates of the AABB in the following order:
   *       0. min_corner_x
   *       1. min_corner_y
   *       2. min_corner_z
   *       3. max_corner_x
   *       4. max_corner_y
   *       5. max_corner_z
   *  @return the surface mesh based on these coordinates
   */
  CGSurfaceMesh createMeshFromAABB(const std::vector<float>& bb_coords);

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
