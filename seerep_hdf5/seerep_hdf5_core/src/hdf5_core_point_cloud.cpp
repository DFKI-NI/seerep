#include "seerep_hdf5_core/hdf5_core_point_cloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CorePointCloud::Hdf5CorePointCloud(std::shared_ptr<HighFive::File>& file,
                                       std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

std::optional<seerep_core_msgs::DatasetIndexable>
Hdf5CorePointCloud::readDataset(const boost::uuids::uuid& uuid)
{
  return readDataset(boost::lexical_cast<std::string>(uuid));
}

std::optional<seerep_core_msgs::DatasetIndexable>
Hdf5CorePointCloud::readDataset(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetPath = HDF5_GROUP_POINTCLOUD + "/" + uuid;

  if (!m_file->exist(hdf5DatasetPath))
  {
    return std::nullopt;
  }

  const std::shared_ptr<HighFive::Group> group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5DatasetPath));

  seerep_core_msgs::DatasetIndexable data;

  data.header.datatype = seerep_core_msgs::Datatype::PointCloud;

  boost::uuids::string_generator gen;
  data.header.uuidData = gen(uuid);

  readHeader(uuid, *group_ptr, data.header);

  std::vector<float> bb;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX)
      .read(bb);
  data.boundingbox.min_corner().set<0>(bb.at(0));
  data.boundingbox.min_corner().set<1>(bb.at(1));
  data.boundingbox.min_corner().set<2>(bb.at(2));
  data.boundingbox.max_corner().set<0>(bb.at(3));
  data.boundingbox.max_corner().set<1>(bb.at(4));
  data.boundingbox.max_corner().set<2>(bb.at(5));

  readLabelsAndAddToLabelsPerCategory(HDF5_GROUP_POINTCLOUD, uuid,
                                      data.labelsCategory);

  return data;
}

std::vector<std::string> Hdf5CorePointCloud::getDatasetUuids()
{
  return getGroupDatasets(HDF5_GROUP_POINTCLOUD);
}

std::optional<seerep_core_msgs::TimestampFrameMesh>
Hdf5CorePointCloud::getPolygonConstraintMesh(const boost::uuids::uuid& uuid_entry)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string uuid_str = boost::lexical_cast<std::string>(uuid_entry);

  std::string hdf5DatasetPath = HDF5_GROUP_POINTCLOUD + "/" + uuid_str;

  if (!m_file->exist(hdf5DatasetPath))
  {
    throw std::invalid_argument("Hdf5DatasetPath not present for uuid " +
                                uuid_str + "! Skipping precise check...");
    BOOST_LOG_SEV(this->m_logger, boost::log::trivial::severity_level::info);
    return std::nullopt;
  }

  const std::shared_ptr<HighFive::Group> group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5DatasetPath));

  std::vector<float> bb;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX)
      .read(bb);

  seerep_core_msgs::Header head;

  readHeader(uuid_str, *group_ptr, head);

  return seerep_core_msgs::TimestampFrameMesh{ head.timestamp, head.frameId,
                                               this->createMeshFromAABB(bb) };
}

CGSurfaceMesh
Hdf5CorePointCloud::createMeshFromAABB(const std::vector<float>& bb_coords)
{
  CGSurfaceMesh mesh;

  // define base indices
  int x_base = 0;
  int y_base = 1;
  int z_base = 2;

  std::vector<CGVertexIndex> verts;

  // create vertices with 3bit cube coords
  for (uint idx = 0; idx < 0b1000; idx++)
  {
    int x_i = x_base + 3 * (idx & 0b001);
    int y_i = y_base + 3 * ((idx & 0b010) >> 1);
    int z_i = z_base + 3 * ((idx & 0b100) >> 2);

    CGPoint_3 p{ bb_coords.at(x_i), bb_coords.at(y_i), bb_coords.at(z_i) };

    verts.push_back(mesh.add_vertex(p));
  }

  // bottom plane
  auto desc = mesh.add_face(verts[0], verts[4], verts[6], verts[2]);
  if (desc == CGSurfaceMesh::null_face())
  {
    throw std::invalid_argument(
        "Could not add bottom face from AABB to SurfaceMesh correctly!");
  }
  // front plane
  mesh.add_face(verts[0], verts[1], verts[3], verts[2]);
  if (desc == CGSurfaceMesh::null_face())
  {
    throw std::invalid_argument(
        "Could not add front face from AABB to SurfaceMesh correctly!");
  }
  // left plane
  mesh.add_face(verts[4], verts[5], verts[1], verts[0]);
  if (desc == CGSurfaceMesh::null_face())
  {
    throw std::invalid_argument(
        "Could not add left face from AABB to SurfaceMesh correctly!");
  }
  // back plane
  mesh.add_face(verts[4], verts[5], verts[7], verts[6]);
  if (desc == CGSurfaceMesh::null_face())
  {
    throw std::invalid_argument(
        "Could not add back face from AABB to SurfaceMesh correctly!");
  }
  // right plane
  mesh.add_face(verts[2], verts[3], verts[7], verts[6]);
  if (desc == CGSurfaceMesh::null_face())
  {
    throw std::invalid_argument(
        "Could not add right face from AABB to SurfaceMesh correctly!");
  }
  // top plane
  mesh.add_face(verts[1], verts[5], verts[7], verts[3]);
  if (desc == CGSurfaceMesh::null_face())
  {
    throw std::invalid_argument(
        "Could not add top face from AABB to SurfaceMesh correctly!");
  }
  return mesh;
}

}  // namespace seerep_hdf5_core
