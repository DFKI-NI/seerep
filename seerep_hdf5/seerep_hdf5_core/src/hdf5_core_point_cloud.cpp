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

  std::shared_ptr<HighFive::Group> group_ptr =
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

std::optional<seerep_core_msgs::TimestampFramePoints>
Hdf5CorePointCloud::getPolygonConstraintPoints(
    std::optional<boost::uuids::uuid> uuid_entry)
{
  return std::nullopt;
}

}  // namespace seerep_hdf5_core
