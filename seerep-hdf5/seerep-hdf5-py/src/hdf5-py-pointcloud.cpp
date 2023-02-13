#include "seerep-hdf5-py/hdf5-py-pointcloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyPointCloud::Hdf5PyPointCloud(Hdf5FileWrapper& hdf5_file)
  : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex()), Hdf5PyGeneral(hdf5_file)
{
}

std::vector<std::string> Hdf5PyPointCloud::getPointClouds()
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->exist(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD))
  {
    return std::vector<std::string>();
  }
  const HighFive::Group& clouds_group = m_file->getGroup(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD);
  return clouds_group.listObjectNames();
}

void Hdf5PyPointCloud::writePointCloud(const std::string& uuid, const std::string& frame_id, int64_t seconds,
                                       int32_t nanos, uint32_t sequence,
                                       const std::map<std::string, py::array> channels,
                                       const std::vector<GeneralLabel>& general_labels,
                                       const std::vector<CategorizedBoundingBoxLabel<3>>& bb_labels)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloud_group_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> data_group_ptr = getHdf5Group(cloud_group_id);

  std::size_t num_points = 0;
  for (const auto& [name, data] : channels)
  {
    py::buffer_info buff_info = data.request();
    if (name.compare("xyz") == 0)
    {
      num_points = buff_info.shape[0];
      break;
    }
    else if (name.compare("x") == 0)
    {
      num_points = buff_info.shape[0];
      break;
    }
  }

  // write header

  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::HEADER_FRAME_ID, frame_id);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::HEADER_SEQ, sequence);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_SECONDS, seconds);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_NANOS, nanos);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::HEADER_SEQ, sequence);

  // write labels

  Hdf5PyGeneral::writeBoundingBoxLabeled(cloud_group_id, bb_labels);
  Hdf5PyGeneral::writeLabelsGeneral(cloud_group_id, general_labels);

  // write data

  for (const auto& [name, data] : channels)
  {
    writeChannel(cloud_group_id, name, channels);
  }

  // write boundingbox

  bool success = false;

  if (!success)
  {
    success = writeBoundingBox<double, 3>(cloud_group_id, { "points" }, channels);
  }
  if (!success)
  {
    success = writeBoundingBox<float, 3>(cloud_group_id, { "points" }, channels);
  }
  if (!success)
  {
    success = writeBoundingBox<double, 3>(cloud_group_id, { "xyz" }, channels);
  }
  if (!success)
  {
    success = writeBoundingBox<float, 3>(cloud_group_id, { "xyz" }, channels);
  }
  if (!success)
  {
    success = writeBoundingBox<double, 3>(cloud_group_id, { "x", "y", "z" }, channels);
  }
  if (!success)
  {
    success = writeBoundingBox<float, 3>(cloud_group_id, { "x", "y", "z" }, channels);
  }

  // write point fields

  std::vector<std::string> names(channels.size());
  std::vector<uint32_t> offsets(channels.size());
  std::vector<uint8_t> datatypes(channels.size());
  std::vector<uint32_t> counts(channels.size());
  uint32_t offset = 0;
  for (const auto& [name, data] : channels)
  {
    py::buffer_info channel_info = data.request();

    names.push_back(name);
    offsets.push_back(offset);

    switch (data.dtype().char_())
    {
      case 'b':
        datatypes.push_back(1);  // int8
        break;
      case 'i':
        datatypes.push_back(5);  // int32
        break;
      case 'B':
        datatypes.push_back(2);  // uint8
        break;
      case 'I':
        datatypes.push_back(6);  // uint32
        break;
      case 'f':
        datatypes.push_back(7);  // float32
        break;
      case 'd':
        datatypes.push_back(8);  // float64
        break;
    }

    uint32_t num_elements = 1;
    if (channel_info.shape.size() > 1)
    {
      num_elements = channel_info.shape[1];
    }
    counts.push_back(num_elements);

    offset += data.dtype().itemsize() * num_elements;
  }

  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, offsets);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(
      *data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE, datatypes);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*data_group_ptr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);

  // write basic attributes

  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, 1);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, num_points);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN, false);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, offset);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP, num_points * offset);
  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, true);

  m_file->flush();
}

std::tuple<std::map<std::string, py::array>, std::vector<GeneralLabel>, std::vector<CategorizedBoundingBoxLabel<3>>>
Hdf5PyPointCloud::readPointCloud(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloud_group_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  if (!m_file->exist(cloud_group_id))
  {
    return std::make_tuple(std::map<std::string, py::array>(), std::vector<GeneralLabel>(),
                           std::vector<CategorizedBoundingBoxLabel<3>>());
  }

  HighFive::Group cloud_group = m_file->getGroup(cloud_group_id);

  uint32_t height, width, point_step, row_step;
  bool is_bigendian, is_dense;
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT).read(height);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH).read(width);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN).read(is_bigendian);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP).read(point_step);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP).read(row_step);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE).read(is_dense);

  std::vector<std::string> names;
  std::vector<uint32_t> offsets;
  std::vector<uint32_t> counts;
  std::vector<uint8_t> datatypes;

  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME).read(names);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET).read(offsets);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE).read(datatypes);
  cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT).read(counts);

  if (names.size() != offsets.size() || names.size() != counts.size() || names.size() != datatypes.size())
  {
    throw std::invalid_argument("point field sizes do not match up for pointcloud " + uuid);
  }

  std::map<std::string, py::array> channels;

  for (std::size_t i = 0; i < names.size(); i++)
  {
    if (counts[i] == 0)
    {
      continue;
    }

    if (!m_file->exist(cloud_group_id + "/" + names[i]))
    {
      throw std::invalid_argument("pointcloud field " + names[i] + " not found in pointcloud " + uuid);
    }

    std::shared_ptr<HighFive::DataSet> dataset = getHdf5DataSet(cloud_group_id + "/" + names[i]);

    if (datatypes[i] == 1)
    {
      channels[names[i]] = readField<int8_t>(dataset);
    }
    else if (datatypes[i] == 5)
    {
      channels[names[i]] = readField<int32_t>(dataset);
    }
    else if (datatypes[i] == 2)
    {
      channels[names[i]] = readField<uint8_t>(dataset);
    }
    else if (datatypes[i] == 6)
    {
      channels[names[i]] = readField<uint32_t>(dataset);
    }
    else if (datatypes[i] == 7)
    {
      channels[names[i]] = readField<float>(dataset);
    }
    else if (datatypes[i] == 7)
    {
      channels[names[i]] = readField<double>(dataset);
    }
    else
    {
      throw std::invalid_argument("unknown type in field " + names[i] + " of pointcloud " + uuid);
    }
  }

  // read labels
  auto general_labels = Hdf5PyGeneral::readLabelsGeneral(cloud_group_id);
  auto bb_labels = Hdf5PyGeneral::readBoundingBoxLabeled<3>(cloud_group_id);

  return std::make_tuple(channels, general_labels, bb_labels);
}

void Hdf5PyPointCloud::writeChannel(const std::string& cloud_group_id, const std::string& channel_name,
                                    const std::map<std::string, py::array>& channels)
{
  std::vector<std::vector<std::string>> channel_names({ { channel_name } });

  if (!writeChannelTyped<char, int, uint8_t, unsigned int, float, double>(cloud_group_id, channel_name, channel_name,
                                                                          channels))
  {
    throw std::invalid_argument("unable to write field '" + channel_name + "'");
  }
}

} /* namespace seerep_hdf5_py */
