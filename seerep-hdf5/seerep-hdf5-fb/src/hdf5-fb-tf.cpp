#include "seerep-hdf5-fb/hdf5-fb-tf.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbTf::Hdf5FbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5FbGeneral(file, write_mtx)
{
}

void Hdf5FbTf::writeTransformStamped(const seerep::fb::TransformStamped& tf)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetPath = HDF5_GROUP_TF + "/" + tf.header()->frame_id()->str() + "_" + tf.child_frame_id()->str();
  std::string hdf5DatasetTimePath = hdf5DatasetPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5DatasetPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5DatasetPath + "/" + "rotation";

  std::shared_ptr<HighFive::DataSet> data_set_time_ptr, data_set_trans_ptr, data_set_rot_ptr;
  uint64_t size = 0;

  if (!m_file->exist(hdf5DatasetPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "data id " << hdf5DatasetPath << " does not exist! Creat new dataset in hdf5";
    HighFive::Group group = m_file->createGroup(hdf5DatasetPath);
    group.createAttribute("CHILD_FRAME", tf.child_frame_id()->str());
    group.createAttribute("PARENT_FRAME", tf.header()->frame_id()->str());

    // TIME
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_time({ 1, 2 }, { HighFive::DataSpace::UNLIMITED, 2 });
    // Use chunking
    HighFive::DataSetCreateProps props_time;
    props_time.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 2 }));
    data_set_time_ptr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<int64_t>(hdf5DatasetTimePath, data_space_time, props_time));

    // TRANSLATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_trans({ 1, 3 }, { HighFive::DataSpace::UNLIMITED, 3 });
    // Use chunking
    HighFive::DataSetCreateProps props_trans;
    props_trans.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 3 }));
    data_set_trans_ptr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<double>(hdf5DatasetTransPath, data_space_trans, props_trans));

    // ROTATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_rot({ 1, 4 }, { HighFive::DataSpace::UNLIMITED, 4 });
    // Use chunking
    HighFive::DataSetCreateProps props_rot;
    props_rot.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 4 }));
    data_set_rot_ptr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<double>(hdf5DatasetRotPath, data_space_rot, props_rot));
  }
  else
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "data id " << hdf5DatasetPath << " already exists!";
    data_set_time_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTimePath));
    data_set_trans_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTransPath));
    data_set_rot_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRotPath));

    HighFive::Group group = m_file->getGroup(hdf5DatasetPath);
    group.getAttribute(SIZE).read(size);

    // Resize the dataset to a larger size
    data_set_time_ptr->resize({ size + 1, 2 });
    data_set_trans_ptr->resize({ size + 1, 3 });
    data_set_rot_ptr->resize({ size + 1, 4 });
  }

  // write time
  std::vector<int64_t> time;
  time.push_back(tf.header()->stamp()->seconds());
  time.push_back(tf.header()->stamp()->nanos());
  data_set_time_ptr->select({ size, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.transform()->translation()->x());
  trans.push_back(tf.transform()->translation()->y());
  trans.push_back(tf.transform()->translation()->z());
  data_set_trans_ptr->select({ size, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.transform()->rotation()->x());
  rot.push_back(tf.transform()->rotation()->y());
  rot.push_back(tf.transform()->rotation()->z());
  rot.push_back(tf.transform()->rotation()->w());
  data_set_rot_ptr->select({ size, 0 }, { 1, 4 }).write(rot);

  // write the size as group attribute
  HighFive::Group group = m_file->getGroup(hdf5DatasetPath);
  if (!group.hasAttribute(SIZE))
    group.createAttribute(SIZE, ++size);
  else
    group.getAttribute(SIZE).write(++size);

  m_file->flush();
}

std::optional<std::vector<flatbuffers::Offset<seerep::fb::TransformStamped>>>
Hdf5FbTf::readTransformStamped(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) || !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading " << hdf5GroupPath;

  // read size
  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  long unsigned int size;
  group_ptr->getAttribute(SIZE).read(size);
  if (size == 0)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << "tf data has size 0.";
    return std::nullopt;
  }

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  // read time
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTimePath));
  std::vector<std::vector<int64_t>> time;
  data_set_time_ptr->read(time);

  // read translation
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTransPath));
  std::vector<std::vector<double>> trans;
  data_set_trans_ptr->read(trans);

  // read rotation
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRotPath));
  std::vector<std::vector<double>> rot;
  data_set_rot_ptr->read(rot);

  // check if all have the right size
  if (time.size() != size || trans.size() != size || rot.size() != size)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "sizes of time (" << time.size() << "), translation (" << trans.size() << ") and rotation (" << rot.size()
        << ") not matching. Size expected by value in metadata (" << size << ")";
    return std::nullopt;
  }

  std::vector<flatbuffers::Offset<seerep::fb::TransformStamped>> tfs;
  flatbuffers::FlatBufferBuilder builder;
  seerep::fb::TransformStampedBuilder tfBuilder(builder);
  for (long unsigned int i = 0; i < size; i++)
  {
    tfBuilder.add_child_frame_id(builder.CreateString(childframe));
    tfBuilder.add_header(seerep::fb::CreateHeader(
        builder, 0, seerep::fb::CreateTimestamp(builder, time.at(i).at(0), time.at(i).at(1)),
        builder.CreateString(parentframe), builder.CreateString(""), builder.CreateString("")));

    tfBuilder.add_transform(seerep::fb::CreateTransform(
        builder, seerep::fb::CreateVector3(builder, trans.at(i).at(0), trans.at(i).at(1), trans.at(i).at(2)),
        seerep::fb::CreateQuaternion(builder, rot.at(i).at(0), rot.at(i).at(1), rot.at(i).at(2))));

    tfs.push_back(tfBuilder.Finish());
  }
  return tfs;
}

std::optional<std::vector<std::string>> Hdf5FbTf::readTransformStampedFrames(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;

  if (!m_file->exist(hdf5GroupPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading parent frame of " << hdf5GroupPath;

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  return std::vector<std::string>{ parentframe, childframe };
}

}  // namespace seerep_hdf5_fb
