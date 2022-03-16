#include "seerep-io-pb/io-pb-tf.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_io_pb
{
IoPbTf::IoPbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : IoPbGeneral(file, write_mtx)
{
}

void IoPbTf::writeTransformStamped(const seerep::TransformStamped& tf)
{
  std::string hdf5DatasetPath = HDF5_GROUP_TF + "/" + tf.header().frame_id() + "_" + tf.child_frame_id();
  std::string hdf5DatasetTimePath = hdf5DatasetPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5DatasetPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5DatasetPath + "/" + "rotation";

  std::shared_ptr<HighFive::DataSet> data_set_time_ptr, data_set_trans_ptr, data_set_rot_ptr;
  uint64_t size = 0;

  if (!m_file->exist(hdf5DatasetPath))
  {
    std::cout << "data id " << hdf5DatasetPath << " does not exist! Creat new dataset in hdf5" << std::endl;
    HighFive::Group group = m_file->createGroup(hdf5DatasetPath);
    group.createAttribute("CHILD_FRAME", tf.child_frame_id());
    group.createAttribute("PARENT_FRAME", tf.header().frame_id());

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
    std::cout << "data id " << hdf5DatasetPath << " already exists!" << std::endl;
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
  time.push_back(tf.header().stamp().seconds());
  time.push_back(tf.header().stamp().nanos());
  data_set_time_ptr->select({ size, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.transform().translation().x());
  trans.push_back(tf.transform().translation().y());
  trans.push_back(tf.transform().translation().z());
  data_set_trans_ptr->select({ size, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.transform().rotation().x());
  rot.push_back(tf.transform().rotation().y());
  rot.push_back(tf.transform().rotation().z());
  rot.push_back(tf.transform().rotation().w());
  data_set_rot_ptr->select({ size, 0 }, { 1, 4 }).write(rot);

  // write the size as group attribute
  HighFive::Group group = m_file->getGroup(hdf5DatasetPath);
  if (!group.hasAttribute(SIZE))
    group.createAttribute(SIZE, ++size);
  else
    group.getAttribute(SIZE).write(++size);

  m_file->flush();
}

std::optional<std::vector<seerep::TransformStamped>> IoPbTf::readTransformStamped(const std::string& id)
{
  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) || !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }

  std::cout << "loading " << hdf5GroupPath << std::endl;

  // read size
  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  int size;
  group_ptr->getAttribute(SIZE).read(size);
  if (size == 0)
  {
    std::cout << "tf data has size 0." << std::endl;
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
    std::cout << "sizes of time (" << time.size() << "), translation (" << trans.size() << ") and rotation ("
              << rot.size() << ") not matching. Size expected by value in metadata (" << size << ")" << std::endl;
    return std::nullopt;
  }

  std::vector<seerep::TransformStamped> tfs;
  for (int i = 0; i < size; i++)
  {
    seerep::TransformStamped tf;
    tf.mutable_header()->set_frame_id(parentframe);
    tf.set_child_frame_id(childframe);

    tf.mutable_header()->mutable_stamp()->set_seconds(time.at(i).at(0));
    tf.mutable_header()->mutable_stamp()->set_nanos(time.at(i).at(1));

    seerep::Vector3 translation;
    translation.set_x(trans.at(i).at(0));
    translation.set_y(trans.at(i).at(1));
    translation.set_z(trans.at(i).at(2));
    *tf.mutable_transform()->mutable_translation() = translation;

    seerep::Quaternion rotation;
    rotation.set_x(rot.at(i).at(0));
    rotation.set_y(rot.at(i).at(1));
    rotation.set_z(rot.at(i).at(2));
    rotation.set_w(rot.at(i).at(3));
    *tf.mutable_transform()->mutable_rotation() = rotation;

    tfs.push_back(tf);
  }
  return tfs;
}

std::optional<std::vector<std::string>> IoPbTf::readTransformStampedFrames(const std::string& id)
{
  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;

  if (!m_file->exist(hdf5GroupPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  std::cout << "loading parent frame of " << hdf5GroupPath << std::endl;

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  return std::vector<std::string>{ parentframe, childframe };
}

} /* namespace seerep_io_pb */
