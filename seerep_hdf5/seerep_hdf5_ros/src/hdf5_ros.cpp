#include "seerep_hdf5_ros/hdf5_ros.h"

namespace seerep_hdf5_ros
{

Hdf5Ros::Hdf5Ros(const std::string& fileName, const std::string& projectName,
                 const std::string& projectFrameId)

  : mutex_(std::make_shared<std::mutex>())
  , file_(std::make_shared<HighFive::File>(
        fileName, HighFive::File::ReadWrite | HighFive::File::Create))
  , hdf5Core_(std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(file_, mutex_))
  , hdf5CoreTf_(std::make_shared<seerep_hdf5_core::Hdf5CoreTf>(file_, mutex_))
{
  hdf5Core_->writeProjectname(projectName);
  hdf5Core_->writeProjectFrameId(projectFrameId);
}

Hdf5Ros::~Hdf5Ros()
{
  file_->flush();
}

void Hdf5Ros::dump(const sensor_msgs::Image& image, const std::string& uuid)
{
  std::string uuid =
      boost::lexical_cast<std::string>(boost::uuids::random_generator()());
  const std::string path = "images/" + uuid;
  HighFive::DataSpace dataspace =
      HighFive::DataSpace(image.height * image.step);

  HighFive::Group group = *hdf5Core_->getHdf5Group(path);

  dump(path, image.header);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "height", image.height);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "width", image.width);
  hdf5Core_->writeAttributeToHdf5<std::string>(group, "encoding",
                                               image.encoding);
  hdf5Core_->writeAttributeToHdf5<bool>(group, "is_bigendian",
                                        image.is_bigendian);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "step", image.step);
  hdf5Core_->getHdf5DataSet<uint8_t>(path + "/rawdata", dataspace)
      ->write(std::move(image.data.data()));

  if (!uuid.empty())
  {
    hdf5Core_->writeAttributeToHdf5<std::string>(
        group, "camera_intrinsics_uuid", uuid);
  }

  return uuid;
}

void Hdf5Ros::dump(const sensor_msgs::CompressedImage& image)
{
  const std::string path = "images/" + boost::lexical_cast<std::string>(
                                           boost::uuids::random_generator()());
  HighFive::DataSpace dataspace = HighFive::DataSpace(image.data.size());
  HighFive::Group group = *hdf5Core_->getHdf5Group(path);

  dump(path, image.header);
  hdf5Core_->writeAttributeToHdf5<std::string>(group, "format", image.format);
  hdf5Core_->getHdf5DataSet<uint8_t>(path + "/rawdata", dataspace)
      ->write(std::move(image.data.data()));
}

std::string Hdf5Ros::dump(const sensor_msgs::CameraInfo& cameraInfo,
                          float maxViewingDistance)
{
  const std::string uuid =
      boost::lexical_cast<std::string>(boost::uuids::random_generator()());

  const std::string path = "cameraintrinsics/" + uuid;

  HighFive::Group group = *hdf5Core_->getHdf5Group(path);

  dump(path, cameraInfo.header);

  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "height", cameraInfo.height);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "width", cameraInfo.width);
  hdf5Core_->writeAttributeToHdf5<std::string>(group, "distortion_model",
                                               cameraInfo.distortion_model);

  std::vector<double> D(cameraInfo.D.data(),
                        cameraInfo.D.data() + cameraInfo.D.size());
  std::vector<double> K(cameraInfo.K.data(),
                        cameraInfo.K.data() + cameraInfo.K.size());
  std::vector<double> R(cameraInfo.R.data(),
                        cameraInfo.R.data() + cameraInfo.R.size());
  std::vector<double> P(cameraInfo.P.data(),
                        cameraInfo.P.data() + cameraInfo.P.size());

  hdf5Core_->writeAttributeToHdf5<std::vector<double>>(group, "distortion", D);
  hdf5Core_->writeAttributeToHdf5<std::vector<double>>(group,
                                                       "intrinsic_matrix", K);
  hdf5Core_->writeAttributeToHdf5<std::vector<double>>(
      group, "rectification_matrix", R);
  hdf5Core_->writeAttributeToHdf5<std::vector<double>>(group,
                                                       "projection_matrix", P);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "binning_x",
                                            cameraInfo.binning_x);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "binning_y",
                                            cameraInfo.binning_y);
  dump(path, cameraInfo.roi);

  hdf5Core_->writeAttributeToHdf5<float>(group, "max_viewing_distance",
                                         maxViewingDistance);

  return uuid;
}

void Hdf5Ros::dump(const std::string& groupPath,
                   const sensor_msgs::RegionOfInterest& roi)
{
  HighFive::Group group = *hdf5Core_->getHdf5Group(groupPath);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(
      group, "region_of_interest/x_offset", roi.x_offset);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(
      group, "region_of_interest/y_offset", roi.y_offset);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "region_of_interest/height",
                                            roi.height);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "region_of_interest/width",
                                            roi.width);
  hdf5Core_->writeAttributeToHdf5<bool>(group, "region_of_interest/do_rectify",
                                        roi.do_rectify);
}

void Hdf5Ros::dump(const std::string& groupPath, const std_msgs::Header& header)
{
  HighFive::Group group = *hdf5Core_->getHdf5Group(groupPath);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(group, "header_seq", header.seq);
  hdf5Core_->writeAttributeToHdf5<uint64_t>(group, "stamp_seconds",
                                            header.stamp.sec);
  hdf5Core_->writeAttributeToHdf5<uint64_t>(group, "stamp_nanos",
                                            header.stamp.nsec);
  hdf5Core_->writeAttributeToHdf5<std::string>(group, "frame_id",
                                               header.frame_id);
}

std::string Hdf5Ros::dump(const sensor_msgs::PointCloud2& pcl)
{
  std::string uuid =
      boost::lexical_cast<std::string>(boost::uuids::random_generator()());
  const std::string path = "pointclouds/" + uuid;
  HighFive::DataSpace dataspace = HighFive::DataSpace(pcl.data.size());
  HighFive::Group group = *hdf5Core_->getHdf5Group(path);

  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> types;
  for (const sensor_msgs::PointField& field : pcl.fields)
  {
    names.push_back(field.name);
    offsets.push_back(field.offset);
    counts.push_back(field.count);
    types.push_back(field.datatype);
  }

  dump(path, pcl.header);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, pcl.height);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, pcl.width);
  hdf5Core_->writeAttributeToHdf5<bool>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN,
      pcl.is_bigendian);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, pcl.point_step);
  hdf5Core_->writeAttributeToHdf5<uint32_t>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP, pcl.row_step);
  hdf5Core_->writeAttributeToHdf5<bool>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, pcl.is_dense);
  hdf5Core_->writeAttributeToHdf5<std::vector<std::string>>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);
  hdf5Core_->writeAttributeToHdf5<std::vector<uint32_t>>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, offsets);
  hdf5Core_->writeAttributeToHdf5<std::vector<uint32_t>>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);
  hdf5Core_->writeAttributeToHdf5<std::vector<uint8_t>>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE, types);
  auto [min_corner, max_corner] = computeBoundingBox(pcl);
  std::vector<float> bb{ min_corner.get<0>(), min_corner.get<1>(),
                         min_corner.get<2>(), max_corner.get<0>(),
                         max_corner.get<1>(), max_corner.get<2>() };
  hdf5Core_->writeAttributeToHdf5<std::vector<float>>(
      group, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX, bb);

  hdf5Core_->getHdf5DataSet<uint8_t>(path + "/points", dataspace)
      ->write(std::move(pcl.data.data()));

  return uuid;
}

void Hdf5Ros::dump(const tf2_msgs::TFMessage& tf2_msg, bool is_static)
{
  std::string hdf5_group_tf;
  if (is_static)
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF_STATIC;
  }
  else
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF;
  }
  for (const auto& transform : tf2_msg.transforms)
  {
    const std::string datagroupPath = hdf5_group_tf + "/" +
                                      transform.header.frame_id + "_" +
                                      transform.child_frame_id;

    std::unique_ptr<HighFive::Group> group;

    if (!file_->exist(datagroupPath))
    {
      group =
          std::make_unique<HighFive::Group>(file_->createGroup(datagroupPath));
      group->createAttribute<uint64_t>(seerep_hdf5_core::Hdf5CoreTf::SIZE, 0);
      hdf5Core_->writeAttributeToHdf5<std::string>(*group, "PARENT_FRAME",
                                                   transform.header.frame_id);
      hdf5Core_->writeAttributeToHdf5<std::string>(*group, "CHILD_FRAME",
                                                   transform.child_frame_id);
    }
    else
    {
      group = std::make_unique<HighFive::Group>(file_->getGroup(datagroupPath));
    }
    hdf5CoreTf_->writeTimestamp(datagroupPath, { transform.header.stamp.sec,
                                                 transform.header.stamp.nsec });
    hdf5CoreTf_->writeTranslation(datagroupPath,
                                  { transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z });
    hdf5CoreTf_->writeRotation(
        datagroupPath,
        { transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w });

    uint64_t size = hdf5Core_->readAttributeFromHdf5<uint64_t>(
        *group, seerep_hdf5_core::Hdf5CoreTf::SIZE, "");

    hdf5Core_->writeAttributeToHdf5<uint64_t>(
        *group, seerep_hdf5_core::Hdf5CoreTf::SIZE, size + 1);
  }
}

std::pair<seerep_core_msgs::Point, seerep_core_msgs::Point>
Hdf5Ros::computeBoundingBox(const sensor_msgs::PointCloud2& pcl)
{
  seerep_core_msgs::Point min_corner = { std::numeric_limits<float>::max(),
                                         std::numeric_limits<float>::max(),
                                         std::numeric_limits<float>::max() };
  seerep_core_msgs::Point max_corner = { std::numeric_limits<float>::lowest(),
                                         std::numeric_limits<float>::lowest(),
                                         std::numeric_limits<float>::lowest() };

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pcl, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pcl, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pcl, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    min_corner.set<0>(std::min(min_corner.get<0>(), *iter_x));
    min_corner.set<1>(std::min(min_corner.get<1>(), *iter_y));
    min_corner.set<2>(std::min(min_corner.get<2>(), *iter_z));

    max_corner.set<0>(std::max(max_corner.get<0>(), *iter_x));
    max_corner.set<1>(std::max(max_corner.get<1>(), *iter_y));
    max_corner.set<2>(std::max(max_corner.get<2>(), *iter_z));
  }
  return std::make_pair(min_corner, max_corner);
}

}  // namespace seerep_hdf5_ros
