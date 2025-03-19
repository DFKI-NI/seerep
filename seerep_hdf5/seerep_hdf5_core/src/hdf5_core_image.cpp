#include "seerep_hdf5_core/hdf5_core_image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreImage::Hdf5CoreImage(std::shared_ptr<HighFive::File>& file,
                             std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
  m_ioCI = std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(
      file, write_mtx);
}

std::optional<seerep_core_msgs::DatasetIndexable>
Hdf5CoreImage::readDataset(const boost::uuids::uuid& uuid)
{
  return readDataset(boost::lexical_cast<std::string>(uuid));
}

std::optional<seerep_core_msgs::DatasetIndexable>
Hdf5CoreImage::readDataset(const std::string& uuid)
{
  seerep_core_msgs::DatasetIndexable data;
  std::string camintrinsics_uuid;

  {
    // perform the read of the dataset in this scope to release the lock as this
    // scope ends the lock needs to be released to allow for the readout of
    // camera intrinsics
    const std::scoped_lock lock(*m_write_mtx);

    std::string hdf5DataGroupPath = getHdf5GroupPath(uuid);

    auto dataGroupPtr = getHdf5Group(hdf5DataGroupPath);

    if (!dataGroupPtr)
    {
      return std::nullopt;
    }

    data.header.datatype = seerep_core_msgs::Datatype::Image;

    readHeader(uuid, *dataGroupPtr, data.header);

    boost::uuids::string_generator gen;
    boost::uuids::uuid uuid_generated = gen(uuid);
    data.header.uuidData = uuid_generated;

    readLabelsAndAddToLabelsPerCategory(HDF5_GROUP_IMAGE, uuid,
                                        data.labelsCategory);

    // fetch cam intrinsics uuid from hdf5_core_cameraintrinsics
    camintrinsics_uuid = readAttributeFromHdf5<std::string>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::CAMERA_INTRINSICS_UUID,
        hdf5DataGroupPath);
  }
  // lock released

  computeFrustumBB(camintrinsics_uuid, data.boundingbox);

  return data;
}

std::vector<std::string> Hdf5CoreImage::getDatasetUuids()
{
  return getGroupDatasets(HDF5_GROUP_IMAGE);
}

std::optional<seerep_core_msgs::TimestampFrameMesh>
Hdf5CoreImage::getPolygonConstraintMesh(const boost::uuids::uuid& uuid_entry)
{
  std::string uuid_str = boost::uuids::to_string(uuid_entry);
  std::string hdf5DataGroupPath = getHdf5GroupPath(uuid_str);

  auto dataGroupPtr = getHdf5Group(hdf5DataGroupPath);

  if (dataGroupPtr == nullptr)
  {
    throw std::invalid_argument("Hdf5DatasetPath not present for uuid " +
                                uuid_str + "! Skipping precise check...");
    BOOST_LOG_SEV(this->m_logger, boost::log::trivial::severity_level::warning);
    return std::nullopt;
  }

  // fetch the camera_intrinsics directly
  auto camintrinsics_uuid = readAttributeFromHdf5<std::string>(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::CAMERA_INTRINSICS_UUID,
      hdf5DataGroupPath);

  auto frame_id = readAttributeFromHdf5<std::string>(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::HEADER_FRAME_ID,
      hdf5DataGroupPath);

  // retrieve timestamp from image
  auto stamp_seconds = readAttributeFromHdf5<int>(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::HEADER_STAMP_SECONDS,
      hdf5DataGroupPath);

  auto stamp_nanos = readAttributeFromHdf5<int>(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::HEADER_STAMP_NANOS,
      hdf5DataGroupPath);

  seerep_core_msgs::Timestamp ts{ stamp_seconds,
                                  static_cast<uint32_t>(stamp_nanos) };

  auto points = this->computeFrustumPoints(camintrinsics_uuid);
  auto mesh = this->computeFrustumMesh(points);

  return seerep_core_msgs::TimestampFrameMesh{ ts, frame_id, mesh };
}

void Hdf5CoreImage::writeLabels(
    const std::string& uuid,
    const std::vector<seerep_core_msgs::LabelCategory>& labelCategory)
{
  Hdf5CoreGeneral::writeLabels(
      seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, uuid, labelCategory);
}

void Hdf5CoreImage::writeImageAttributes(HighFive::Group& dataset,
                                         const ImageAttributes& attributes)
{
  writeAttributeToHdf5<uint32_t>(
      dataset, seerep_hdf5_core::Hdf5CoreImage::HEIGHT, attributes.height);
  writeAttributeToHdf5<uint32_t>(
      dataset, seerep_hdf5_core::Hdf5CoreImage::WIDTH, attributes.width);
  writeAttributeToHdf5<std::string>(
      dataset, seerep_hdf5_core::Hdf5CoreImage::ENCODING, attributes.encoding);
  writeAttributeToHdf5<bool>(dataset,
                             seerep_hdf5_core::Hdf5CoreImage::IS_BIGENDIAN,
                             attributes.isBigendian);
  writeAttributeToHdf5<uint32_t>(dataset, seerep_hdf5_core::Hdf5CoreImage::STEP,
                                 attributes.step);
  writeAttributeToHdf5<std::string>(
      dataset, seerep_hdf5_core::Hdf5CoreImage::CAMERA_INTRINSICS_UUID,
      attributes.cameraIntrinsicsUuid);
}

ImageAttributes Hdf5CoreImage::readImageAttributes(HighFive::Group& group)
{
  ImageAttributes attributes;

  attributes.height = readAttributeFromHdf5<uint32_t>(
      group, seerep_hdf5_core::Hdf5CoreImage::HEIGHT, "");
  attributes.width = readAttributeFromHdf5<uint32_t>(
      group, seerep_hdf5_core::Hdf5CoreImage::WIDTH, "");
  attributes.encoding = readAttributeFromHdf5<std::string>(
      group, seerep_hdf5_core::Hdf5CoreImage::ENCODING, "");
  attributes.isBigendian = readAttributeFromHdf5<bool>(
      group, seerep_hdf5_core::Hdf5CoreImage::IS_BIGENDIAN, "");
  attributes.step = readAttributeFromHdf5<uint32_t>(
      group, seerep_hdf5_core::Hdf5CoreImage::STEP, "");
  attributes.cameraIntrinsicsUuid = readAttributeFromHdf5<std::string>(
      group, seerep_hdf5_core::Hdf5CoreImage::CAMERA_INTRINSICS_UUID, "");
  return attributes;
}

const std::string Hdf5CoreImage::getHdf5GroupPath(const std::string& id) const
{
  return HDF5_GROUP_IMAGE + "/" + id;
}

const std::string Hdf5CoreImage::getHdf5DataSetPath(const std::string& id) const
{
  return getHdf5GroupPath(id) + "/" + RAWDATA;
}

std::array<seerep_core_msgs::Point, 5>
Hdf5CoreImage::computeFrustumPoints(const std::string& camintrinsics_uuid)
{
  seerep_core_msgs::camera_intrinsics ci = m_ioCI->readCameraIntrinsics(
      boost::lexical_cast<boost::uuids::uuid>(camintrinsics_uuid));

  double far_plane_dist = ci.maximum_viewing_distance;

  double far_fov_x = (far_plane_dist * ci.height) / ci.intrinsic_matrix[0];
  double far_fov_y = (far_plane_dist * ci.width) / ci.intrinsic_matrix[4];

  seerep_core_msgs::Point near_p{ 0, 0, 0 };
  seerep_core_msgs::Point far_topleft{ static_cast<float>(far_fov_x / 2),
                                       static_cast<float>(far_fov_y / 2),
                                       static_cast<float>(far_plane_dist) };

  seerep_core_msgs::Point far_topright{ static_cast<float>(-far_fov_x / 2),
                                        static_cast<float>(far_fov_y / 2),
                                        static_cast<float>(far_plane_dist) };

  seerep_core_msgs::Point far_bottomleft{ static_cast<float>(far_fov_x / 2),
                                          static_cast<float>(-far_fov_y / 2),
                                          static_cast<float>(far_plane_dist) };

  seerep_core_msgs::Point far_bottomright{ static_cast<float>(-far_fov_x / 2),
                                           static_cast<float>(-far_fov_y / 2),
                                           static_cast<float>(far_plane_dist) };

  return std::array<seerep_core_msgs::Point, 5>{ near_p, far_topleft,
                                                 far_topright, far_bottomleft,
                                                 far_bottomright };
}

CGSurfaceMesh
Hdf5CoreImage::computeFrustumMesh(std::array<seerep_core_msgs::Point, 5>& points)
{
  CGSurfaceMesh mesh;
  auto o = mesh.add_vertex(
      CGPoint_3{ points[0].get<0>(), points[0].get<1>(), points[0].get<2>() });

  auto tl = mesh.add_vertex(
      CGPoint_3{ points[1].get<0>(), points[1].get<1>(), points[1].get<2>() });

  auto tr = mesh.add_vertex(
      CGPoint_3{ points[2].get<0>(), points[2].get<1>(), points[2].get<2>() });

  auto bl = mesh.add_vertex(
      CGPoint_3{ points[3].get<0>(), points[3].get<1>(), points[3].get<2>() });

  auto br = mesh.add_vertex(
      CGPoint_3{ points[4].get<0>(), points[4].get<1>(), points[4].get<2>() });

  std::vector<CGSurfaceMesh::Face_index> descriptors;

  descriptors.push_back(mesh.add_face(o, tl, tr));
  descriptors.push_back(mesh.add_face(o, tr, br));
  descriptors.push_back(mesh.add_face(o, br, bl));
  descriptors.push_back(mesh.add_face(o, bl, tl));
  // for whatever reason the face needs to be constructed in this way,
  // when the add_face returns a null_face() ALWAYS try reversing the vertex
  // order
  descriptors.push_back(mesh.add_face(bl, br, tr, tl));

  // check if any of the faces was not constructed properly
  if (std::find_if(descriptors.begin(), descriptors.end(), [](auto elem) {
        return elem == CGSurfaceMesh::null_face();
      }) != descriptors.end())
  {
    throw std::invalid_argument("Could not create the faces for the "
                                "SurfaceMesh from the camera frustum points!");
  }

  return mesh;
}

void Hdf5CoreImage::computeFrustumBB(const std::string& camintrinsics_uuid,
                                     seerep_core_msgs::AABB& bb)
{
  seerep_core_msgs::camera_intrinsics ci = m_ioCI->readCameraIntrinsics(
      boost::lexical_cast<boost::uuids::uuid>(camintrinsics_uuid));

  // compute frustrum
  double object_dist = ci.maximum_viewing_distance;

  double field_of_view_x = object_dist * ci.height / ci.intrinsic_matrix[0];
  double field_of_view_y = object_dist * ci.width / ci.intrinsic_matrix[4];

  // generate bounding box
  bb.min_corner().set<0>(field_of_view_x / -2);
  bb.min_corner().set<1>(field_of_view_y / -2);
  bb.min_corner().set<2>(0);
  bb.max_corner().set<0>(field_of_view_x / 2);
  bb.max_corner().set<1>(field_of_view_y / 2);
  bb.max_corner().set<2>(object_dist);
}
}  // namespace seerep_hdf5_core
