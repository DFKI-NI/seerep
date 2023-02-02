#ifndef SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_
#define SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-general.h>

// std
#include <boost/geometry.hpp>
#include <filesystem>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_hdf5_py
{

class Hdf5FileWrapper
{
public:
  Hdf5FileWrapper(const std::string& filename)
    : file_ptr_(std::make_shared<HighFive::File>(filename, HighFive::File::ReadWrite | HighFive::File::Create))
    , write_mutex_(std::make_shared<std::mutex>())
  {
  }

  std::shared_ptr<HighFive::File>& getFile()
  {
    return file_ptr_;
  }

  std::shared_ptr<std::mutex>& getMutex()
  {
    return write_mutex_;
  }

private:
  std::shared_ptr<HighFive::File> file_ptr_;
  std::shared_ptr<std::mutex> write_mutex_;
};

struct InstanceLabel
{
public:
  InstanceLabel(const std::string& new_label, const std::string& new_instance_uuid)
    : label(new_label), instance_uuid(new_instance_uuid)
  {
  }

  std::string label = "";
  std::string instance_uuid = "";
};

struct GeneralLabel
{
public:
  GeneralLabel(const std::string& new_category) : category(new_category)
  {
  }

  std::string category = "";
  std::vector<InstanceLabel> labels;
};

template <int NumDimensions>
struct BoundingBoxLabel
{
public:
  BoundingBoxLabel(InstanceLabel& new_label, std::array<double, NumDimensions>& new_min_point,
                   std::array<double, NumDimensions>& new_max_point)
    : label(new_label), min_point(new_min_point), max_point(new_max_point)
  {
  }

  InstanceLabel label;
  std::array<double, NumDimensions> min_point;
  std::array<double, NumDimensions> max_point;
};

template <int NumDimensions>
struct CategorizedBoundingBoxLabel
{
public:
  CategorizedBoundingBoxLabel(const std::string& new_category) : category(new_category)
  {
  }

  std::string category = "";
  std::vector<BoundingBoxLabel<NumDimensions>> labels;
};

class Hdf5PyGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
protected:
  Hdf5PyGeneral(Hdf5FileWrapper& hdf5_file);

  // ################
  //  Attributes
  // ################
  //   template <class T>
  //   void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header);

  //   template <class T>
  //   seerep::Header readHeaderAttributes(HighFive::AnnotateTraits<T>& object, const std::string& id);

  // ################
  //  BoundingBoxes
  // ################
  //   void writeBoundingBoxLabeled(
  //       const std::string& datatypeGroup, const std::string& uuid,
  //       const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeledWithCategory>& boundingboxLabeled);

  //   void writeBoundingBox2DLabeled(
  //       const std::string& datatypeGroup, const std::string& uuid,
  //       const google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeledWithCategory>& boundingbox2DLabeled);

  //   std::optional<google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeledWithCategory>>
  //   readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid);

  // ################
  //  Labels General
  // ################
  //   void writeLabelsGeneral(
  //       const std::string& datatypeGroup, const std::string& uuid,
  //       const google::protobuf::RepeatedPtrField<seerep::LabelsWithInstanceWithCategory>& labelsGeneralWithInstances);

  //   std::optional<google::protobuf::RepeatedPtrField<seerep::LabelsWithInstanceWithCategory>>
  //   readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid);
};

}  // namespace seerep_hdf5_py

#include "impl/hdf5-py-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_ */
