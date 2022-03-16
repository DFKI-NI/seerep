#ifndef SEEREP_IO_PB_IO_PB_IMAGE_H_
#define SEEREP_IO_PB_IO_PB_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-io-pb/io-pb-general.h"

// seerep-msgs
#include <seerep-msgs/image.pb.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_io_pb
{
class IoPbImage : public IoPbGeneral
{
public:
  IoPbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeImage(const std::string& id, const seerep::Image& image);

  std::optional<seerep::Image> readImage(const std::string& id);

private:
  const std::string SIZE = "size";
  const std::string CLASS = "CLASS";

  // image / pointcloud attribute keys
  inline static const std::string HEIGHT = "height";
  inline static const std::string WIDTH = "width";
  inline static const std::string ENCODING = "encoding";
  inline static const std::string IS_BIGENDIAN = "is_bigendian";
  inline static const std::string ROW_STEP = "row_step";
  inline static const std::string POINT_STEP = "point_step";
  inline static const std::string IS_DENSE = "is_dense";

  inline static const std::string RAWDATA = "rawdata";

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_IMAGE = "images";
};

} /* namespace seerep_io_pb */

#endif /* SEEREP_IO_PB_IO_PB_IMAGE_H_ */
