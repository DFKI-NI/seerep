#ifndef SEEREP_HDF5_PY_HDF5_PY_IMAGE_H_
#define SEEREP_HDF5_PY_HDF5_PY_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-image.h>

#include "seerep-hdf5-py/hdf5-py-general.h"

// seerep-msgs
#include <seerep-msgs/image.pb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_py
{
// class Hdf5PyImage : public seerep_hdf5_core::Hdf5CoreImage, public Hdf5PyGeneral
// {
// public:
//   Hdf5PyImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

//   void writeImage(const std::string& id, const seerep::Image& image);

//   std::optional<seerep::Image> readImage(const std::string& id);
// };

}  // namespace seerep_hdf5_py

#endif /* SEEREP_HDF5_PY_HDF5_PY_IMAGE_H_ */
