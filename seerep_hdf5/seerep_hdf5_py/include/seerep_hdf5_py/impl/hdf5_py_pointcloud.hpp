namespace seerep_hdf5_py
{

template <typename T>
bool Hdf5PyPointCloud::checkType(const py::dtype& type)
{
  // compare dtype with templated type T
  return type.equal(py::dtype::of<T>());
}

template <typename T>
bool Hdf5PyPointCloud::getFieldData(
    const std::vector<std::string>& fieldNames,
    const std::map<std::string, py::array>& fields,
    std::vector<std::vector<std::vector<T>>>& fieldData)
{
  // make sure that all fields exist
  for (const auto& fieldName : fieldNames)
  {
    auto search = fields.find(fieldName);
    if (search == fields.end())
    {
      return false;
    }
  }

  // combine fields into single field
  fieldData.clear();
  fieldData.resize(1);
  for (const auto& fieldName : fieldNames)
  {
    auto search = fields.find(fieldName);
    py::buffer_info fieldBuffInfo = search->second.request();

    // check type
    if (!checkType<T>(search->second.dtype()))
    {
      fieldData.clear();
      return false;
    }

    // cast data for easy access
    const py::array_t<T>& data = static_cast<py::array_t<T>>(search->second);

    if (fieldData[0].empty())
    {
      // reserve space for first field
      fieldData[0].resize(fieldBuffInfo.shape[0]);
    }
    else if ((std::size_t)fieldBuffInfo.shape[0] != fieldData[0].size())
    {
      // not matching field sizes
      fieldData.clear();
      return false;
    }

    for (unsigned int i = 0; i < fieldBuffInfo.shape[0]; i++)
    {
      if (fieldBuffInfo.shape.size() > 1)
      {
        for (unsigned int j = 0; j < fieldBuffInfo.shape[1]; j++)
        {
          fieldData[0][i].push_back(data.at(i, j));
        }
      }
      else
      {
        fieldData[0][i].push_back(data.at(i));
      }
    }
  }

  return true;
}

template <typename T, int Nfields>
void Hdf5PyPointCloud::getMinMax(
    const std::vector<std::vector<std::vector<T>>>& fieldData,
    std::array<T, Nfields>& min, std::array<T, Nfields>& max)
{
  for (std::size_t i = 0; i < Nfields; i++)
  {
    // TODO check if min needs to be replaced by lowest for, due to floating point types
    min[i] = std::numeric_limits<T>::max();
    max[i] = std::numeric_limits<T>::min();

    for (const auto& dat : fieldData[0])
    {
      min[i] = std::min(dat[i], min[i]);
      max[i] = std::max(dat[i], max[i]);
    }
  }
}

template <typename T, int Nfields>
bool Hdf5PyPointCloud::writeBoundingBox(
    const std::string& cloudGroupId, const std::vector<std::string>& fieldNames,
    const std::map<std::string, py::array>& fields)
{
  std::vector<std::vector<std::vector<T>>> fieldData(0);

  // find fields to use
  getFieldData<T>(fieldNames, fields, fieldData);

  if (fieldData.size() == 0)
  {
    // no fields found
    return false;
  }

  std::array<T, Nfields> min, max;
  getMinMax<T, Nfields>(fieldData, min, max);

  std::shared_ptr<HighFive::Group> cloudGroupPtr =
      std::make_shared<HighFive::Group>(m_file->getGroup(cloudGroupId));

  // write bounding box as attribute to dataset
  std::vector<T> boundingbox(2 * Nfields);
  for (std::size_t i = 0; i < Nfields; i++)
  {
    boundingbox[i] = min[i];
    boundingbox[i + Nfields] = max[i];
  }

  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(
      *cloudGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX,
      boundingbox);

  return true;
}

template <typename T>
bool Hdf5PyPointCloud::writeFieldTyped(
    const std::string& cloudGroupId, const std::string& fieldDatasetId,
    const std::string& fieldName,
    const std::map<std::string, py::array>& fields)
{
  std::vector<std::vector<std::vector<T>>> fieldData(0);

  // find fields to use
  getFieldData<T>({ fieldName }, fields, fieldData);

  if (fieldData.size() == 0)
  {
    // no fields found
    return false;
  }

  // create dataset
  const std::string datasetId = cloudGroupId + "/" + fieldDatasetId;
  HighFive::DataSpace dataSpace(
      { fieldData.size(), fieldData[0].size(), fieldData[0][0].size() });

  std::shared_ptr<HighFive::DataSet> datasetPtr;
  if (!m_file->exist(datasetId))
  {
    datasetPtr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<T>(datasetId, dataSpace));
  }
  else
  {
    datasetPtr =
        std::make_shared<HighFive::DataSet>(m_file->getDataSet(datasetId));
  }

  // write data
  datasetPtr->write(fieldData);

  return true;
}

template <typename T, typename Second, typename... Other>
bool Hdf5PyPointCloud::writeFieldTyped(
    const std::string& cloudGroupId, const std::string& fieldDatasetId,
    const std::string& fieldName,
    const std::map<std::string, py::array>& fields)
{
  if (writeFieldTyped<T>(cloudGroupId, fieldDatasetId, fieldName, fields))
  {
    return true;
  }

  return writeFieldTyped<Second, Other...>(cloudGroupId, fieldDatasetId,
                                           fieldName, fields);
}

template <typename T>
py::array
Hdf5PyPointCloud::readField(std::shared_ptr<HighFive::DataSet> fieldDatasetPtr)
{
  std::vector<std::size_t> datasetDimensions =
      fieldDatasetPtr->getSpace().getDimensions();
  std::cout << datasetDimensions[0] << " " << datasetDimensions[1] << " "
            << datasetDimensions[2] << std::endl;

  // ignore hight for now -> organized pointclouds unsupported
  py::array field = py::array_t<T>(
      { datasetDimensions[0] * datasetDimensions[1], datasetDimensions[2] });

  std::vector<std::vector<std::vector<T>>> hdf5Data;
  fieldDatasetPtr->read(hdf5Data);

  py::buffer_info fieldBuffInfo = field.request();
  T* fieldBuffData = static_cast<T*>(fieldBuffInfo.ptr);

  for (std::size_t i = 0; i < datasetDimensions[0]; i++)
  {
    for (std::size_t j = 0; j < datasetDimensions[1]; j++)
    {
      for (std::size_t k = 0; k < datasetDimensions[2]; k++)
      {
        fieldBuffData[k + datasetDimensions[2] * (j + datasetDimensions[1] * i)] =
            hdf5Data[i][j][k];
      }
    }
  }

  return field;
}

} /* namespace seerep_hdf5_py */
