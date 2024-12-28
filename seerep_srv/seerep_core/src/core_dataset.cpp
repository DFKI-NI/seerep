#include "seerep_core/core_dataset.h"

namespace seerep_core
{
CoreDataset::CoreDataset(
    std::shared_ptr<seerep_core::CoreTf> tfOverview,
    std::shared_ptr<seerep_core::CoreInstances> coreInstances,
    const std::string& frameId)
  : m_frameId(frameId), m_tfOverview(tfOverview), m_coreInstances(coreInstances)
{
}
CoreDataset::~CoreDataset()
{
}

void CoreDataset::addDatatype(
    const seerep_core_msgs::Datatype& datatype,
    std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io)
{
  DatatypeSpecifics datatypeSpecifics = {
    .hdf5io = hdf5Io,
    .dataWithMissingTF =
        std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>(),
    .rt = seerep_core_msgs::rtree(),
    .rtSensorPos = seerep_core_msgs::rtree(),
    .timetree = seerep_core_msgs::timetree(),
    .categoryLabelDatasetsMap = std::unordered_map<
        std::string,
        std::unordered_map<std::string, std::vector<boost::uuids::uuid>>>(),
    .datasetInstancesMap =
        std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>,
                           boost::hash<boost::uuids::uuid>>()
  };
  m_datatypeDatatypeSpecificsMap.emplace(
      datatype, std::make_shared<DatatypeSpecifics>(datatypeSpecifics));
  recreateDatasets(datatype, hdf5Io);
}

void CoreDataset::recreateDatasets(
    const seerep_core_msgs::Datatype& datatype,
    std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io)
{
  std::vector<std::string> datasets = hdf5Io->getDatasetUuids();
  for (auto uuid : datasets)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "found " << uuid << " in HDF5 file.";

    try
    {
      std::optional<seerep_core_msgs::DatasetIndexable> dataset =
          hdf5Io->readDataset(uuid);

      if (dataset)
      {
        addDatasetToIndices(datatype, dataset.value());
      }
    }
    catch (const std::runtime_error& e)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << e.what();
    }
  }
}

void CoreDataset::recreateSpatialRt(
    const seerep_core_msgs::Datatype& datatype,
    std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io)
{
  std::vector<std::string> datasets = hdf5Io->getDatasetUuids();
  auto datatypeSpecifics = m_datatypeDatatypeSpecificsMap.at(datatype);
  std::vector<seerep_core_msgs::AabbIdPair> insertableData;
  std::vector<seerep_core_msgs::AabbIdPair> insertableDataSensorPos;

  for (auto uuid : datasets)
  {
    try
    {
      std::optional<seerep_core_msgs::DatasetIndexable> dataset =
          hdf5Io->readDataset(uuid);

      if (dataset)
      {
        if (isSpatiallyTransformable(dataset.value()))
        {
          insertableData.push_back(
              { transformIndexableAABB(dataset.value()),
                boost::lexical_cast<boost::uuids::uuid>(uuid) });
          insertableDataSensorPos.push_back(
              { getSensorPositionAsAABB(dataset.value()),
                boost::lexical_cast<boost::uuids::uuid>(uuid) });
        }
        else
        {
          datatypeSpecifics->dataWithMissingTF.push_back(
              std::make_shared<seerep_core_msgs::DatasetIndexable>(
                  dataset.value()));
        }
      }
    }
    catch (const std::runtime_error& e)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << e.what();
    }
  }
  // recreate spatial rtree
  datatypeSpecifics->rt =
      seerep_core_msgs::rtree{ insertableData.begin(), insertableData.end() };
  datatypeSpecifics->rtSensorPos =
      seerep_core_msgs::rtree{ insertableDataSensorPos.begin(),
                               insertableDataSensorPos.end() };
}

std::vector<boost::uuids::uuid>
CoreDataset::getData(const seerep_core_msgs::Query& query)
{
  std::vector<boost::uuids::uuid> result;

  // check if the data has now some tf for adding it to spatial rtree
  tryAddingDataWithMissingTF(query.header.datatype);

  auto datatypeSpecifics =
      m_datatypeDatatypeSpecificsMap.at(query.header.datatype);
  // space
  auto resultRt = querySpatial(datatypeSpecifics, query);
  auto resultRtSensorPos = querySpatialSensorPos(datatypeSpecifics, query);
  // time
  auto resultTime = queryTemporal(datatypeSpecifics, query);
  // semantic
  auto resultSemantic = querySemantic(datatypeSpecifics, query);

  // instances
  std::optional<std::vector<boost::uuids::uuid>> instanceResult;
  if (query.instances)
  {
    auto instances = m_coreInstances->getDatasets(query.instances.value(),
                                                  query.header.datatype);
    if (!instances.empty())
    {
      instanceResult = instances;
    }
  }

  if (!query.polygon && !query.polygonSensorPos && !query.timeintervals &&
      !query.label && !query.instances && !query.dataUuids)
  {
    return getAllDatasetUuids(datatypeSpecifics, query.sortByTime);
  }

  return intersectQueryResults(resultRt, resultRtSensorPos, resultTime,
                               resultSemantic, instanceResult, query.dataUuids,
                               query.sortByTime);
}

std::vector<boost::uuids::uuid>
CoreDataset::getInstances(const seerep_core_msgs::Query& query)
{
  std::vector<boost::uuids::uuid> result;
  if (query.header.datatype == seerep_core_msgs::Datatype::Unknown)
  {
    auto queryLocal = query;
    for (auto& datatypeSpecifics : m_datatypeDatatypeSpecificsMap)
    {
      queryLocal.header.datatype = datatypeSpecifics.first;
      auto datasets = getData(queryLocal);
      getUuidsFromMap(datatypeSpecifics.second->datasetInstancesMap, datasets,
                      result);
    }
  }
  else
  {
    auto datasets = getData(query);
    getUuidsFromMap(m_datatypeDatatypeSpecificsMap.at(query.header.datatype)
                        ->datasetInstancesMap,
                    datasets, result);
  }
  return result;
}

void CoreDataset::getUuidsFromMap(
    std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>,
                       boost::hash<boost::uuids::uuid>>& datasetInstancesMap,
    std::vector<boost::uuids::uuid>& datasets,
    std::vector<boost::uuids::uuid>& result)
{
  std::set<boost::uuids::uuid> instances;
  for (auto dataset : datasets)
  {
    // find the dataset in the dataset-instances-map
    auto resultPtr = datasetInstancesMap.find(dataset);

    if (resultPtr != datasetInstancesMap.end())
    {
      // add instances of dataset to instances set
      std::copy(resultPtr->second.begin(), resultPtr->second.end(),
                std::inserter(instances, instances.end()));
    }
  }

  std::copy(instances.begin(), instances.end(), std::back_inserter(result));
}

seerep_core_msgs::AABB
CoreDataset::polygonToAABB(const seerep_core_msgs::Polygon2D& polygon)
{
  seerep_core_msgs::Point min, max;

  bg::set<0>(min, std::numeric_limits<float>::max());
  bg::set<0>(max, std::numeric_limits<float>::lowest());

  bg::set<1>(min, std::numeric_limits<float>::max());
  bg::set<1>(max, std::numeric_limits<float>::lowest());

  bg::set<2>(min, polygon.z);
  bg::set<2>(max, polygon.height + polygon.z);

  for (auto p : polygon.vertices)
  {
    // update x dimension of min
    if (bg::get<0>(p) < bg::get<0>(min))
    {
      bg::set<0>(min, bg::get<0>(p));
    }

    // update y dimension of min
    if (bg::get<1>(p) < bg::get<1>(min))
    {
      bg::set<1>(min, bg::get<1>(p));
    }

    // update x dimension of max
    if (bg::get<0>(p) > bg::get<0>(max))
    {
      bg::set<0>(max, bg::get<0>(p));
    }

    // update y dimension of max
    if (bg::get<1>(p) > bg::get<1>(max))
    {
      bg::set<1>(max, bg::get<1>(p));
    }
  }

  // generate aabb from provided polygon
  seerep_core_msgs::AABB aabb(min, max);

  return aabb;
}

std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
CoreDataset::querySpatial(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                          const seerep_core_msgs::Query& query)
{
  if (query.polygon)
  {
    return queryRtree(datatypeSpecifics->rt, *datatypeSpecifics->hdf5io,
                      query.polygon.value(), query.fullyEncapsulated);
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
CoreDataset::querySpatialSensorPos(
    std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
    const seerep_core_msgs::Query& query)
{
  if (query.polygonSensorPos)
  {
    return queryRtree(datatypeSpecifics->rtSensorPos,
                      *datatypeSpecifics->hdf5io,
                      query.polygonSensorPos.value(), query.fullyEncapsulated);
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<std::vector<seerep_core_msgs::AabbIdPair>> CoreDataset::queryRtree(
    const seerep_core_msgs::rtree& rt,
    seerep_hdf5_core::Hdf5CoreDatatypeInterface& coreDatatype,
    const seerep_core_msgs::Polygon2D& polygon,
    const bool queryFullyEncapsulated)
{
  // generate rtree result container
  std::optional<std::vector<seerep_core_msgs::AabbIdPair>> rt_result =
      std::vector<seerep_core_msgs::AabbIdPair>();

  seerep_core_msgs::AABB aabb = polygonToAABB(polygon);

  // perform the query on the r tree using the aabb
  rt.query(boost::geometry::index::intersects(aabb),
           std::back_inserter(rt_result.value()));

  bool fullyEncapsulated = false;
  bool partiallyEncapsulated = false;
  std::vector<seerep_core_msgs::AabbIdPair>::iterator it;

  // traverse query results and confirm if they are contained inside the polygon
  for (it = rt_result.value().begin(); it != rt_result.value().end();)
  {
    // check if query result is not fully distant to the oriented bb box
    // provided in the query
    intersectionDegree(it->first, polygon, fullyEncapsulated,
                       partiallyEncapsulated);

    auto ts_frame_points = coreDatatype.getPolygonConstraintMesh(it->second);
    if (ts_frame_points.has_value())
    {
      CGSurfaceMesh& mesh = ts_frame_points->mesh;
      std::vector<std::reference_wrapper<CGPoint_3>> ref_points;

      for (auto& idx : mesh.vertices())
      {
        ref_points.push_back(mesh.point(idx));
      }

      // transform points from another coordinate system to the map frame of the project
      auto points_vec =
          m_tfOverview->transform(ts_frame_points->frame_id, m_frameId,
                                  ts_frame_points->timestamp.seconds,
                                  ts_frame_points->timestamp.nanos, ref_points);

      for (auto& idx : mesh.vertices())
      {
        mesh.point(idx) = points_vec[idx.idx()];
      }

      // check if these point are enclosed by the query polygon
      checkIntersectionWithZExtrudedPolygon(mesh, polygon, fullyEncapsulated,
                                            partiallyEncapsulated);
    }

    // if there is no intersection between the result and the user's
    // request, remove it from the iterator
    if (!partiallyEncapsulated && !fullyEncapsulated)
    {
      it = rt_result.value().erase(it);
    }

    // does the user want only results fully encapsulated by the requested
    // bb and is this query result not fully encapsulated
    else if (queryFullyEncapsulated && !fullyEncapsulated)
    {
      // if yes, then delete partially encapsulated query results from the result set
      it = rt_result.value().erase(it);
    }
    else
    {
      ++it;
    }
  }

  // return the result after deletion of undesired elements has been performed
  return rt_result;
}

std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>
CoreDataset::queryTemporal(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                           const seerep_core_msgs::Query& query)
{
  if (query.timeintervals)
  {
    std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>> timetree_result =
        std::vector<seerep_core_msgs::AabbTimeIdPair>();
    for (auto timeinterval : query.timeintervals.value())
    {
      seerep_core_msgs::AabbTime aabbtime(
          seerep_core_msgs::TimePoint(((int64_t)timeinterval.timeMin.seconds)
                                          << 32 |
                                      ((uint64_t)timeinterval.timeMin.nanos)),
          seerep_core_msgs::TimePoint(((int64_t)timeinterval.timeMax.seconds)
                                          << 32 |
                                      ((uint64_t)timeinterval.timeMax.nanos)));

      datatypeSpecifics->timetree.query(
          boost::geometry::index::intersects(aabbtime),
          std::back_inserter(timetree_result.value()));
    }
    return timetree_result;
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<std::set<boost::uuids::uuid>>
CoreDataset::querySemantic(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                           const seerep_core_msgs::Query& query)
{
  if (query.label)
  {
    if (query.mustHaveAllLabels)
    {
      return querySemanticWithAllTheLabels(datatypeSpecifics, query);
    }
    else
    {
      return querySemanticWithAnyOfLabels(datatypeSpecifics, query);
    }
  }

  return std::nullopt;
}

std::set<boost::uuids::uuid> CoreDataset::querySemanticWithAnyOfLabels(
    std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
    const seerep_core_msgs::Query& query)
{
  std::set<boost::uuids::uuid> result = std::set<boost::uuids::uuid>();
  // find the queried label in the label-imageID-map
  for (auto categorylabelquery : query.label.value())
  {
    auto& category = categorylabelquery.first;
    auto categoryPtr =
        datatypeSpecifics->categoryLabelDatasetsMap.find(category);
    if (categoryPtr != datatypeSpecifics->categoryLabelDatasetsMap.end())
    {
      for (std::string label : categorylabelquery.second)
      {
        auto labelPtr = categoryPtr->second.find(label);
        if (labelPtr != categoryPtr->second.end())
        {
          // add all imageIDs to result set
          for (boost::uuids::uuid id : labelPtr->second)
          {
            result.insert(id);
          }
        }
      }
    }
  }
  return result;
}

std::set<boost::uuids::uuid> CoreDataset::querySemanticWithAllTheLabels(
    std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
    const seerep_core_msgs::Query& query)
{
  std::set<boost::uuids::uuid> result = std::set<boost::uuids::uuid>();
  ;
  bool firstLabel = true;
  // find the queried label in the label-imageID-map
  for (auto categorylabelquery : query.label.value())
  {
    auto& category = categorylabelquery.first;
    for (std::string label : categorylabelquery.second)
    {
      auto categoryPtr =
          datatypeSpecifics->categoryLabelDatasetsMap.find(category);
      if (categoryPtr != datatypeSpecifics->categoryLabelDatasetsMap.end())
      {
        auto labelPtr = categoryPtr->second.find(label);
        if (labelPtr != categoryPtr->second.end())
        {
          std::set<boost::uuids::uuid> thisLabelSet;
          // vector to set
          for (boost::uuids::uuid id : labelPtr->second)
          {
            thisLabelSet.insert(id);
          }

          if (firstLabel)
          {
            result = std::move(thisLabelSet);
            firstLabel = false;
          }
          else
          {
            std::set<boost::uuids::uuid> intersection;
            std::set_intersection(result.begin(), result.end(),
                                  thisLabelSet.begin(), thisLabelSet.end(),
                                  std::inserter(intersection,
                                                intersection.begin()));
            result = std::move(intersection);
          }
        }
        else
        {
          return std::set<boost::uuids::uuid>();
        }
        firstLabel = false;
        if (result.empty())
        {
          return std::set<boost::uuids::uuid>();
        }
      }
      else
      {
        return std::set<boost::uuids::uuid>();
      }
    }
  }
  return result;
}

std::vector<boost::uuids::uuid> CoreDataset::intersectQueryResults(
    std::optional<std::vector<seerep_core_msgs::AabbIdPair>>& rt_result,
    std::optional<std::vector<seerep_core_msgs::AabbIdPair>>& rt_resultSensorPos,
    std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>& timetree_result,
    std::optional<std::set<boost::uuids::uuid>>& semanticResult,
    std::optional<std::vector<boost::uuids::uuid>>& instanceResult,
    const std::optional<std::vector<boost::uuids::uuid>>& dataUuids,
    bool sortByTime)
{
  std::vector<std::set<boost::uuids::uuid>> idsPerSingleModality;

  if (rt_result)
  {
    std::set<boost::uuids::uuid> idsSpatial;
    for (auto it = std::make_move_iterator(rt_result.value().begin()),
              end = std::make_move_iterator(rt_result.value().end());
         it != end; ++it)
    {
      idsSpatial.insert(std::move(it->second));
    }
    idsPerSingleModality.push_back(std::move(idsSpatial));
  }

  if (rt_resultSensorPos)
  {
    std::set<boost::uuids::uuid> idsSpatialSensorPos;
    for (auto it = std::make_move_iterator(rt_resultSensorPos.value().begin()),
              end = std::make_move_iterator(rt_resultSensorPos.value().end());
         it != end; ++it)
    {
      idsSpatialSensorPos.insert(std::move(it->second));
    }
    idsPerSingleModality.push_back(std::move(idsSpatialSensorPos));
  }

  if (timetree_result)
  {
    std::set<boost::uuids::uuid> idsTemporal;
    for (auto it = std::make_move_iterator(timetree_result.value().begin()),
              end = std::make_move_iterator(timetree_result.value().end());
         it != end; ++it)
    {
      idsTemporal.insert(it->second);
    }
    idsPerSingleModality.push_back(std::move(idsTemporal));
  }

  if (semanticResult)
  {
    idsPerSingleModality.push_back(std::move(semanticResult.value()));
  }

  if (instanceResult)
  {
    idsPerSingleModality.push_back(std::move(std::set<boost::uuids::uuid>(
        instanceResult.value().begin(), instanceResult.value().end())));
  }

  if (dataUuids)
  {
    idsPerSingleModality.push_back(std::move(std::set<boost::uuids::uuid>(
        dataUuids.value().begin(), dataUuids.value().end())));
  }

  auto intersectionResult = intersectVectorOfSets(idsPerSingleModality);

  // sort by timestamp
  if (sortByTime)
  {
    if (timetree_result.has_value())
    {
      return sortResultByTime(timetree_result, intersectionResult);
    }
    else
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
          << "Results will not be sorted by time (even though it was "
             "requested), because there is no temporal part in "
             "the query";
    }
  }

  return std::vector(intersectionResult.begin(), intersectionResult.end());
}

std::vector<boost::uuids::uuid> CoreDataset::sortResultByTime(
    std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>& timetree_result,
    std::optional<std::set<boost::uuids::uuid>> intersectionResult)
{
  std::vector<std::pair<int64_t, boost::uuids::uuid>> sortingVector;
  for (auto it = std::make_move_iterator(timetree_result.value().begin()),
            end = std::make_move_iterator(timetree_result.value().end());
       it != end; ++it)
  {
    if (!intersectionResult.has_value() ||
        intersectionResult.value().find(it->second) !=
            intersectionResult.value().end())
    {
      sortingVector.push_back(std::make_pair(
          boost::geometry::get<0>(it->first.min_corner()), it->second));
    }
  }

  std::sort(sortingVector.begin(), sortingVector.end());
  std::vector<boost::uuids::uuid> sortedResults;
  sortedResults.reserve(sortingVector.size());
  for (auto x : sortingVector)
  {
    sortedResults.push_back(x.second);
  }
  return sortedResults;
}

std::set<boost::uuids::uuid> CoreDataset::intersectVectorOfSets(
    std::vector<std::set<boost::uuids::uuid>>& vectorOfSets)
{
  if (vectorOfSets.size() > 1)
  {
    // intersect two sets until the vector has only one set remaining

    std::set<boost::uuids::uuid> intersectionResult;
    // intersect the last and the second-last set
    std::set_intersection(vectorOfSets.at(vectorOfSets.size() - 1).begin(),
                          vectorOfSets.at(vectorOfSets.size() - 1).end(),
                          vectorOfSets.at(vectorOfSets.size() - 2).begin(),
                          vectorOfSets.at(vectorOfSets.size() - 2).end(),
                          std::inserter(intersectionResult,
                                        intersectionResult.begin()));
    // pop the two intersected sets
    vectorOfSets.pop_back();
    vectorOfSets.pop_back();

    // add the intersection result to the vector
    vectorOfSets.push_back(std::move(intersectionResult));
    // recursive call until all sets are intersected
    return intersectVectorOfSets(vectorOfSets);
  }
  else if (vectorOfSets.size() == 1)
  {
    // return the result as soon as all sets are intersected
    return vectorOfSets.at(0);
  }

  // return empty vector if input vector is empty
  return std::set<boost::uuids::uuid>();
}

std::vector<boost::uuids::uuid> CoreDataset::getAllDatasetUuids(
    std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics>
        datatypeSpecifics,
    bool sortByTime)
{
  std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>
      allIdsFromTimeTree = std::vector<seerep_core_msgs::AabbTimeIdPair>();
  seerep_core_msgs::AabbTime aabbtime(
      seerep_core_msgs::TimePoint(
          ((int64_t)std::numeric_limits<uint32_t>::min() << 32 |
           ((uint64_t)std::numeric_limits<uint32_t>::min()))),
      seerep_core_msgs::TimePoint(
          ((int64_t)std::numeric_limits<int32_t>::max()) << 32 |
          ((uint64_t)std::numeric_limits<uint32_t>::max())));

  datatypeSpecifics->timetree.query(
      boost::geometry::index::intersects(aabbtime),
      std::back_inserter(allIdsFromTimeTree.value()));

  if (sortByTime)
  {
    return sortResultByTime(allIdsFromTimeTree);
  }
  else
  {
    std::vector<boost::uuids::uuid> allIds;
    for (auto it = std::make_move_iterator(allIdsFromTimeTree.value().begin()),
              end = std::make_move_iterator(allIdsFromTimeTree.value().end());
         it != end; ++it)
    {
      allIds.push_back(std::move(it->second));
    }
    return allIds;
  }
}

void CoreDataset::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  /// @todo does the default constructor of uuid generate nil uuids?
  if (dataset.header.uuidData.is_nil())
  {
    throw std::invalid_argument("invalid uuid");
  }

  // check if datatype is available
  if (m_datatypeDatatypeSpecificsMap.find(dataset.header.datatype) ==
      m_datatypeDatatypeSpecificsMap.end())
  {
    throw std::invalid_argument("datatype not available in seerep_core");
  }

  addDatasetToIndices(dataset.header.datatype, dataset);
}

/*
 * Data without tf cannot be added to rtree
 * check if tf is now available and add data to spatial rtree
 */
void CoreDataset::tryAddingDataWithMissingTF(
    const seerep_core_msgs::Datatype& datatype)
{
  auto datatypeSpecifics = m_datatypeDatatypeSpecificsMap.at(datatype);

  for (std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>::iterator
           it = datatypeSpecifics->dataWithMissingTF.begin();
       it != datatypeSpecifics->dataWithMissingTF.end();
       /*it++*/ /*<-- increment in loop itself!*/)
  {
    if (isSpatiallyTransformable(**it))
    {
      datatypeSpecifics->rt.insert(
          std::make_pair(transformIndexableAABB(**it), (*it)->header.uuidData));
      datatypeSpecifics->rtSensorPos.insert(std::make_pair(
          getSensorPositionAsAABB(**it), (*it)->header.uuidData));
      it = datatypeSpecifics->dataWithMissingTF.erase(it);
    }
    else
    {
      it++;
    }
  }
}

void CoreDataset::addDatasetToIndices(
    const seerep_core_msgs::Datatype& datatype,
    const seerep_core_msgs::DatasetIndexable& dataset)
{
  auto datatypeSpecifics = m_datatypeDatatypeSpecificsMap.at(datatype);

  if (isSpatiallyTransformable(dataset))
  {
    auto transformedAABB = transformIndexableAABB(dataset);
    std::vector<float> x{ bg::get<bg::min_corner, 0>(transformedAABB),
                          bg::get<bg::max_corner, 0>(transformedAABB) };
    std::vector<float> y{ bg::get<bg::min_corner, 1>(transformedAABB),
                          bg::get<bg::max_corner, 1>(transformedAABB) };
    std::vector<float> z{ bg::get<bg::min_corner, 2>(transformedAABB),
                          bg::get<bg::max_corner, 2>(transformedAABB) };

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "bounding box based on frustum\n"
        << dataset.header.uuidData << " :\n"
        << "xmin: " << bg::get<bg::min_corner, 0>(transformedAABB) << "\n"
        << "xmax: " << bg::get<bg::max_corner, 0>(transformedAABB) << "\n"
        << "ymin: " << bg::get<bg::min_corner, 1>(transformedAABB) << "\n"
        << "ymax: " << bg::get<bg::max_corner, 1>(transformedAABB) << "\n"
        << "zmin: " << bg::get<bg::min_corner, 2>(transformedAABB) << "\n"
        << "zmax: " << bg::get<bg::max_corner, 2>(transformedAABB);

    datatypeSpecifics->rt.insert(
        std::make_pair(transformedAABB, dataset.header.uuidData));
    datatypeSpecifics->rtSensorPos.insert(std::make_pair(
        getSensorPositionAsAABB(dataset), dataset.header.uuidData));
  }
  else
  {
    datatypeSpecifics->dataWithMissingTF.push_back(
        std::make_shared<seerep_core_msgs::DatasetIndexable>(dataset));
  }

  seerep_core_msgs::AabbTime aabbTime(
      seerep_core_msgs::TimePoint(((int64_t)dataset.header.timestamp.seconds)
                                      << 32 |
                                  ((uint64_t)dataset.header.timestamp.nanos)),
      seerep_core_msgs::TimePoint(((int64_t)dataset.header.timestamp.seconds)
                                      << 32 |
                                  ((uint64_t)dataset.header.timestamp.nanos)));
  datatypeSpecifics->timetree.insert(
      std::make_pair(aabbTime, dataset.header.uuidData));

  addLabels(datatype, datatypeSpecifics, dataset.labelsCategory,
            dataset.header.uuidData);
}

void CoreDataset::addLabels(
    const seerep_core_msgs::Datatype& datatype,
    const std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
        labelPerCategory,
    const boost::uuids::uuid& msgUuid)
{
  addLabels(datatype, m_datatypeDatatypeSpecificsMap.at(datatype),
            labelPerCategory, msgUuid);
}

void CoreDataset::addLabels(
    const seerep_core_msgs::Datatype& datatype,
    std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics>
        datatypeSpecifics,
    const std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
        labelPerCategory,
    const boost::uuids::uuid& msgUuid)
{
  std::vector<boost::uuids::uuid> instanceUuids;
  // loop categories
  for (auto labelOfCategory : labelPerCategory)
  {
    // check if label already exists
    auto categoryMapEntry =
        datatypeSpecifics->categoryLabelDatasetsMap.find(labelOfCategory.first);
    if (categoryMapEntry == datatypeSpecifics->categoryLabelDatasetsMap.end())
    {
      auto emplaceReturn = datatypeSpecifics->categoryLabelDatasetsMap.emplace(
          labelOfCategory.first,
          std::unordered_map<std::string, std::vector<boost::uuids::uuid>>());
      if (!emplaceReturn.second)
      {
        throw std::runtime_error(
            "could not insert label category to datatype specifics.");
      }
      categoryMapEntry = emplaceReturn.first;
    }
    // loop labels
    for (auto labelWithInstance : labelOfCategory.second.labels)
    {
      // check if label already exists
      auto labelMapEntry =
          categoryMapEntry->second.find(labelWithInstance.label);
      if (labelMapEntry != categoryMapEntry->second.end())
      {
        // label already exists, add id of dataset to the vector
        labelMapEntry->second.push_back(msgUuid);
      }
      else
      {
        // label doesn't already exist. Create new pair of label and vector of dataset ids
        categoryMapEntry->second.emplace(
            labelWithInstance.label, std::vector<boost::uuids::uuid>{ msgUuid });
      }

      // add to instance
      if (!labelWithInstance.uuidInstance.is_nil())
      {
        m_coreInstances->addDataset(labelWithInstance, msgUuid, datatype);

        // collect the instance uuids of this dataset in this vector
        instanceUuids.push_back(labelWithInstance.uuidInstance);
      }
    }
  }
  // add the vector of instance uuids to the datatypespecifics
  datatypeSpecifics->datasetInstancesMap.emplace(msgUuid, instanceUuids);
}

bool CoreDataset::verifyPolygonIntegrity(
    CGAL::Polygon_2<ExactKernel>& polygon_cgal)
{
  if (!polygon_cgal.is_simple())
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << "Two or more points in the polygon are the same.";
    return false;
  }
  if (!polygon_cgal.is_convex())
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << "Polygon is not convex.";
    return false;
  }

  // cgal polygon must be counter clockwise oriented
  if (polygon_cgal.is_clockwise_oriented())
  {
    polygon_cgal.reverse_orientation();
  }

  return true;
}

CGAL::Polygon_2<ExactKernel>
CoreDataset::toCGALPolygon(const seerep_core_msgs::Polygon2D& polygon)
{
  CGAL::Polygon_2<ExactKernel> polygon_cgal;
  for (auto point : polygon.vertices)
  {
    polygon_cgal.push_back(
        ExactKernel::Point_2(bg::get<0>(point), bg::get<1>(point)));
  }
  return polygon_cgal;
}

CGAL::Polygon_2<ExactKernel>
CoreDataset::toCGALPolygon(const seerep_core_msgs::AABB& aabb)
{
  /* Check if the bounding box has no spatial extent -> Only add one point to the polygon */
  if (bg::get<bg::min_corner, 0>(aabb) == bg::get<bg::max_corner, 0>(aabb) &&
      bg::get<bg::min_corner, 1>(aabb) == bg::get<bg::max_corner, 1>(aabb))
  {
    ExactKernel::Point_2 points_aabb[] = { ExactKernel::Point_2(
        bg::get<bg::min_corner, 0>(aabb), bg::get<bg::min_corner, 1>(aabb)) };
    CGAL::Polygon_2<ExactKernel> aabb_cgal(points_aabb, points_aabb + 1);
    return aabb_cgal;
  }
  else
  {
    ExactKernel::Point_2 points_aabb[] = {
      ExactKernel::Point_2(bg::get<bg::min_corner, 0>(aabb),
                           bg::get<bg::min_corner, 1>(aabb)),
      ExactKernel::Point_2(bg::get<bg::max_corner, 0>(aabb),
                           bg::get<bg::min_corner, 1>(aabb)),
      ExactKernel::Point_2(bg::get<bg::max_corner, 0>(aabb),
                           bg::get<bg::max_corner, 1>(aabb)),
      ExactKernel::Point_2(bg::get<bg::min_corner, 0>(aabb),
                           bg::get<bg::max_corner, 1>(aabb)),
    };
    CGAL::Polygon_2<ExactKernel> aabb_cgal(points_aabb, points_aabb + 4);

    return aabb_cgal;
  }
}

void CoreDataset::intersectionDegree(const seerep_core_msgs::AABB& aabb,
                                     const seerep_core_msgs::Polygon2D& polygon,
                                     bool& fullEncapsulation,
                                     bool& partialEncapsulation)
{
  // convert seerep core aabb to cgal polygon
  CGAL::Polygon_2<ExactKernel> aabb_cgal = toCGALPolygon(aabb);

  // convert seerep core polygon to cgal polygon
  CGAL::Polygon_2<ExactKernel> polygon_cgal = toCGALPolygon(polygon);

  // a cgal polyon needs to be simple, convex and the points should be added
  // in a counter clockwise order
  if (!verifyPolygonIntegrity(polygon_cgal) ||
      !verifyPolygonIntegrity(aabb_cgal))
  {
    return;
  }

  intersectionDegreeAABBinPolygon(aabb, polygon, aabb_cgal, polygon_cgal,
                                  fullEncapsulation, partialEncapsulation);
}

void CoreDataset::intersectionDegreeCgalPolygons(
    CGAL::Polygon_2<ExactKernel> cgal1, CGAL::Polygon_2<ExactKernel> cgal2,
    bool z_partially, bool checkIfFullyEncapsulated, bool& fullEncapsulation,
    bool& partialEncapsulation)
{
  for (CGAL::Polygon_2<ExactKernel>::Vertex_iterator vi = cgal1.vertices_begin();
       vi != cgal1.vertices_end(); ++vi)
  {
    // is the vertex of the aabb inside or on the boundary (not outside) the
    // polygon on the x and y axis
    bool xy_bounded = !(cgal2.bounded_side(*vi) == CGAL::ON_UNBOUNDED_SIDE);

    if (!xy_bounded && checkIfFullyEncapsulated)
    {
      fullEncapsulation = false;
    }

    if (xy_bounded && z_partially)
    {
      partialEncapsulation = true;
    }
  }
}

void CoreDataset::intersectionDegreeAABBinPolygon(
    const seerep_core_msgs::AABB& aabb,
    const seerep_core_msgs::Polygon2D& polygon,
    CGAL::Polygon_2<ExactKernel> aabb_cgal,
    CGAL::Polygon_2<ExactKernel> polygon_cgal, bool& fullEncapsulation,
    bool& partialEncapsulation)
{
  fullEncapsulation = true;
  partialEncapsulation = false;

  // are the corners of the aabb inside the polygon on the z axis
  bool z_aabb_fully_in_polygon =
      bg::get<bg::min_corner, 2>(aabb) >= polygon.z &&
      bg::get<bg::min_corner, 2>(aabb) <= (polygon.z + polygon.height) &&
      bg::get<bg::max_corner, 2>(aabb) >= polygon.z &&
      bg::get<bg::max_corner, 2>(aabb) <= (polygon.z + polygon.height);
  bool z_aabb_partially_in_polygon =
      (bg::get<bg::min_corner, 2>(aabb) >= polygon.z &&
       bg::get<bg::min_corner, 2>(aabb) <= (polygon.z + polygon.height)) ||
      (bg::get<bg::max_corner, 2>(aabb) >= polygon.z &&
       bg::get<bg::max_corner, 2>(aabb) <= (polygon.z + polygon.height));
  // are the corners of the polygon inside the aabb on the z axis
  /*bool z_polygon_fully_in_aabb =
      polygon.z >= bg::get<bg::min_corner, 2>(aabb) &&
      polygon.z <= bg::get<bg::max_corner, 2>(aabb) &&
      (polygon.z + polygon.height) >= bg::get<bg::min_corner, 2>(aabb) &&
      (polygon.z + polygon.height) <= bg::get<bg::max_corner, 2>(aabb);*/
  bool z_polygon_partially_in_aabb =
      (polygon.z >= bg::get<bg::min_corner, 2>(aabb) &&
       polygon.z <= bg::get<bg::max_corner, 2>(aabb)) ||
      ((polygon.z + polygon.height) >= bg::get<bg::min_corner, 2>(aabb) &&
       (polygon.z + polygon.height) <= bg::get<bg::max_corner, 2>(aabb));

  bool z_fully = z_aabb_fully_in_polygon;
  bool z_partially = z_aabb_partially_in_polygon || z_polygon_partially_in_aabb;

  // if not fully in z direction then for sure not fully
  if (!z_fully)
  {
    fullEncapsulation = false;
  }

  // if not fully nor partially skip following checks
  if (!z_fully && !z_partially)
  {
    return;
  }

  // traverse the axis aligned bounding box and check if AABB is in polygon
  intersectionDegreeCgalPolygons(aabb_cgal, polygon_cgal, z_partially, true,
                                 fullEncapsulation, partialEncapsulation);

  // traverse polygon and check if at least one vertex is in the AABB
  // we want to handle this as partially encapsulated because inner parts of the
  // AABB are inside the polygon
  intersectionDegreeCgalPolygons(polygon_cgal, aabb_cgal, z_partially, false,
                                 fullEncapsulation, partialEncapsulation);
}

CGSurfaceMesh
CoreDataset::toSurfaceMesh(const seerep_core_msgs::Polygon2D& seerep_polygon)
{
  using CGVertexIndex = CGSurfaceMesh::Vertex_index;

  if (seerep_polygon.vertices.size() <= 2)
  {
    throw std::invalid_argument(
        "seerep_core_msgs::Polygon2D has not enough vertices to be expanded"
        " to a full 3D SurfaceMesh");
  }

  CGSurfaceMesh surface_mesh;

  std::vector<CGVertexIndex> lower_surface, upper_surface;
  CGVertexIndex topInitial, bottomInitial;
  CGVertexIndex topFirst, bottomFirst, topSecond, bottomSecond;

  CGFaceIdx face_desc;

  auto z_top = seerep_polygon.z + seerep_polygon.height;
  auto& z_bottom = seerep_polygon.z;
  auto& verts = seerep_polygon.vertices;

  auto x = verts[0].get<0>();
  auto y = verts[0].get<1>();
  topInitial = topFirst = surface_mesh.add_vertex(CGPoint_3{ x, y, z_top });
  bottomInitial = bottomFirst =
      surface_mesh.add_vertex(CGPoint_3{ x, y, z_bottom });

  // the vertices are connected in order; The assumption is that the last point
  // of the polygon connects to the first
  for (size_t j = 1; j < verts.size(); ++j)
  {
    x = verts[j].get<0>();
    y = verts[j].get<1>();
    topSecond = surface_mesh.add_vertex(CGPoint_3{ x, y, z_top });
    bottomSecond = surface_mesh.add_vertex(CGPoint_3{ x, y, z_bottom });

    upper_surface.push_back(topFirst);
    lower_surface.push_back(bottomFirst);
    face_desc =
        surface_mesh.add_face(topFirst, topSecond, bottomSecond, bottomFirst);

    if (face_desc == CGSurfaceMesh::null_face())
    {
      throw std::invalid_argument(
          "Could not add side face from Polygon2d to SurfaceMesh correctly!");
    }

    topFirst = topSecond;
    bottomFirst = bottomSecond;
  }
  upper_surface.push_back(topFirst);
  lower_surface.push_back(bottomFirst);

  std::vector<CGSurfaceMesh::Face_index> descriptors;

  descriptors.push_back(
      surface_mesh.add_face(topFirst, topInitial, bottomInitial, bottomFirst));
  descriptors.push_back(surface_mesh.add_face(lower_surface));

  // for whatever reason the vertices need to be reversed for the latest face to
  // construct the mesh correctly.
  // If the construction of the latest face does not work ALWAYS try
  // reversing the vertex order
  std::reverse(upper_surface.begin(), upper_surface.end());
  descriptors.push_back(surface_mesh.add_face(upper_surface));

  // check if any of the faces was not constructed properly
  if (std::find_if(descriptors.begin(), descriptors.end(), [](auto elem) {
        return elem == CGSurfaceMesh::null_face();
      }) != descriptors.end())
  {
    throw std::invalid_argument("Could not upper/lower or latest side face from"
                                " Polygon2d to SurfaceMesh correctly!");
  }

  return surface_mesh;
}

void CoreDataset::checkIntersectionWithZExtrudedPolygon(
    CGSurfaceMesh enclosedMesh,
    const seerep_core_msgs::Polygon2D& enclosingPolygon,
    bool& fullEncapsulation, bool& partialEncapsulation)
{
  using CGAL::Polygon_mesh_processing::triangulate_faces;

  if (!enclosedMesh.is_valid() || enclosedMesh.is_empty())
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "enclosedMesh passed to checkIntersectionWithZExtrudedPolygon "
           "is either not valid or empty";
    return;
  }

  fullEncapsulation = false;
  partialEncapsulation = false;

  // check if the enclosing polygon encloses every point
  // get highest and lowest point of innerPoints
  auto highestPoint = std::max_element(
      enclosedMesh.vertices_begin(), enclosedMesh.vertices_end(),
      [&enclosedMesh](const CGSurfaceMesh::Vertex_index& p1,
                      const CGSurfaceMesh::Vertex_index& p2) {
        return enclosedMesh.point(p1).z() < enclosedMesh.point(p2).z();
      });

  auto lowestPoint = std::min_element(
      enclosedMesh.vertices_begin(), enclosedMesh.vertices_end(),
      [&enclosedMesh](const CGSurfaceMesh::Vertex_index& p1,
                      const CGSurfaceMesh::Vertex_index& p2) {
        return enclosedMesh.point(p1).z() > enclosedMesh.point(p2).z();
      });

  const double& z_low = enclosingPolygon.z;
  double z_high = z_low + enclosingPolygon.height;

  double p_low =
      enclosedMesh.point(*lowestPoint).z().exact().convert_to<double>();
  double p_high =
      enclosedMesh.point(*highestPoint).z().exact().convert_to<double>();

  // fully enclosed if all points of the enclosedMesh are contained in the enclosing Polygon
  if ((z_low <= p_low && z_low <= p_high && z_high >= p_low && z_high >= p_high))
  {
    auto enclosedPolygon2CG = reduceZDimension(enclosedMesh);
    auto enclosingPolygon2CG = toCGALPolygon(enclosingPolygon);

    bool xy_bounded = true;
    for (CGPolygon_2::Vertex_iterator vi = enclosedPolygon2CG.vertices_begin();
         vi != enclosedPolygon2CG.vertices_end(); ++vi)
    {
      if (enclosingPolygon2CG.bounded_side(*vi) == CGAL::ON_UNBOUNDED_SIDE)
      {
        xy_bounded = false;
        break;
      }
    }
    if (xy_bounded)
    {
      fullEncapsulation = true;
      partialEncapsulation = true;
      return;
    }
  }

  auto enclosingMesh = toSurfaceMesh(enclosingPolygon);

  if (!enclosingMesh.is_valid() || enclosingMesh.is_empty())
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "enclosingMesh derived from enclosingPolygon"
           "passed to checkIntersectionWithZExtrudedPolygon "
           "is either not valid or empty";
    return;
  }

  triangulate_faces(enclosedMesh);
  triangulate_faces(enclosingMesh);

  partialEncapsulation =
      CGAL::Polygon_mesh_processing::do_intersect(enclosedMesh, enclosingMesh);
}

CGPolygon_2 CoreDataset::reduceZDimension(const CGSurfaceMesh& mesh)
{
  CGPolygon_2 poly2;

  for (CGSurfaceMesh::Vertex_index idx : mesh.vertices())
  {
    CGPoint_2 p{ mesh.point(idx).x(), mesh.point(idx).y() };
    poly2.push_back(p);
  }
  return poly2;
}

seerep_core_msgs::AabbTime
CoreDataset::getTimeBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  seerep_core_msgs::AabbTime overalltime;

  overalltime.max_corner().set<0>(std::numeric_limits<int64_t>::min());
  overalltime.min_corner().set<0>(std::numeric_limits<int64_t>::max());

  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    seerep_core_msgs::AabbTime timeinterval =
        m_datatypeDatatypeSpecificsMap.at(dt)->timetree.bounds();

    // compare min and update if need be
    if (timeinterval.min_corner().get<0>() < overalltime.min_corner().get<0>())
    {
      overalltime.min_corner().set<0>(timeinterval.min_corner().get<0>());
    }

    // compare min and update if need be
    if (timeinterval.max_corner().get<0>() > overalltime.max_corner().get<0>())
    {
      overalltime.max_corner().set<0>(timeinterval.max_corner().get<0>());
    }
  }

  return overalltime;
}

seerep_core_msgs::AABB
CoreDataset::getSpatialBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  seerep_core_msgs::AABB overallbb;

  // set the minimum to minimum possible for the datatype
  overallbb.max_corner().set<0>(std::numeric_limits<float>::lowest());
  overallbb.max_corner().set<1>(std::numeric_limits<float>::lowest());
  overallbb.max_corner().set<2>(std::numeric_limits<float>::lowest());

  // set the maximum for the maximum possible for the datatype
  overallbb.min_corner().set<0>(std::numeric_limits<float>::max());
  overallbb.min_corner().set<1>(std::numeric_limits<float>::max());
  overallbb.min_corner().set<2>(std::numeric_limits<float>::max());

  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    // check if the data has now some tf for adding it to spatial rtree
    tryAddingDataWithMissingTF(dt);

    seerep_core_msgs::AABB rtree_bounds =
        m_datatypeDatatypeSpecificsMap.at(dt)->rt.bounds();

    seerep_core_msgs::AABB rtreeSensorPos_bounds =
        m_datatypeDatatypeSpecificsMap.at(dt)->rtSensorPos.bounds();

    // update the min if needed for dimension 0
    auto min0 = std::min(rtree_bounds.min_corner().get<0>(),
                         rtreeSensorPos_bounds.min_corner().get<0>());
    if (min0 < overallbb.min_corner().get<0>())
    {
      overallbb.min_corner().set<0>(min0);
    }

    // update the min if needed for dimension 1
    auto min1 = std::min(rtree_bounds.min_corner().get<1>(),
                         rtreeSensorPos_bounds.min_corner().get<1>());
    if (min1 < overallbb.min_corner().get<1>())
    {
      overallbb.min_corner().set<1>(min1);
    }

    // update the min if needed for dimension 2
    auto min2 = std::min(rtree_bounds.min_corner().get<2>(),
                         rtreeSensorPos_bounds.min_corner().get<2>());
    if (min2 < overallbb.min_corner().get<2>())
    {
      overallbb.min_corner().set<2>(min2);
    }

    // update the max if needed for dimension 0
    auto max0 = std::max(rtree_bounds.max_corner().get<0>(),
                         rtreeSensorPos_bounds.max_corner().get<0>());
    if (max0 > overallbb.max_corner().get<0>())
    {
      overallbb.max_corner().set<0>(max0);
    }

    // update the max if needed for dimension 1
    auto max1 = std::max(rtree_bounds.max_corner().get<1>(),
                         rtreeSensorPos_bounds.max_corner().get<1>());
    if (max1 > overallbb.max_corner().get<1>())
    {
      overallbb.max_corner().set<1>(max1);
    }

    // update the max if needed for dimension 2
    auto max2 = std::max(rtree_bounds.max_corner().get<2>(),
                         rtreeSensorPos_bounds.max_corner().get<2>());
    if (max2 > overallbb.max_corner().get<2>())
    {
      overallbb.max_corner().set<2>(max2);
    }
  }
  return overallbb;
}

std::unordered_set<std::string>
CoreDataset::getAllCategories(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  std::unordered_set<std::string> categories;

  // traverse all datatypes
  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    // obtain the map which holds the categories
    auto categories_map =
        &(m_datatypeDatatypeSpecificsMap.at(dt)->categoryLabelDatasetsMap);

    // traverse this map
    for (auto& [category, value] : *categories_map)
    {
      categories.insert(category);
    }
  }
  // return the prepared vector
  return categories;
}

std::unordered_set<std::string>
CoreDataset::getAllLabels(std::vector<seerep_core_msgs::Datatype> datatypes,
                          std::string category)
{
  std::unordered_set<std::string> labels;

  // traverse all datatypes
  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    // obtain the map pertaining to the provided category
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>*
        label_to_uuid_map = &(m_datatypeDatatypeSpecificsMap.at(dt)
                                  ->categoryLabelDatasetsMap[category]);

    // traverse this map
    for (auto& [label, value] : *label_to_uuid_map)
    {
      labels.insert(label);
    }
  }
  // return the prepared vector
  return labels;
}

bool CoreDataset::isSpatiallyTransformable(
    const seerep_core_msgs::DatasetIndexable& dataset)
{
  return m_tfOverview->canTransform(dataset.header.frameId, m_frameId,
                                    dataset.header.timestamp.seconds,
                                    dataset.header.timestamp.nanos);
}

seerep_core_msgs::AABB CoreDataset::transformIndexableAABB(
    const seerep_core_msgs::DatasetIndexable& indexable)
{
  return m_tfOverview->transformAABB(indexable.boundingbox,
                                     indexable.header.frameId, m_frameId,
                                     indexable.header.timestamp.seconds,
                                     indexable.header.timestamp.nanos);
}

seerep_core_msgs::AABB CoreDataset::getSensorPositionAsAABB(
    const seerep_core_msgs::DatasetIndexable& indexable)
{
  auto tf = m_tfOverview->getData(indexable.header.timestamp.seconds,
                                  indexable.header.timestamp.nanos, m_frameId,
                                  indexable.header.frameId);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "sensor position\n"
      << indexable.header.uuidData << " :\n"
      << tf.value().transform.translation;

  seerep_core_msgs::AABB sensorPosAABB;
  bg::set<bg::min_corner, 0>(sensorPosAABB, tf.value().transform.translation.x);
  bg::set<bg::min_corner, 1>(sensorPosAABB, tf.value().transform.translation.y);
  bg::set<bg::min_corner, 2>(sensorPosAABB, tf.value().transform.translation.z);

  bg::set<bg::max_corner, 0>(sensorPosAABB, tf.value().transform.translation.x);
  bg::set<bg::max_corner, 1>(sensorPosAABB, tf.value().transform.translation.y);
  bg::set<bg::max_corner, 2>(sensorPosAABB, tf.value().transform.translation.z);

  return sensorPosAABB;
}

} /* namespace seerep_core */
