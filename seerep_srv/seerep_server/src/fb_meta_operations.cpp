#include "seerep_server/fb_meta_operations.h"

namespace seerep_server
{
FbMetaOperations::FbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status FbMetaOperations::CreateProject(grpc::ServerContext* context,
                                             const flatbuffers::grpc::Message<seerep::fb::ProjectCreation>* request,
                                             flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "create new project... ";
  const seerep::fb::ProjectCreation* requestMsg = request->GetRoot();
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = requestMsg->map_frame_id()->str();
  projectInfo.name = requestMsg->name()->str();
  projectInfo.uuid = boost::uuids::random_generator()();

  // extracting geodetic coordinates attribute information from flatbuffer and saving in seerep core msg struct
  projectInfo.geodetCoords.coordinateSystem = requestMsg->geodetic_position()->coordinateSystem()->str();
  projectInfo.geodetCoords.ellipsoid = requestMsg->geodetic_position()->ellipsoid()->str();
  projectInfo.geodetCoords.longitude = requestMsg->geodetic_position()->longitude();
  projectInfo.geodetCoords.latitude = requestMsg->geodetic_position()->latitude();
  projectInfo.geodetCoords.altitude = requestMsg->geodetic_position()->altitude();

  seerepCore->createProject(projectInfo);

  flatbuffers::grpc::MessageBuilder builder;
  auto nameOffset = builder.CreateString(projectInfo.name);
  auto uuidOffset = builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid));
  auto frameIdOffset = builder.CreateString(projectInfo.frameId);
  auto responseOffset = seerep::fb::CreateProjectInfo(builder, nameOffset, uuidOffset, frameIdOffset);

  builder.Finish(responseOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfo>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::GetProjects(grpc::ServerContext* context,
                                           const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                           flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "query the project infos... ";
  auto projectInfos = seerepCore->getProjects();

  flatbuffers::grpc::MessageBuilder builder;
  std::vector<flatbuffers::Offset<seerep::fb::ProjectInfo>> projectInfosVector;
  for (auto projectInfo : projectInfos)
  {
    auto nameOffset = builder.CreateString(projectInfo.name);
    auto uuidOffset = builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid));
    auto frameIdOffset = builder.CreateString(projectInfo.frameId);

    auto coordinateSystemOffset = builder.CreateString(projectInfo.geodetCoords.coordinateSystem);
    auto ellipsoidOffset = builder.CreateString(projectInfo.geodetCoords.ellipsoid);

    seerep::fb::GeodeticCoordinatesBuilder gcbuilder(builder);
    gcbuilder.add_coordinateSystem(coordinateSystemOffset);
    gcbuilder.add_ellipsoid(ellipsoidOffset);
    gcbuilder.add_altitude(projectInfo.geodetCoords.altitude);
    gcbuilder.add_latitude(projectInfo.geodetCoords.latitude);
    gcbuilder.add_longitude(projectInfo.geodetCoords.longitude);
    auto geodeticCoordinatesOffset = gcbuilder.Finish();

    projectInfosVector.push_back(
        seerep::fb::CreateProjectInfo(builder, nameOffset, uuidOffset, frameIdOffset, geodeticCoordinatesOffset));
  }
  auto vectorOffset = builder.CreateVector(projectInfosVector);
  auto projectInfosOffset = seerep::fb::CreateProjectInfos(builder, vectorOffset);
  builder.Finish(projectInfosOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfos>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::LoadProjects(grpc::ServerContext* context,
                                            const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                            flatbuffers::grpc::Message<seerep::fb::Empty>* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading all projects in the working directory... ";

  try
  {
    seerepCore->loadProjectsInFolder();
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  flatbuffers::grpc::MessageBuilder builder;
  seerep::fb::CreateEmpty(builder);
  *response = builder.ReleaseMessage<seerep::fb::Empty>();

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::DeleteProject(grpc::ServerContext* context,
                                             const flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* request,
                                             flatbuffers::grpc::Message<seerep::fb::Empty>* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  std::string uuid = request->GetRoot()->uuid()->str();
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "deleting project... with uuid: " << uuid;

  try
  {
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);
    seerepCore->deleteProject(uuidFromString);
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }
  flatbuffers::grpc::MessageBuilder builder;
  seerep::fb::CreateEmpty(builder);
  *response = builder.ReleaseMessage<seerep::fb::Empty>();

  return grpc::Status::OK;
}

grpc::Status
FbMetaOperations::GetOverallTimeInterval(grpc::ServerContext* context,
                                         const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
                                         flatbuffers::grpc::Message<seerep::fb::TimeInterval>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  seerep::fb::Datatype casted_datatype = static_cast<seerep::fb::Datatype>(requestRoot->datatype());
  dt_vector = convertFbDatatypeVector(casted_datatype);

  try
  {
    std::string uuid = request->GetRoot()->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AabbTime timeinterval = seerepCore->getOverallTimeInterval(uuidFromString, dt_vector);

    // isolate second and nano second bits from min time
    uint64_t mintime = timeinterval.min_corner().get<0>();
    uint32_t min_nanos = (uint32_t)mintime;
    uint32_t min_seconds = (int32_t)(mintime >> 32);

    // isolate second and nano second bits from max time
    uint64_t maxtime = timeinterval.max_corner().get<0>();
    uint32_t max_nanos = (uint32_t)maxtime;
    uint32_t max_seconds = (int32_t)(maxtime >> 32);

    flatbuffers::grpc::MessageBuilder builder;

    seerep::fb::TimestampBuilder minTimeStampBuilder(builder);
    minTimeStampBuilder.add_seconds(min_seconds);
    minTimeStampBuilder.add_nanos(min_nanos);
    flatbuffers::Offset<seerep::fb::Timestamp> min = minTimeStampBuilder.Finish();

    seerep::fb::TimestampBuilder maxTimeStampBuilder(builder);
    maxTimeStampBuilder.add_seconds(max_seconds);
    maxTimeStampBuilder.add_nanos(max_nanos);
    flatbuffers::Offset<seerep::fb::Timestamp> max = maxTimeStampBuilder.Finish();

    seerep::fb::TimeIntervalBuilder timeIntervalBuilder(builder);
    timeIntervalBuilder.add_time_min(min);
    timeIntervalBuilder.add_time_max(max);
    flatbuffers::Offset<seerep::fb::TimeInterval> bb = timeIntervalBuilder.Finish();

    builder.Finish(bb);
    *response = builder.ReleaseMessage<seerep::fb::TimeInterval>();
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}

grpc::Status
FbMetaOperations::GetOverallBoundingBox(grpc::ServerContext* context,
                                        const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
                                        flatbuffers::grpc::Message<seerep::fb::Boundingbox>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  // flatbuffers::IsFieldPresent(requestRoot, seerep::fb::UuidDatatypePair::datatype);
  // requestRoot->CheckField(seerep::fb::UuidDatatypePair::datatype);

  seerep::fb::Datatype casted_datatype = static_cast<seerep::fb::Datatype>(requestRoot->datatype());
  dt_vector = convertFbDatatypeVector(casted_datatype);

  try
  {
    std::string uuid = requestRoot->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AABB overallBB = seerepCore->getOverallBound(uuidFromString, dt_vector);

    // center
    int center_x = (overallBB.min_corner().get<0>() + overallBB.max_corner().get<0>()) / 2;
    int center_y = (overallBB.min_corner().get<1>() + overallBB.max_corner().get<1>()) / 2;
    int center_z = (overallBB.min_corner().get<2>() + overallBB.max_corner().get<2>()) / 2;

    // spatial extent
    int se_x = (overallBB.max_corner().get<0>() - overallBB.min_corner().get<0>());
    int se_y = (overallBB.max_corner().get<1>() - overallBB.min_corner().get<1>());
    int se_z = (overallBB.max_corner().get<2>() - overallBB.min_corner().get<2>());

    flatbuffers::grpc::MessageBuilder builder;

    seerep::fb::PointBuilder centerPointBuilder(builder);
    centerPointBuilder.add_x(center_x);
    centerPointBuilder.add_y(center_y);
    centerPointBuilder.add_z(center_z);
    flatbuffers::Offset<seerep::fb::Point> centerPoint = centerPointBuilder.Finish();

    seerep::fb::PointBuilder spatialExtentBuilder(builder);
    spatialExtentBuilder.add_x(se_x);
    spatialExtentBuilder.add_y(se_y);
    spatialExtentBuilder.add_z(se_z);
    flatbuffers::Offset<seerep::fb::Point> spatialExtent = spatialExtentBuilder.Finish();

    seerep::fb::BoundingboxBuilder boundingBoxBuilder(builder);
    boundingBoxBuilder.add_center_point(centerPoint);
    boundingBoxBuilder.add_spatial_extent(spatialExtent);
    flatbuffers::Offset<seerep::fb::Boundingbox> bb = boundingBoxBuilder.Finish();

    builder.Finish(bb);
    *response = builder.ReleaseMessage<seerep::fb::Boundingbox>();
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::GetAllCategories(grpc::ServerContext* context,
                                                const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
                                                flatbuffers::grpc::Message<seerep::fb::Categories>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  seerep::fb::Datatype casted_datatype = static_cast<seerep::fb::Datatype>(requestRoot->datatype());
  dt_vector = convertFbDatatypeVector(casted_datatype);

  try
  {
    std::string uuid = requestRoot->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    std::vector<std::string> categories = seerepCore->getAllCategories(uuidFromString, dt_vector);

    flatbuffers::grpc::MessageBuilder builder;

    std::vector<flatbuffers::Offset<flatbuffers::String>> fb_categories;

    for (std::string cat : categories)
    {
      flatbuffers::Offset<flatbuffers::String> fb_category = builder.CreateString(cat);
      fb_categories.push_back(fb_category);
    }

    auto fb_categories_vector = builder.CreateVector(fb_categories);

    seerep::fb::CategoriesBuilder cb(builder);
    cb.add_categories(fb_categories_vector);
    builder.Finish(cb.Finish());

    *response = builder.ReleaseMessage<seerep::fb::Categories>();
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}

grpc::Status
FbMetaOperations::GetAllLabels(grpc::ServerContext* context,
                               const flatbuffers::grpc::Message<seerep::fb::UuidDatatypeWithCategory>* request,
                               flatbuffers::grpc::Message<seerep::fb::Labels>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  seerep::fb::Datatype casted_datatype = static_cast<seerep::fb::Datatype>(requestRoot->UuidAndDatatype()->datatype());
  dt_vector = convertFbDatatypeVector(casted_datatype);

  try
  {
    std::string category = requestRoot->category()->str();

    std::string uuid = requestRoot->UuidAndDatatype()->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    std::vector<std::string> allLabels = seerepCore->getAllLabels(uuidFromString, dt_vector, category);

    flatbuffers::grpc::MessageBuilder builder;

    std::vector<flatbuffers::Offset<flatbuffers::String>> fb_labels;

    for (std::string lbl : allLabels)
    {
      flatbuffers::Offset<flatbuffers::String> fb_label = builder.CreateString(lbl);
      fb_labels.push_back(fb_label);
    }

    auto fb_labels_vector = builder.CreateVector(fb_labels);

    seerep::fb::LabelsBuilder lb(builder);
    lb.add_labels(fb_labels_vector);
    builder.Finish(lb.Finish());

    *response = builder.ReleaseMessage<seerep::fb::Labels>();
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }
  return grpc::Status::OK;
}

std::vector<seerep_core_msgs::Datatype> FbMetaOperations::convertFbDatatypeVector(const seerep::fb::Datatype dt)
{
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  if (dt == seerep::fb::Datatype_Image)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
  }
  else if (dt == seerep::fb::Datatype_PointCloud)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
  }
  else if (dt == seerep::fb::Datatype_Point)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else if (dt == seerep::fb::Datatype_All)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Unknown);
  }

  return dt_vector;
}

} /* namespace seerep_server */
