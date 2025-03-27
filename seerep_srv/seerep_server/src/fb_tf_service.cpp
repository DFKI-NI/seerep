#include "seerep_server/fb_tf_service.h"

namespace seerep_server
{
FbTfService::FbTfService(std::shared_ptr<seerep_core::Core> seerepCore)
  : tfFb(std::make_shared<seerep_core_fb::CoreFbTf>(seerepCore))
{
}

grpc::Status FbTfService::TransferTransformStamped(
    grpc::ServerContext* context,
    grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::TransformStamped>>*
        reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "received transform... ";
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::TransformStamped> tfMsg;
  while (reader->Read(&tfMsg))
  {
    auto transform = tfMsg.GetRoot();
    std::string uuidProject = transform->header()->uuid_project()->str();
    if (!uuidProject.empty())
    {
      try
      {
        tfFb->addData(*transform);
      }
      catch (std::runtime_error const& e)
      {
        // mainly catching "invalid uuid string" when transforming uuid_project
        // from string to uuid also catching core doesn't have project with uuid
        // error
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << e.what();

        seerep_server_util::createResponseFb(
            std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
            response);

        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
      catch (const std::exception& e)
      {
        // specific handling for all exceptions extending std::exception, except
        // std::runtime_error which is handled explicitly
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << e.what();
        seerep_server_util::createResponseFb(
            std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
            response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
      catch (...)
      {
        // catch any other errors (that we have no information about)
        std::string msg =
            "Unknown failure occurred. Possible memory corruption";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << msg;
        seerep_server_util::createResponseFb(
            msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
      }
    }
    else
    {
      answer = "a msg had no project uuid!";
    }
  }

  seerep_server_util::createResponseFb(
      answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

grpc::Status FbTfService::DeleteTransformStamped(
    grpc::ServerContext* context,
    grpc::ServerReader<
        flatbuffers::grpc::Message<seerep::fb::TransformStampedIntervalQuery>>*
        reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "received transform deletion request... ";

  std::string answer = "selected tfs deleted!";

  flatbuffers::grpc::Message<seerep::fb::TransformStampedIntervalQuery>
      tfIntervalQueryMsg;
  std::unordered_set<boost::uuids::uuid, boost::hash<boost::uuids::uuid>>
      projectUuids;
  while (reader->Read(&tfIntervalQueryMsg))
  {
    auto tfIntervalQuery = tfIntervalQueryMsg.GetRoot();
    auto projectUuidStr = tfIntervalQuery->transform_stamped_query()
                              ->header()
                              ->uuid_project()
                              ->str();
    if (!projectUuidStr.empty())
    {
      try
      {
        if (auto deletedInProjectUuid = tfFb->deleteHdf5(*tfIntervalQuery))
        {
          projectUuids.insert(deletedInProjectUuid.value());
        }
      }
      catch (std::runtime_error const& e)
      {
        // mainly catching "invalid uuid string" when transforming uuid_project from
        // string to uuid also catching core doesn't have project with uuid error
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << e.what();
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
      catch (const std::exception& e)
      {
        // specific handling for all exceptions extending std::exception, except
        // std::runtime_error which is handled explicitly
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << e.what();
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
      catch (...)
      {
        // catch any other errors (that we have no information about)
        std::string msg =
            "Unknown failure occurred. Possible memory corruption";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << msg;
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
      }
    }
    else
    {
      answer = "one msg had no project uuid!";
    }
  }
  for (auto projectUuid : projectUuids)
  {
    try
    {
      tfFb->reinitializeTFs(projectUuid);
    }
    catch (std::runtime_error const& e)
    {
      // mainly catching core doesn't have project with uuid error
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << e.what();
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (const std::exception& e)
    {
      // specific handling for all exceptions extending std::exception, except
      // std::runtime_error which is handled explicitly
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << e.what();
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (...)
    {
      // catch any other errors (that we have no information about)
      std::string msg = "Unknown failure occurred. Possible memory corruption";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << msg;
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
    }
  }

  seerep_server_util::createResponseFb(
      answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

grpc::Status FbTfService::GetFrames(
    grpc::ServerContext* context,
    const flatbuffers::grpc::Message<seerep::fb::FrameQuery>* request,
    flatbuffers::grpc::Message<seerep::fb::StringVector>* response)
{
  (void)context;  // ignore that variable without causing warnings
  boost::uuids::uuid uuid;
  try
  {
    boost::uuids::string_generator gen;
    uuid = gen(request->GetRoot()->projectuuid()->str());

    auto frames = tfFb->getFrames(uuid);

    flatbuffers::grpc::MessageBuilder builder;

    std::vector<flatbuffers::Offset<flatbuffers::String>> framesOffset;
    for (auto framename : tfFb->getFrames(uuid))
    {
      framesOffset.push_back(builder.CreateString(framename));
    }

    auto stringVectorOffset = builder.CreateVector(framesOffset);
    seerep::fb::StringVectorBuilder frameinfosbuilder(builder);
    frameinfosbuilder.add_stringVector(stringVectorOffset);
    auto frameinfosOffset = frameinfosbuilder.Finish();
    builder.Finish(frameinfosOffset);
    *response = builder.ReleaseMessage<seerep::fb::StringVector>();
    assert(response->Verify());
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from
    // string to uuid also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
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

grpc::Status FbTfService::GetTransformStamped(
    grpc::ServerContext* context,
    const flatbuffers::grpc::Message<seerep::fb::TransformStampedQuery>* request,
    flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response)
{
  (void)context;  // ignore that variable without causing warnings
  try
  {
    tfFb->getData(*request->GetRoot(), response);
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from
    // string to uuid also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
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
} /* namespace seerep_server */
