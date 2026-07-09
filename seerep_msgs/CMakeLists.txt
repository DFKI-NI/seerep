cmake_minimum_required(VERSION 3.10)
project(seerep_msgs VERSION 0.1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(BuildFlatBuffers)
include(FindFlatBuffers)
include(FindProtobuf)

find_package(Protobuf REQUIRED)
find_package(Flatbuffers REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED)

find_package(CGAL REQUIRED)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

set(MY_PROTO_FILES
    protos/boundingbox.proto
    protos/camera_intrinsics.proto
    protos/camera_intrinsics_query.proto
    protos/datatype.proto
    protos/frame_query.proto
    protos/geodetic_coordinates.proto
    protos/header.proto
    protos/image.proto
    protos/label_category.proto
    protos/label.proto
    protos/point_cloud_2.proto
    protos/point_field.proto
    protos/point_stamped.proto
    protos/point.proto
    protos/point2d.proto
    protos/polygon2d.proto
    protos/project_info.proto
    protos/project_infos.proto
    protos/projectCreation.proto
    protos/quaternion.proto
    protos/query.proto
    protos/region_of_interest.proto
    protos/server_response.proto
    protos/string_vector.proto
    protos/time_interval.proto
    protos/timestamp.proto
    protos/transform_stamped.proto
    protos/transform_stamped_query.proto
    protos/transform.proto
    protos/uuid_datatype_with_category.proto
    protos/uuid_datatype_pair.proto
    protos/vector3.proto
)

set(MY_FBS_FILES
    fbs/attributes_stamped.fbs
    fbs/boundingbox.fbs
    fbs/camera_intrinsics.fbs
    fbs/camera_intrinsics_query.fbs
    fbs/dataset_uuid_label.fbs
    fbs/datatype.fbs
    fbs/empty.fbs
    fbs/frame_query.fbs
    fbs/geodeticCoordinates.fbs
    fbs/header.fbs
    fbs/image.fbs
    fbs/label_category.fbs
    fbs/label.fbs
    fbs/point_cloud_2.fbs
    fbs/point_field.fbs
    fbs/point_stamped.fbs
    fbs/point.fbs
    fbs/point2d.fbs
    fbs/polygon2d.fbs
    fbs/project_info.fbs
    fbs/project_infos.fbs
    fbs/projectCreation.fbs
    fbs/quaternion.fbs
    fbs/query.fbs
    fbs/query_instance.fbs
    fbs/region_of_interest.fbs
    fbs/server_response.fbs
    fbs/sparql_query.fbs
    fbs/string_vector.fbs
    fbs/time_interval.fbs
    fbs/timestamp.fbs
    fbs/transform_stamped.fbs
    fbs/transform_stamped_interval_query.fbs
    fbs/transform_stamped_query.fbs
    fbs/transform.fbs
    fbs/union_map_entry.fbs
    fbs/uuids_per_project.fbs
    fbs/uuids_project_pair.fbs
    fbs/uuid_datatype_with_category.fbs
    fbs/uuid_datatype_pair.fbs
    fbs/vector3.fbs
)

set(MY_CORE_MSGS
    ${CMAKE_CURRENT_SOURCE_DIR}/core/aabb.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/dataset_indexable.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/camera_intrinsics.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/camera_intrinsics_query.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/header.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/datatype.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/label.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/label_category.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/label_datumaro.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/project_info.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/polygon2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query_result_project.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query_result.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query_tf.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/region_of_interest.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/sparql_query.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/timeinterval.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/timestamp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/timestamp_frame_mesh.h
)

add_library(SeerepCoreMsgsTarget INTERFACE)
target_sources(SeerepCoreMsgsTarget INTERFACE ${MY_CORE_MSGS})
target_include_directories(
  SeerepCoreMsgsTarget INTERFACE core/ ${Boost_INCLUDE_DIRS}
)
target_link_libraries(SeerepCoreMsgsTarget ${Boost_LIBRARIES})

build_flatbuffers(
  "${MY_FBS_FILES}"
  ""
  fbschemas
  ""
  "${CMAKE_CURRENT_BINARY_DIR}/fbs"
  "${CMAKE_CURRENT_BINARY_DIR}/fbs"
  ""
)

add_library(FlatBuffersTarget INTERFACE)
target_include_directories(
  FlatBuffersTarget INTERFACE ${FLATBUFFERS_INCLUDE_DIR}
)
add_dependencies(FlatBuffersTarget fbschemas)

get_target_property(FlatBuffers_HEADERS fbschemas GENERATED_HEADERS)

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${MY_PROTO_FILES})

protobuf_generate_python(PY_SOURCES ${MY_PROTO_FILES})

add_custom_target(protoPythonTarget ALL DEPENDS ${PY_SOURCES})

add_custom_command(
  TARGET protoPythonTarget
  POST_BUILD
  COMMAND bash -c "sed -i -E \"s/^import.*_pb2/from seerep.pb &/\" *_pb2*.py"
  COMMAND bash -c "mkdir -p protos/seerep/pb && mv *_pb2.py protos/seerep/pb"
  COMMAND
    bash -c
    "touch protos/seerep/pb/__init__.py && touch protos/seerep/__init__.py"
  VERBATIM
)

include_directories(${CMAKE_BUILD_RPATH} ${PROTOBUF_INCLUDE_DIR})

configure_file(SeerepMsgsConfig.h.in SeerepMsgsConfig.h)

add_library(seerepmsgs ${PROTO_SOURCES})

add_dependencies(seerepmsgs FlatBuffersTarget)
add_dependencies(seerepmsgs SeerepCoreMsgsTarget)

set(INSTALL_HEADERS ${PROTO_HEADERS} ${FlatBuffers_HEADERS})

set_target_properties(seerepmsgs PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(PROTOBUF_IMPORT_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(FLATBUFFERS_IMPORT_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(
  TARGETS seerepmsgs
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/SeerepMsgsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR
            CMAKE_INSTALL_LIBDIR
            PROTOBUF_IMPORT_DIR
            FLATBUFFERS_IMPORT_DIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfigVersion.cmake"
  VERSION "${seerep-msgs_VERSION_MAJOR}.${seerep-msgs_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
)

set(SEEREP_COM_DIR_FB ${CMAKE_CURRENT_BINARY_DIR}/../seerep_com/fbs)

install(
  FILES ${MY_FBS_FILES}/
  PERMISSIONS
    OWNER_READ
    OWNER_WRITE
    OWNER_EXECUTE
    GROUP_READ
    GROUP_WRITE
    WORLD_READ
    WORLD_WRITE
    WORLD_EXECUTE
  DESTINATION ${SEEREP_COM_DIR_FB}
)

# install the configuration file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepMsgs
)

install(FILES "${PROJECT_BINARY_DIR}/SeerepMsgsConfig.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# install ProtoBuf proto files
install(
  DIRECTORY protos/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.proto"
)

# install FlatBuffers fbs files
install(
  DIRECTORY fbs/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.fbs"
)

# install seerepCoreMsgs files
install(
  DIRECTORY core/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
)

# add PIC gcc flag
set(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
