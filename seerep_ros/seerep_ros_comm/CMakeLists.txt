#catkin_lint: ignore critical_var_overwrite

cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_comm VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

find_package(SeerepCom REQUIRED)

find_package(SeerepHdf5Pb REQUIRED)
find_package(SeerepHdf5Fb REQUIRED)
find_package(seerep_hdf5_ros REQUIRED)
find_package(SeerepHdf5Core REQUIRED)
find_package(SeerepCoreFb REQUIRED)
find_package(SeerepCore REQUIRED)
find_package(seerep_ros_conversions_pb REQUIRED)
find_package(seerep_ros_conversions_fb REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV)

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             topic_tools
             std_msgs
             geometry_msgs
             geographic_msgs
             sensor_msgs
             roscpp
             tf
             rosbag
)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
  Boost
  COMPONENTS log log_setup
  REQUIRED
)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  # LIBRARIES
  CATKIN_DEPENDS
  cv_bridge
  std_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
  roscpp
  seerep_ros_conversions_pb
  seerep_ros_conversions_fb
  seerep_hdf5_ros
  roscpp
  topic_tools
  tf
  rosbag
  DEPENDS
  GeographicLib
)

include_directories(
  include
  ${SeerepCom_INCLUDE_DIRS}
  ${SeerepHdf5Pb_INCLUDE_DIRS}
  ${SeerepHdf5Fb_INCLUDE_DIRS}
  ${seerep_hdf5_ros_INCLUDE_DIRS}
  ${SeerepHdf5Core_INCLUDE_DIRS}
  ${SeerepCoreFb_INCLUDE_DIRS}
  ${seerep_ros_conversions_pb_INCLUDE_DIRS}
  ${seerep_ros_conversions_fb_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rostopic_dumper src/rostopic_dumper.cpp)
add_executable(rosbag_dumper_aitf src/rosbag_dumper_aitf.cpp)
add_executable(rosbag_dumper src/rosbag_dumper.cpp)
add_executable(point_dumper src/point_dumper.cpp)

add_dependencies(
  rostopic_dumper ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  rosbag_dumper_aitf ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  rosbag_dumper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  point_dumper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  rostopic_dumper
  ${SeerepCom_LIBRARIES}
  ${seerep_ros_conversions_pb_LIBRARIES}
  ${SeerepHdf5Pb_LIBRARIES}
  ${seerep_hdf5_ros_LIBRARIES}
  ${SeerepHdf5Core_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
)

target_link_libraries(
  rosbag_dumper_aitf
  ${SeerepCore_LIBRARIES}
  ${seerep_hdf5_ros_LIBRARIES}
  ${catkin_LIBRARIES}
  gRPC::grpc++_reflection
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  ${OpenCV_LIBRARIES}
  jsoncpp
)

target_link_libraries(
  rosbag_dumper
  ${seerep_ros_conversions_fb_LIBRARIES}
  ${SeerepHdf5Fb_LIBRARIES}
  ${SeerepHdf5Core_LIBRARIES}
  ${SeerepCoreFb_LIBRARIES}
  ${SeerepCore_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  gRPC::grpc++_reflection
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  jsoncpp
  ${CURL_LIBRARIES}
)

target_link_libraries(
  point_dumper
  ${SeerepHdf5Core_LIBRARIES}
  ${SeerepHdf5Fb_LIBRARIES}
  ${catkin_LIBRARIES}
  gRPC::grpc++_reflection
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  jsoncpp
  ${CURL_LIBRARIES}
)

install(
  TARGETS rostopic_dumper
          rosbag_dumper_aitf
          rosbag_dumper
          point_dumper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
