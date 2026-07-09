cmake_minimum_required(VERSION 3.10)
project(seerep_hdf5_core VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FindHighFive)

find_package(SeerepMsgs REQUIRED)
find_package(HighFive REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
  Boost
  COMPONENTS log log_setup
  REQUIRED
)

find_package(CGAL REQUIRED)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

enable_testing()
find_package(GTest 1.12.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS geometry_msgs)

configure_file(include/SeerepHdf5CoreConfig.h.in SeerepHdf5CoreConfig.h)

# message("HighFive Include Dirs: ${HighFive_INCLUDE_DIRS} Libraries:
# ${HighFive_LIBRARIES}")

include_directories(
  include
  ${SeerepMsgs_INCLUDE_DIRS}
  ${HighFive_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  seerephdf5core
  src/hdf5_core_general.cpp
  src/hdf5_core_image.cpp
  src/hdf5_core_instance.cpp
  src/hdf5_core_point_cloud.cpp
  src/hdf5_core_point.cpp
  src/hdf5_core_tf.cpp
  src/hdf5_core_cameraintrinsics.cpp
)

target_link_libraries(
  seerephdf5core
  ${SeerepMsgs_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${HighFive_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  ${catkin_LIBRARIES}
  CGAL::CGAL
)

# create executable with tests
add_executable(core_write_load_test test/core_write_load_test.cpp)
add_dependencies(
  core_write_load_test ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  core_write_load_test
  GTest::gtest
  GTest::gtest_main
  gmock_main
  ${catkin_LIBRARIES}
  ${HighFive_LIBRARIES}
  seerephdf5core
)
add_test(core_write_load_test core_write_load_test)

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(
  TARGETS seerephdf5core
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  cmake/SeerepHdf5CoreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5CoreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5CoreConfigVersion.cmake"
  VERSION "${seerep-hdf5-core_VERSION_MAJOR}.${seerep-hdf5-core_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5CoreConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5CoreConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHdf5Core
)

install(FILES "${PROJECT_BINARY_DIR}/SeerepHdf5CoreConfig.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(DIRECTORY include/seerep_hdf5_core
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES cmake/FindHighFive.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHdf5Core/Modules
)

# add PIC gcc flag
set(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
