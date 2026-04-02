cmake_minimum_required(VERSION 3.22)

# Fix duplicate uninstall target conflict between ament_cmake and Eigen
if(POLICY CMP0002)
  cmake_policy(SET CMP0002 OLD)
endif()

project(basalt_ros2 VERSION 0.1.0 LANGUAGES CXX C)

find_package(ament_cmake REQUIRED)

##################################################
###########  PATHS & MODULE SEARCH  ##############

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

if(NOT EIGEN_ROOT)
  set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers/thirdparty/eigen")
endif()

##################################################
################# COMPILER FLAGS #################

if (NOT CMAKE_C_COMPILER_LAUNCHER AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  endif()
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CXX_MARCH)
  set(CXX_MARCH native)
endif()

if(NOT APPLE OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(BASALT_MARCH_FLAGS "-march=${CXX_MARCH}")
else()
  message(STATUS "Running on Apple ${CMAKE_SYSTEM_PROCESSOR}. Disabled -march flag.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

set(BASALT_CXX_FLAGS "-Wall -Wextra -Wno-error=unused-parameter -ftemplate-backtrace-limit=0")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-error=maybe-uninitialized -Wno-error=implicit-fallthrough")
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-error=strict-aliasing -Wno-error=stringop-overflow")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 9)
    set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-error=deprecated-copy")
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0)
    set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-parentheses")
  endif()
endif()

set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS "${BASALT_CXX_FLAGS} ${BASALT_MARCH_FLAGS} ${CMAKE_CXX_FLAGS}")

if(UNIX AND NOT APPLE)
  set(STD_CXX_FS stdc++fs)
endif()

##################################################
################ TEMPLATE OPTIONS ################

option(BASALT_INSTANTIATIONS_DOUBLE "Instantiate templates for Scalar=double." ON)
option(BASALT_INSTANTIATIONS_FLOAT  "Instantiate templates for Scalar=float."  OFF)

if(BASALT_INSTANTIATIONS_DOUBLE)
  list(APPEND BASALT_COMPILE_DEFINITIONS BASALT_INSTANTIATIONS_DOUBLE)
endif()
if(BASALT_INSTANTIATIONS_FLOAT)
  list(APPEND BASALT_COMPILE_DEFINITIONS BASALT_INSTANTIATIONS_FLOAT)
endif()

##################################################
############### SYSTEM DEPENDENCIES ##############

set(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
find_package(Eigen3 3.4.0 EXACT REQUIRED MODULE)
message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")

find_package(TBB REQUIRED)
message(STATUS "Found TBB ${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR}")
if(TBB_INTERFACE_VERSION LESS 11004)
  add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui)
message(STATUS "Found OpenCV ${OpenCV_VERSION}")
add_library(basalt::opencv INTERFACE IMPORTED)
set_property(TARGET basalt::opencv PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})
set_property(TARGET basalt::opencv PROPERTY INTERFACE_LINK_LIBRARIES ${OpenCV_LIBS})

find_package(fmt REQUIRED)
message(STATUS "Found {fmt} ${fmt_VERSION}")

# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)
message(STATUS "Found yaml-cpp ${yaml-cpp_VERSION}")

find_package(rosbag2_compression QUIET)
if(rosbag2_compression_FOUND)
  message(STATUS "Found rosbag2_compression support")
endif()

##################################################
############### THIRDPARTY SUBMODULES #############

set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_UNINSTALL OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(SKIP_INSTALL_ALL ON)
set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY ON)

add_subdirectory(thirdparty)

# Scoped interface targets for vendored single-header libs
add_library(basalt::cli11 INTERFACE IMPORTED)
set_property(TARGET basalt::cli11 PROPERTY INTERFACE_INCLUDE_DIRECTORIES
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/CLI11/include)

add_library(basalt::magic_enum INTERFACE IMPORTED)
set_property(TARGET basalt::magic_enum PROPERTY INTERFACE_INCLUDE_DIRECTORIES
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/magic_enum/include)

##################################################
################# BASALT CORE LIBRARY #############

# ROS2 bag support compile definitions
add_definitions(-DBASALT_USE_ROS2)
if(rosbag2_compression_FOUND)
  add_definitions(-DBASALT_ROS2_COMPRESSION_SUPPORT)
  message(STATUS "ROS2 compression support enabled")
endif()

add_library(basalt SHARED)

target_sources(basalt
  PUBLIC
    # Calibration
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/calibration/aprilgrid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/calibration/calibration_helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/calibration/cam_calib.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/calibration/cam_imu_calib.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/calibration/vignette.h
    # IO
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/io/dataset_io.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/io/dataset_io_rosbag2.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/io/rosbag2_storage_detector.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/io/marg_data_io.h
    # Optical flow
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optical_flow/optical_flow.h
    # VIO estimator
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/vio_estimator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sqrt_keypoint_vio.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sqrt_keypoint_vo.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/ba_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sqrt_ba_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sc_ba_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/landmark_database.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/marg_helper.h
  PRIVATE
    # Calibration sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/aprilgrid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/calibraiton_helper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calibration/vignette.cpp
    # IO sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io/dataset_io.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io/dataset_io_rosbag2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io/marg_data_io.cpp
    # Algorithm sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/landmark_block.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_abs_qr.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_abs_sc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_rel_sc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/optical_flow/optical_flow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/keypoints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/system_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/time_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/vio_config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/ba_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/landmark_database.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/marg_helper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sc_ba_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sqrt_ba_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sqrt_keypoint_vio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sqrt_keypoint_vo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/vio_estimator.cpp
)

target_include_directories(basalt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rosbag2_cpp_INCLUDE_DIRS}
  ${rosbag2_storage_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

target_link_libraries(basalt
  PUBLIC  ${STD_CXX_FS} basalt::opencv basalt::basalt-headers TBB::tbb
  PRIVATE basalt::magic_enum rosbag apriltag opengv nlohmann::json fmt::fmt
          yaml-cpp
          ${rosbag2_cpp_LIBRARIES}
          ${rosbag2_storage_LIBRARIES}
          ${rosbag2_storage_default_plugins_LIBRARIES}
          ${rclcpp_LIBRARIES}
          ${sensor_msgs_LIBRARIES}
          ${geometry_msgs_LIBRARIES}
)

target_compile_definitions(basalt PUBLIC ${BASALT_COMPILE_DEFINITIONS})

if(rosbag2_compression_FOUND)
  target_include_directories(basalt PRIVATE ${rosbag2_compression_INCLUDE_DIRS})
  target_link_libraries(basalt PRIVATE ${rosbag2_compression_LIBRARIES})
endif()

##################################################
############# CALIBRATION EXECUTABLES ############

option(BASALT_BUILD_CALIBRATION_TOOLS "Build basalt_calibrate and basalt_calibrate_imu" ON)

if(BASALT_BUILD_CALIBRATION_TOOLS)
  add_executable(basalt_calibrate
    src/calibrate.cpp
    src/calibration/cam_calib.cpp
  )
  target_link_libraries(basalt_calibrate basalt pangolin basalt::cli11)

  add_executable(basalt_calibrate_imu
    src/calibrate_imu.cpp
    src/calibration/cam_imu_calib.cpp
  )
  target_link_libraries(basalt_calibrate_imu basalt pangolin basalt::cli11)
endif()

##################################################
############# ROS2 DEVICE WRAPPER LIBRARY ########

add_library(basalt_ros2_lib STATIC
  src/device/ros_camera.cpp
  src/device/ros_imu.cpp
)

target_include_directories(basalt_ros2_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(basalt_ros2_lib
  rclcpp
  cv_bridge
  image_transport
  message_filters
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

target_link_libraries(basalt_ros2_lib basalt TBB::tbb fmt::fmt ${OpenCV_LIBRARIES})

##################################################
############# VISUAL ODOMETRY ROS2 NODE ##########

add_executable(visual_odometry_node
  src/node/visual_odometer_node.cpp
)

ament_target_dependencies(visual_odometry_node
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  image_transport
  message_filters
  cv_bridge
)

target_link_libraries(visual_odometry_node basalt_ros2_lib fmt::fmt TBB::tbb yaml-cpp)

##################################################
######## ROS2 COMPOSABLE NODES LIBRARY ###########

add_library(basalt_ros2_components SHARED
  src/node/visual_odometer_node.cpp
  src/node/visual_odometer_component.cpp
  src/node/calibration_publisher_node.cpp
  src/node/calibration_publisher_component.cpp
  src/device/ros_camera.cpp
  src/device/ros_imu.cpp
)

target_include_directories(basalt_ros2_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(basalt_ros2_components
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  image_transport
  message_filters
  cv_bridge
)

target_link_libraries(basalt_ros2_components basalt fmt::fmt TBB::tbb yaml-cpp ${OpenCV_LIBRARIES})

# Register both composable nodes
rclcpp_components_register_nodes(basalt_ros2_components
  PLUGIN "basalt::VisualOdometerNode"
  EXECUTABLE visual_odometry_node_composable
)

rclcpp_components_register_nodes(basalt_ros2_components
  PLUGIN "basalt::CalibrationPublisherNode"
  EXECUTABLE calibration_publisher_composable
)

##################################################
#################### INSTALL ####################

install(TARGETS visual_odometry_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BASALT_BUILD_CALIBRATION_TOOLS)
  install(TARGETS basalt_calibrate basalt_calibrate_imu
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS basalt basalt_ros2_lib basalt_ros2_components
  EXPORT basalt_ros2Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include
)

file(GLOB SCRIPTS_TO_INSTALL "${CMAKE_CURRENT_SOURCE_DIR}/scripts/basalt_*.py")
if(SCRIPTS_TO_INSTALL)
  install(PROGRAMS ${SCRIPTS_TO_INSTALL} DESTINATION lib/${PROJECT_NAME})
endif()

##################################################
############# EIGEN INSTALL GUARD ################
# Eigen's CMakeLists.txt has two separate install() calls:
#   - install(TARGETS eigen EXPORT Eigen3Targets)  → suppressed by EXCLUDE_FROM_ALL
#   - install(FILES ... Eigen3Config.cmake ...)     → NOT suppressed by EXCLUDE_FROM_ALL
# This leaves a broken Eigen3Config.cmake (referencing a missing Eigen3Targets.cmake)
# in the colcon install prefix, causing every other workspace package that calls
# find_package(Eigen3) to fail.  Remove those artifacts as a post-install step.
install(CODE "
  file(REMOVE_RECURSE \"\${CMAKE_INSTALL_PREFIX}/share/eigen3\")
")

##################################################
################# ROS2 EXPORT ####################

ament_package()
