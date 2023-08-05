cmake_minimum_required(VERSION 3.5.1)
project(gazebo_wheel_slip_plugin)

find_package(gazebo REQUIRED)
find_package(ignition-common3 QUIET REQUIRED COMPONENTS profiler)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  cv_bridge
  polled_camera
  diagnostic_updater
  camera_info_manager
  std_msgs
  visualization_msgs
)

# Through transitive dependencies in the packages above, gazebo_plugins depends
# on Simbody.  There is a bug in the Ubuntu Artful (17.10) version of the
# Simbody package where it includes /usr/lib/libblas.so and
# /usr/lib/liblapack.so in the CMake list of libraries even though neither of
# those two paths exist (they both really live in /usr/lib/<arch>-linux-gnu).
# We remove these two during build-time on artful below; this works because
# they both will get resolved to the proper paths during runtime linking.
find_program(LSB_RELEASE_EXEC lsb_release)
if(NOT LSB_RELEASE_EXEC STREQUAL "LSB_RELEASE_EXEC-NOTFOUND")
  execute_process(COMMAND ${LSB_RELEASE_EXEC} -cs
    OUTPUT_VARIABLE OS_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(OS_CODENAME STREQUAL "artful")
    list(FILTER catkin_LIBRARIES EXCLUDE REGEX "/usr/lib/libblas.so")
    list(FILTER catkin_LIBRARIES EXCLUDE REGEX "/usr/lib/liblapack.so")
    list(FILTER GAZEBO_LIBRARIES EXCLUDE REGEX "/usr/lib/libblas.so")
    list(FILTER GAZEBO_LIBRARIES EXCLUDE REGEX "/usr/lib/liblapack.so")
  endif()
endif()

find_package(Boost REQUIRED COMPONENTS thread)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()


generate_dynamic_reconfigure_options(
  cfg/WheelSlip.cfg
  #cfg/WheelPacejka.cfg
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  gazebo_ros_utils
  gazebo_ros_template
  gazebo_ros_force
  gazebo_ros_joint_state_publisher
  gazebo_ros_joint_pose_trajectory
  gazebo_ros_diff_drive
  gazebo_ros_tricycle_drive
  gazebo_ros_skid_steer_drive
  gazebo_ros_planar_move
  gazebo_ros_range

  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  rosgraph_msgs
  trajectory_msgs
  image_transport
  rosconsole
  std_msgs
  visualization_msgs
)
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

## Plugins

if (NOT GAZEBO_VERSION VERSION_LESS 9.5)
  add_library(gazebo_ros_wheel_slip src/gazebo_ros_wheel_slip.cpp src/wheel_slip_pacejka.cc)

  add_dependencies(gazebo_ros_wheel_slip ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
  target_link_libraries(gazebo_ros_wheel_slip
    ${Boost_LIBRARIES} ${catkin_LIBRARIES})

  target_link_libraries(gazebo_ros_wheel_slip ignition-common3::profiler)
endif()


if (NOT GAZEBO_VERSION VERSION_LESS 9.5)
  install(TARGETS gazebo_ros_wheel_slip
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
endif()

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
