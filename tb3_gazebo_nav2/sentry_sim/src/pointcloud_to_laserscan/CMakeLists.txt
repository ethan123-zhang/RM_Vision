cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Vendored upstream (ros-perception/pointcloud_to_laserscan@442b332, jazzy) with two
# local changes, both for ROS 2 Humble / RDK S100 compatibility (see sentry_sim README):
#
#  1. The reverse LaserScan->PointCloud2 node needs laser_geometry, which is not
#     always installable (offline Humble images). The sentry stack only needs
#     PointCloud2 -> LaserScan, so the reverse node is opt-in (OFF by default).
#  2. tf2_sensor_msgs is header-only on Humble (no CMake target) but exports an
#     INTERFACE target on Jazzy -> guard it, and add ${sensor_msgs_TARGETS}
#     (Humble does not get sensor_msgs through the tf2_sensor_msgs target).
option(BUILD_LASERSCAN_TO_POINTCLOUD
  "Build the reverse LaserScan->PointCloud2 node (needs laser_geometry)" OFF)

if(BUILD_LASERSCAN_TO_POINTCLOUD)
  find_package(laser_geometry REQUIRED)
endif()

function(p2l_link_tf2_sensor_msgs target)
  if(TARGET tf2_sensor_msgs::tf2_sensor_msgs)
    target_link_libraries(${target} tf2_sensor_msgs::tf2_sensor_msgs)
  else()
    target_include_directories(${target} SYSTEM PUBLIC
      ${tf2_sensor_msgs_INCLUDE_DIRS})
  endif()
endfunction()

add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)
target_compile_definitions(pointcloud_to_laserscan
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
target_include_directories(pointcloud_to_laserscan PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pointcloud_to_laserscan
  ${sensor_msgs_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
)
p2l_link_tf2_sensor_msgs(pointcloud_to_laserscan)
rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

set(P2L_INSTALL_TARGETS "")
if(BUILD_LASERSCAN_TO_POINTCLOUD)
  add_library(laserscan_to_pointcloud SHARED
    src/laserscan_to_pointcloud_node.cpp)
  target_compile_definitions(laserscan_to_pointcloud
    PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
  target_include_directories(laserscan_to_pointcloud PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
  target_link_libraries(laserscan_to_pointcloud
    ${sensor_msgs_TARGETS}
    laser_geometry::laser_geometry
    message_filters::message_filters
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_ros::tf2_ros
  )
  p2l_link_tf2_sensor_msgs(laserscan_to_pointcloud)
  rclcpp_components_register_node(laserscan_to_pointcloud
    PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
    EXECUTABLE laserscan_to_pointcloud_node)
  set(P2L_INSTALL_TARGETS laserscan_to_pointcloud)
endif()

add_executable(dummy_pointcloud_publisher
  src/dummy_pointcloud_publisher.cpp
)
target_link_libraries(dummy_pointcloud_publisher
  rclcpp::rclcpp
  "${sensor_msgs_TARGETS}"
)

install(TARGETS
  pointcloud_to_laserscan
  ${P2L_INSTALL_TARGETS}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS
  dummy_pointcloud_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

if(BUILD_LASERSCAN_TO_POINTCLOUD)
  ament_export_dependencies(laser_geometry)
endif()

ament_package()