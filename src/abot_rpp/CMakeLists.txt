cmake_minimum_required(VERSION 3.0.2)
project(abot_rpp)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES abot_rpp
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# glog doesn't ship cmake configs on 18.04, use pkgconfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)

add_library(abot_rpp src/rpp_controller.cpp)
target_include_directories(abot_rpp PRIVATE ${GLOG_INCLUDE_DIRS})
target_link_libraries(abot_rpp ${catkin_LIBRARIES} ${GLOG_LIBRARIES})

install(TARGETS abot_rpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES rpp_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
