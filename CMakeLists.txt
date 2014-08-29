cmake_minimum_required(VERSION 2.4.6)

include(cmake/base.cmake)
include(cmake/pkg-config.cmake)
project(sot_sandbox)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_graph_bridge_msgs)

catkin_package(
 DEPENDS sot-core)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(start_controller src/start_controller.cpp)
ADD_REQUIRED_DEPENDENCY(sot-core)
pkg_config_use_dependency(start_controller sot-core)
target_link_libraries(start_controller ${catkin_LIBRARIES} dl)

