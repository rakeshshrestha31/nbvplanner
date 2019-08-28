cmake_minimum_required(VERSION 2.8.3)
project(nbvplanner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  octomap_world
  tf
  tf2
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  kdtree
  multiagent_collision_check
  scene_completion_3d_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

add_service_files(
  FILES
  nbvp_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include tests/include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES nbvPlannerLib ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf kdtree
)

include_directories(
  include
  tests/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(nbvPlannerLib src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp)
add_executable(nbvPlanner src/nbv_planner_node.cpp src/mesh_structure.cpp src/nbvp.cpp src/rrt.cpp src/tree.cpp)
add_dependencies(nbvPlannerLib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nbvPlanner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(nbvPlannerLib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_link_libraries(nbvPlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

### Test executables ###
add_executable(original_gain_viz
  tests/original_gain_viz.cpp
  tests/utils.cpp
)

add_dependencies(original_gain_viz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(original_gain_viz
  nbvPlannerLib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

### Test executables ###
add_executable(predicted_gain_viz
  tests/predicted_gain_viz.cpp
  tests/utils.cpp
)

add_dependencies(predicted_gain_viz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(predicted_gain_viz
  nbvPlannerLib
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
