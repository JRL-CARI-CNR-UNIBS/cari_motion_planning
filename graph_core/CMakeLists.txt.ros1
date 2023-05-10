cmake_minimum_required(VERSION 2.8.3)
project(graph_core)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rosdyn_core
  human_probablistic_occupancy
  moveit_visual_tools
  rviz_visual_tools
  eigen_conversions
  geometry_msgs
  velocity_scaling_iso15066
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_core
  CATKIN_DEPENDS actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp rosdyn_core human_probablistic_occupancy moveit_visual_tools rviz_visual_tools eigen_conversions geometry_msgs velocity_scaling_iso15066
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/metrics.cpp
  src/${PROJECT_NAME}/goal_cost_function.cpp
  src/${PROJECT_NAME}/avoidance_goal_cost_function.cpp
  src/${PROJECT_NAME}/avoidance_metrics.cpp
  src/${PROJECT_NAME}/avoidance_time_metrics.cpp
  src/${PROJECT_NAME}/avoidance_probabilistic_time_metrics.cpp
  src/${PROJECT_NAME}/occupancy_metrics.cpp
  src/${PROJECT_NAME}/time_metrics.cpp
  src/${PROJECT_NAME}/sampler.cpp
  src/${PROJECT_NAME}/local_informed_sampler.cpp
  src/${PROJECT_NAME}/tube_informed_sampler.cpp
  src/${PROJECT_NAME}/time_sampler.cpp
  src/${PROJECT_NAME}/util.cpp
  src/${PROJECT_NAME}/moveit_collision_checker.cpp
  src/${PROJECT_NAME}/parallel_moveit_collision_checker.cpp
  src/${PROJECT_NAME}/narrow_pass_checker.cpp
  src/${PROJECT_NAME}/boxes_checker.cpp
  src/${PROJECT_NAME}/collision_checker.cpp
  src/${PROJECT_NAME}/multi_goal_selection/goal_selection_manager.cpp
  src/${PROJECT_NAME}/graph/node.cpp
  src/${PROJECT_NAME}/graph/connection.cpp
  src/${PROJECT_NAME}/graph/net_connection.cpp
  src/${PROJECT_NAME}/graph/net.cpp
  src/${PROJECT_NAME}/graph/tree.cpp
  src/${PROJECT_NAME}/graph/subtree.cpp
  src/${PROJECT_NAME}/graph/path.cpp
  src/${PROJECT_NAME}/graph/graph_display.cpp
  src/${PROJECT_NAME}/solvers/multigoal.cpp
  src/${PROJECT_NAME}/solvers/time_multigoal.cpp
  src/${PROJECT_NAME}/solvers/tree_solver.cpp
  src/${PROJECT_NAME}/solvers/birrt.cpp
  src/${PROJECT_NAME}/solvers/rrt_star.cpp
  src/${PROJECT_NAME}/solvers/rrt.cpp
  src/${PROJECT_NAME}/solvers/anytime_rrt.cpp
  src/${PROJECT_NAME}/solvers/path_solver.cpp
  src/${PROJECT_NAME}/datastructure/kdtree.cpp
  src/${PROJECT_NAME}/datastructure/vector.cpp

  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(goal_selection_test_node tests/goal_selection_test_node.cpp)
add_dependencies(goal_selection_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_selection_test_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
