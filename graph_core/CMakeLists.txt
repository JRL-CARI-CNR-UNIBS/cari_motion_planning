#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.8)
project(graph_core)
add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)


# find_package(catkin REQUIRED COMPONENTS
#   actionlib
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   roscpp
#   rosdyn_core
#   human_probablistic_occupancy
#   moveit_visual_tools
#   rviz_visual_tools
#   eigen_conversions
#   geometry_msgs
#   velocity_scaling_iso15066
#   )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES graph_core
#   CATKIN_DEPENDS actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp rosdyn_core human_probablistic_occupancy moveit_visual_tools rviz_visual_tools eigen_conversions geometry_msgs velocity_scaling_iso15066
#   DEPENDS
#   )

find_package(ament_cmake REQUIRED)

#find_package(actionlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rosdyn_core REQUIRED)
#find_package(human_probablistic_occupancy REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
#find_package(eigen_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(velocity_scaling_iso15066 REQUIRED)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   )

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/metrics.cpp
  src/${PROJECT_NAME}/goal_cost_function.cpp
  # src/${PROJECT_NAME}/avoidance_goal_cost_function.cpp
  #src/${PROJECT_NAME}/avoidance_metrics.cpp
  #src/${PROJECT_NAME}/avoidance_time_metrics.cpp
  #src/${PROJECT_NAME}/avoidance_probabilistic_time_metrics.cpp
  #src/${PROJECT_NAME}/occupancy_metrics.cpp
  #src/${PROJECT_NAME}/time_metrics.cpp
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
  # src/${PROJECT_NAME}/multi_goal_selection/goal_selection_manager.cpp
  src/${PROJECT_NAME}/graph/node.cpp
  src/${PROJECT_NAME}/graph/connection.cpp
  src/${PROJECT_NAME}/graph/net_connection.cpp
  src/${PROJECT_NAME}/graph/net.cpp
  src/${PROJECT_NAME}/graph/tree.cpp
  src/${PROJECT_NAME}/graph/subtree.cpp
  src/${PROJECT_NAME}/graph/path.cpp
  src/${PROJECT_NAME}/graph/graph_display.cpp
  # src/${PROJECT_NAME}/solvers/multigoal.cpp
  # src/${PROJECT_NAME}/solvers/time_multigoal.cpp
  src/${PROJECT_NAME}/solvers/tree_solver.cpp
  # src/${PROJECT_NAME}/solvers/birrt.cpp
  # src/${PROJECT_NAME}/solvers/rrt_star.cpp
  # src/${PROJECT_NAME}/solvers/rrt.cpp
  # src/${PROJECT_NAME}/solvers/anytime_rrt.cpp
  src/${PROJECT_NAME}/solvers/path_solver.cpp
  src/${PROJECT_NAME}/datastructure/kdtree.cpp
  src/${PROJECT_NAME}/datastructure/vector.cpp

  )
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  #actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
  # rosdyn_core
  #human_probablistic_occupancy
  moveit_visual_tools
  rviz_visual_tools
 # eigen_conversions
  geometry_msgs
  # velocity_scaling_iso15066
  )

# add_executable(goal_selection_test_node tests/goal_selection_test_node.cpp)
# add_dependencies(goal_selection_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(goal_selection_test_node
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
# )


# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#   )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
