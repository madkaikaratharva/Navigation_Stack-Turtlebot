cmake_minimum_required(VERSION 3.8)
project(bumperbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)


# Include directories
include_directories(include)

# Set Dependencies
set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
)

# Add executables
add_executable(dijkstra_planner src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(astar_planner src/astar_planner.cpp)
ament_target_dependencies(astar_planner rclcpp nav_msgs geometry_msgs tf2_ros)


# Export the Plugins
add_library(dijkstra_planner_plugin SHARED src/dijkstra_planner_plugin.cpp)
ament_target_dependencies(dijkstra_planner_plugin rclcpp nav_msgs geometry_msgs tf2_ros nav2_core nav2_util nav2_costmap_2d pluginlib)

add_library(astar_planner_plugin SHARED src/astar_planner_plugin.cpp)
ament_target_dependencies(astar_planner_plugin rclcpp nav_msgs geometry_msgs tf2_ros nav2_core nav2_util nav2_costmap_2d pluginlib)

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

install(TARGETS
  dijkstra_planner_plugin
  astar_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES 
  global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the directories and scripts
install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  dijkstra_planner
  astar_planner
  DESTINATION lib/${PROJECT_NAME}
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

# Export Dependencies
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
