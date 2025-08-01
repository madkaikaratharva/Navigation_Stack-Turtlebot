cmake_minimum_required(VERSION 3.8)
project(bumperbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(pd_motion_planner src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)


# Install Plugins as libraries
add_library(pd_motion_planner_plugin SHARED src/pd_motion_planner_plugin.cpp)
ament_target_dependencies(pd_motion_planner_plugin rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_util nav2_costmap_2d pluginlib)

add_library(pure_pursuit_plugin SHARED src/pure_pursuit_plugin.cpp)
ament_target_dependencies(pure_pursuit_plugin rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_util nav2_costmap_2d pluginlib)

pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)

install(TARGETS
  pd_motion_planner_plugin
  pure_pursuit_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES motion_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the directories and scripts
install(
  DIRECTORY include
  DESTINATION include
)

# Install the executables
install(TARGETS
  pd_motion_planner
  pure_pursuit
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

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs nav2_core nav2_util nav2_costmap_2d pluginlib)
ament_export_libraries(pd_motion_planner_plugin pure_pursuit_plugin)
ament_package()
