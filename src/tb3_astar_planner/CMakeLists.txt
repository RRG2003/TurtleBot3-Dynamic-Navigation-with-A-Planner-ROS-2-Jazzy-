cmake_minimum_required(VERSION 3.8)
project(tb3_astar_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(astar_planner SHARED src/astar_planner.cpp)
ament_target_dependencies(astar_planner
  nav2_core nav2_costmap_2d nav_msgs geometry_msgs rclcpp pluginlib)

pluginlib_export_plugin_description_file(nav2_core astar_planner_plugin.xml)

install(TARGETS astar_planner
  LIBRARY DESTINATION lib
)

install(FILES astar_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

