# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Packages
#     pkg_tb3_gazebo = FindPackageShare("turtlebot3_gazebo")
#     pkg_tb3_nav = FindPackageShare("tb3_dynamic_nav_jazzy")
#     pkg_nav2 = FindPackageShare("nav2_bringup")

#     # World file
#     world_file = PathJoinSubstitution([
#         pkg_tb3_gazebo,
#         "worlds",
#         "empty.world"
#     ])

#     # Nav2 params file
#     nav2_params_file = PathJoinSubstitution([
#         pkg_tb3_nav,
#         "config",
#         "nav2_params.yaml"
#     ])

#     # RViz config
#     rviz_config_file = PathJoinSubstitution([
#         pkg_nav2,
#         "rviz",
#         "nav2_default_view.rviz"
#     ])

#     return LaunchDescription([
#         # Launch Gazebo with TurtleBot3 empty world
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 PathJoinSubstitution([pkg_tb3_gazebo, "launch", "turtlebot3_world.launch.py"])
#             ),
#             launch_arguments={
#                 "world": world_file,
#             }.items(),
#         ),

#         # Launch Navigation2 stack
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 PathJoinSubstitution([pkg_nav2, "launch", "bringup_launch.py"])
#             ),
#             launch_arguments={
#                 "slam": "False",
#                 "use_sim_time": "True",
#                 "params_file": nav2_params_file,
#             }.items(),
#         ),

#         # Launch RViz2 for visualization
#         Node(
#             package="rviz2",
#             executable="rviz2",
#             name="rviz2",
#             arguments=["-d", rviz_config_file],
#             output="screen"
#         ),
#     ])
# tb3_gz_nav.launch.py  (replace old file with this)


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Packages
    pkg_tb3_gazebo = FindPackageShare("turtlebot3_gazebo")
    pkg_tb3_nav = FindPackageShare("tb3_dynamic_nav_jazzy")
    pkg_nav2 = FindPackageShare("nav2_bringup")

    # World file (simple TurtleBot3 world)
    world_file = PathJoinSubstitution([
        pkg_tb3_gazebo,
        "worlds",
        "turtlebot3_world.world"
    ])

    # Nav2 params (with A* plugin configured)
    nav2_params_file = PathJoinSubstitution([
        pkg_tb3_nav,
        "config",
        "nav2_params.yaml"
    ])

    # RViz config
    rviz_config_file = PathJoinSubstitution([
        pkg_tb3_nav,
        "rviz",
        "slam_nav_view.rviz"
    ])

    return LaunchDescription([
        # --- Gazebo simulation ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_tb3_gazebo, "launch", "turtlebot3_world.launch.py"])
            ),
        ),

        # --- Navigation2 with SLAM ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "True",                # Enable online SLAM
                "use_sim_time": "True",
                "params_file": nav2_params_file,
            }.items(),
        ),

        # --- RViz2 visualization ---
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])
