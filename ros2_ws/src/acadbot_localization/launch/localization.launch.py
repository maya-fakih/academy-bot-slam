#!/usr/bin/env python3
"""localization.launch.py — bring up AMCL localization on a saved map.

Simulation + map_server + AMCL + RViz2 + localization_monitor, and nothing
else. No planner, no controller, no behaviour tree — this answers exactly one
question: where is the robot on this map?

    ros2 launch acadbot_localization localization.launch.py

Arguments:
    map:=<path>      occupancy grid .yaml to localise against
                     (default: acadbot_navigation/maps/academy_map.yaml)
    headless:=true   Gazebo server only — no GUI
    rviz:=false      skip RViz2 (no display, or CI)

AMCL publishes nothing until an initial pose is given. Use RViz's
2D Pose Estimate; localization_monitor warns until that happens.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('acadbot_gazebo')
    pkg_desc = get_package_share_directory('acadbot_description')
    pkg_nav = get_package_share_directory('acadbot_navigation')
    pkg_local = get_package_share_directory('acadbot_localization')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Defaults resolved from installed share paths, never hardcoded home dirs.
    default_map = os.path.join(pkg_nav, 'maps', 'academy_map_v2.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    monitor_config = os.path.join(pkg_local, 'config', 'localization_monitor.yaml')
    rviz_config = os.path.join(pkg_desc, 'rviz', 'nav2.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    # The robot in Gazebo. headless is forwarded, as mapping.launch.py does.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    # map_server + amcl + their lifecycle manager, already written by Nav2.
    # AMCL's parameters live in acadbot_navigation/config/nav2_params.yaml.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Gated: the C++ target is commented out in CMakeLists until
    # src/localization_monitor.cpp lands. Launch with monitor:=false to bring
    # up AMCL on its own before then.
    monitor = Node(
        package='acadbot_localization', executable='localization_monitor',
        name='localization_monitor',
        parameters=[monitor_config, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('monitor')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Occupancy grid .yaml for map_server to serve.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 parameter file holding the AMCL settings.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server-only (no GUI).'),
        DeclareLaunchArgument(
            'monitor', default_value='true',
            description='Start localization_monitor (needs the C++ node built).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2. Set false on a machine with no display.'),

        sim,
        localization,
        rviz,
        monitor,
    ])
