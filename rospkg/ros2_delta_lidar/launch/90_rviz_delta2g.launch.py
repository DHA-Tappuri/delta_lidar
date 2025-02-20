#!/usr/bin/env python3
# coding: utf-8

import os
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch_ros.actions                import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution
from launch_ros.substitutions          import FindPackageShare
from ament_index_python.packages       import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    other_launch_file = os.path.join( get_package_share_directory('ros2_delta_lidar'), 'launch', '00_delta2g.launch.py' )
    nodes_delta2g = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        launch_arguments={'some_arg': 'value'}.items()
    )

    rviz_config_file = PathJoinSubstitution([ FindPackageShare('ros2_delta_lidar'), 'config', 'delta2g.rviz' ])
    node_rviz = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        namespace  = 'rviz2',
        arguments  = ['-d', rviz_config_file]
    )

    ld.add_action(nodes_delta2g)
    ld.add_action(node_rviz)

    return ld
