#!/usr/bin/env python3
# coding: utf-8

from launch                   import LaunchDescription
from launch_ros.actions       import Node
from launch.substitutions     import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    node_lidar = Node(
        package    = 'ros2_delta_lidar',
        executable = 'delta_lidar',
        name       = 'delta_lidar',
        namespace  = 'delta_lidar',
        parameters = [PathJoinSubstitution([FindPackageShare('ros2_delta_lidar'), 'config', 'delta2g_params.yaml'])],
        output     = 'screen'        
    )

    ld.add_action(node_lidar)

    return ld
