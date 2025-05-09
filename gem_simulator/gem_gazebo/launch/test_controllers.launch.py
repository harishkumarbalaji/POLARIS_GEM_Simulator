#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Include the gem_vehicle.launch file
    gem_gazebo_path = get_package_share_directory('gem_gazebo')
    gem_vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gem_gazebo_path, 'launch', 'gem_vehicle.launch')
        )
    )
    
    # Start the test_controllers.py script
    test_controllers_node = Node(
        package='gem_gazebo',
        executable='test_controllers.py',
        name='controller_tester',
        output='screen'
    )
    
    return LaunchDescription([
        gem_vehicle_launch,
        test_controllers_node
    ]) 