#!/usr/bin/env python3
"""
Dance Demo Launch File - Simulation
Launches the rebel_dancer node for simulation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the dance demo for simulation.
    
    Assumes that the robot simulation is already running in another terminal:
    ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
    """
    
    return LaunchDescription([
        # ReBeL dancer node
        Node(
            package='rebel_dance_demo',
            executable='rebel_dancer',
            name='rebel_dancer',
            output='screen',
            parameters=[],
        ),
    ])
