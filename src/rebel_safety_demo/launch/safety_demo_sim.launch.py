#!/usr/bin/env python3
"""
Safety Demo Launch File - Simulation
Launches all nodes for the rebel_safety_demo in simulation mode.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the safety demo nodes for simulation.
    
    Assumes that the robot simulation is already running in another terminal:
    ros2 launch irc_ros_moveit_config rebel.launch.py hardware_protocol:=mock_hardware
    """
    
    return LaunchDescription([
        # Human distance publisher (simulated sensor)
        Node(
            package='rebel_safety_demo',
            executable='rebel_human_distance_publisher',
            name='human_distance_publisher',
            output='screen',
            parameters=[],
        ),
        
        # ReBeL mover (MoveIt controller)
        Node(
            package='rebel_safety_demo',
            executable='rebel_mover',
            name='rebel_mover',
            output='screen',
            parameters=[],
        ),
        
        # LLM safety supervisor
        Node(
            package='rebel_safety_demo',
            executable='llm_safety_supervisor',
            name='llm_safety_supervisor',
            output='screen',
            parameters=[],
        ),
    ])
