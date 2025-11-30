#!/usr/bin/env python3
"""
Safety Demo Launch File - Real Robot
Launches all nodes for the rebel_safety_demo with REAL robot.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    """
    Launch the safety demo nodes for REAL robot.
    
    ⚠️  WARNING: This launch file is for the REAL igus ReBeL robot!
    
    Assumes that the robot hardware interface is already running:
    ros2 launch irc_ros_bringup rebel.launch.py hardware_protocol:=cprcanv2
    (or CRI variant)
    
    Safety features:
    - Velocity scaling: 10%
    - Acceleration scaling: 10%
    """
    
    return LaunchDescription([
        # Warning message
        LogInfo(
            msg='╔════════════════════════════════════════════════════════════════╗'
        ),
        LogInfo(
            msg='║                      ⚠️  REAL ROBOT MODE ⚠️                      ║'
        ),
        LogInfo(
            msg='║                                                                ║'
        ),
        LogInfo(
            msg='║  This launch file controls the REAL igus ReBeL robot!         ║'
        ),
        LogInfo(
            msg='║  Safety features enabled:                                     ║'
        ),
        LogInfo(
            msg='║    • Velocity scaling: 10%                                    ║'
        ),
        LogInfo(
            msg='║    • Acceleration scaling: 10%                                ║'
        ),
        LogInfo(
            msg='║                                                                ║'
        ),
        LogInfo(
            msg='║  Ensure the workspace is clear and emergency stop is ready!   ║'
        ),
        LogInfo(
            msg='╚════════════════════════════════════════════════════════════════╝'
        ),
        
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
