#!/usr/bin/env python3
"""
Dance Demo Launch File - Real Robot
Launches the rebel_dancer node for REAL robot with safety settings.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    """
    Launch the dance demo for REAL robot.
    
    ⚠️  WARNING: This launch file is for the REAL igus ReBeL robot!
    
    Assumes that the robot hardware interface is already running:
    ros2 launch irc_ros_bringup rebel.launch.py hardware_protocol:=cprcanv2
    
    Safety features:
    - Velocity scaling: 20% (reduced from 30% for extra safety)
    - Acceleration scaling: 20%
    """
    
    return LaunchDescription([
        # Warning messages
        LogInfo(
            msg='╔════════════════════════════════════════════════════════════════╗'
        ),
        LogInfo(
            msg='║                 ⚠️  REAL ROBOT DANCE MODE ⚠️                    ║'
        ),
        LogInfo(
            msg='║                                                                ║'
        ),
        LogInfo(
            msg='║  This will make the REAL igus ReBeL robot perform a dance!    ║'
        ),
        LogInfo(
            msg='║  Duration: 30 seconds                                         ║'
        ),
        LogInfo(
            msg='║  Safety features:                                             ║'
        ),
        LogInfo(
            msg='║    • Velocity scaling: 20% (reduced for real robot)           ║'
        ),
        LogInfo(
            msg='║    • Acceleration scaling: 20%                                ║'
        ),
        LogInfo(
            msg='║                                                                ║'
        ),
        LogInfo(
            msg='║  ⚠️  ENSURE:                                                   ║'
        ),
        LogInfo(
            msg='║    - Workspace is completely clear                            ║'
        ),
        LogInfo(
            msg='║    - Emergency stop button is accessible                      ║'
        ),
        LogInfo(
            msg='║    - No obstacles in robot\'s path                             ║'
        ),
        LogInfo(
            msg='║    - Someone is monitoring the robot                          ║'
        ),
        LogInfo(
            msg='╚════════════════════════════════════════════════════════════════╝'
        ),
        
        # ReBeL dancer node with reduced speed for safety
        Node(
            package='rebel_dance_demo',
            executable='rebel_dancer',
            name='rebel_dancer',
            output='screen',
            parameters=[
                {'default_velocity_scale': 0.2},  # 20% instead of 30%
                {'default_acceleration_scale': 0.2},
            ],
        ),
    ])
