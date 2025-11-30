from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rebel_demo',
            executable='simple_moveit_demo',
            name='rebel_simple_demo',
            output='screen',
            parameters=[{'use_sim_time': True}] # Optional, good for simulation
        ),
    ])
