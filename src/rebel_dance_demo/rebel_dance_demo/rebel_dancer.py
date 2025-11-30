#!/usr/bin/env python3
"""
ReBeL Dancer Node
30-second dance choreography for igus ReBeL robot.
Inspired by robot dances from Boston Dynamics and industrial manipulator choreographies.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from pymoveit2 import MoveIt2
import time
from threading import Thread


class RebelDancer(Node):
    """
    Choreographic dance controller for igus ReBeL robot.
    Performs a 30-second dance routine with various movement patterns.
    """

    def __init__(self):
        super().__init__('rebel_dancer')
        
        # Joint names for igus ReBeL 6DOF (matching /joint_states topic)
        joint_names = [
            'joint1',
            'joint2', 
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]
        
        # Initialize MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name='base_link',
            end_effector_name='tcp',
            group_name='rebel_6dof'
        )
        
        # Configure velocity and acceleration scaling (smooth dancing)
        self.default_velocity = 0.3  # 30% speed
        self.default_acceleration = 0.3  # 30% accel
        
        # Start MoveIt2 in a separate thread
        self.executor_thread = Thread(target=self._spin_moveit2, daemon=True)
        self.executor_thread.start()
        
        # Define choreography poses (6 joint values in radians)
        self.poses = {
            'neutral': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'greeting': [0.0, -0.8, 1.2, 0.0, 0.5, 0.0],
            
            # Wave motion poses
            'wave_1': [0.3, -0.3, 0.6, 0.2, 0.4, 0.5],
            'wave_2': [-0.3, -0.5, 0.8, -0.2, 0.3, -0.5],
            'wave_3': [0.0, -0.4, 0.7, 0.0, 0.35, 0.0],
            
            # Twist poses
            'twist_1': [1.0, -0.4, 0.7, 0.0, 0.3, 0.0],
            'twist_2': [1.0, -0.4, 0.7, 1.2, 0.3, 0.0],
            'twist_3': [1.0, -0.4, 0.7, 1.2, 0.3, 1.5],
            'twist_4': [1.0, -0.4, 0.7, 0.0, 0.3, 1.5],
            
            # Finale poses (dynamic)
            'finale_1': [0.5, -0.9, 1.3, 0.8, 0.6, 1.0],
            'finale_2': [-0.5, -0.3, 0.4, -0.8, 0.2, -1.0],
            'finale_3': [0.8, -0.7, 1.0, 0.5, 0.8, 0.5],
            'finale_4': [-0.8, -0.5, 0.8, -0.5, 0.4, -0.5],
            'finale_5': [0.0, -0.6, 0.9, 0.0, 0.5, 0.0],
            
            # Bow pose
            'bow': [0.0, 0.3, -0.3, 0.0, -0.2, 0.0],
        }
        
        self.is_dancing = False
        
        # Create service to trigger dance
        self.service = self.create_service(
            Trigger,
            '/rebel_dance_demo/start_dance',
            self.start_dance_callback
        )
        
        self.get_logger().info('🎭 ReBeL Dancer Node started')
        self.get_logger().info(f'MoveIt group: rebel_6dof')
        self.get_logger().info(f'Default velocity: {self.default_velocity*100}%, acceleration: {self.default_acceleration*100}%')
        self.get_logger().info('Service /rebel_dance_demo/start_dance ready')
        self.get_logger().info('Call the service to start the 30-second dance! 🕺')

    def _spin_moveit2(self):
        """Spin MoveIt2 in a separate thread."""
        # MoveIt2 doesn't need spinning, it's not a node
        pass

    def start_dance_callback(self, request, response):
        """
        Service callback to start the dance routine.
        """
        if self.is_dancing:
            response.success = False
            response.message = 'Dance already in progress! 💃'
            return response
        
        self.get_logger().info('🎬 Starting dance performance...')
        
        try:
            self.perform_dance()
            response.success = True
            response.message = 'Dance completed successfully! 🎉'
        except Exception as e:
            response.success = False
            response.message = f'Dance failed: {str(e)}'
            self.get_logger().error(f'Dance error: {str(e)}')
        
        return response

    def move_to_pose(self, joint_values, velocity_scale=None, acceleration_scale=None):
        """
        Move to a specific joint configuration.
        
        Args:
            joint_values: List of 6 joint angles in radians
            velocity_scale: Override default velocity (0.0-1.0)
            acceleration_scale: Override default acceleration (0.0-1.0)
        
        Returns:
            bool: Success status
        """
        # Set velocity and acceleration if provided
        if velocity_scale is not None:
            self.moveit2.max_velocity = velocity_scale
        else:
            self.moveit2.max_velocity = self.default_velocity
            
        if acceleration_scale is not None:
            self.moveit2.max_acceleration = acceleration_scale
        else:
            self.moveit2.max_acceleration = self.default_acceleration
        
        # Move to joint configuration
        self.moveit2.move_to_configuration(joint_positions=joint_values)
        self.moveit2.wait_until_executed()
        return True

    def interpolate_poses(self, start_pose, end_pose, steps):
        """
        Create interpolated poses between start and end.
        
        Args:
            start_pose: Starting joint configuration
            end_pose: Ending joint configuration
            steps: Number of intermediate steps
        
        Returns:
            List of interpolated poses
        """
        interpolated = []
        for i in range(steps + 1):
            t = i / steps
            pose = [start_pose[j] + (end_pose[j] - start_pose[j]) * t for j in range(6)]
            interpolated.append(pose)
        return interpolated

    def perform_dance(self):
        """
        Execute the complete 30-second dance routine.
        """
        self.is_dancing = True
        start_time = time.time()
        
        try:
            # ========== Phase 1: Opening (0-2 sec) ==========
            self.get_logger().info('🎭 Phase 1: Opening - Greeting (0-2s)')
            self.move_to_pose(self.poses['neutral'], velocity_scale=0.2)
            time.sleep(0.5)
            self.move_to_pose(self.poses['greeting'], velocity_scale=0.25)
            time.sleep(0.3)
            self.log_progress(start_time)
            
            # ========== Phase 2: Wave Motion (2-8 sec) ==========
            self.get_logger().info('🌊 Phase 2: Wave Motion (2-8s)')
            wave_sequence = ['wave_1', 'wave_2', 'wave_3', 'wave_2', 'wave_1', 'neutral']
            for wave_pose in wave_sequence:
                self.move_to_pose(self.poses[wave_pose], velocity_scale=0.3)
                time.sleep(0.2)
            self.log_progress(start_time)
            
            # ========== Phase 3: Figure-8 Pattern (8-14 sec) ==========
            self.get_logger().info('∞ Phase 3: Figure-8 Pattern (8-14s)')
            # Create smooth transitions for figure-8
            figure8_poses = [
                [0.5, -0.6, 0.9, 0.3, 0.5, 0.8],
                [0.0, -0.7, 1.0, 0.0, 0.4, 1.2],
                [-0.5, -0.6, 0.9, -0.3, 0.5, 0.8],
                [0.0, -0.5, 0.8, 0.0, 0.6, 0.4],
            ]
            for _ in range(2):  # 2 cycles
                for pose in figure8_poses:
                    self.move_to_pose(pose, velocity_scale=0.35)
                    time.sleep(0.15)
            self.log_progress(start_time)
            
            # ========== Phase 4: Robot Twist (14-20 sec) ==========
            self.get_logger().info('🌀 Phase 4: Robot Twist (14-20s)')
            twist_sequence = ['twist_1', 'twist_2', 'twist_3', 'twist_4', 'twist_1', 'neutral']
            for twist_pose in twist_sequence:
                self.move_to_pose(self.poses[twist_pose], velocity_scale=0.3)
                time.sleep(0.3)
            self.log_progress(start_time)
            
            # ========== Phase 5: Grand Finale (20-28 sec) ==========
            self.get_logger().info('💥 Phase 5: Grand Finale - Fast Combo (20-28s)')
            finale_sequence = ['finale_1', 'finale_2', 'finale_3', 'finale_4', 'finale_5']
            for finale_pose in finale_sequence:
                self.move_to_pose(self.poses[finale_pose], velocity_scale=0.5)  # Faster!
                time.sleep(0.2)
            
            # Additional flourish
            self.move_to_pose(self.poses['greeting'], velocity_scale=0.6)
            time.sleep(0.3)
            self.move_to_pose(self.poses['finale_3'], velocity_scale=0.6)
            self.log_progress(start_time)
            
            # ========== Phase 6: Bow (28-30 sec) ==========
            self.get_logger().info('🙇 Phase 6: Bow - Final Greeting (28-30s)')
            self.move_to_pose(self.poses['neutral'], velocity_scale=0.25)
            time.sleep(0.3)
            self.move_to_pose(self.poses['bow'], velocity_scale=0.2)
            time.sleep(0.5)
            self.move_to_pose(self.poses['neutral'], velocity_scale=0.2)
            
            elapsed = time.time() - start_time
            self.get_logger().info(f'✨ Dance completed! Total time: {elapsed:.1f} seconds')
            
        finally:
            self.is_dancing = False

    def log_progress(self, start_time):
        """Log current progress timestamp."""
        elapsed = time.time() - start_time
        self.get_logger().info(f'⏱️  Progress: {elapsed:.1f}s elapsed')

    def shutdown(self):
        """Clean shutdown of MoveIt."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RebelDancer()
    
    # Auto-start dance after 2 seconds
    node.get_logger().info('⏳ Dance will start automatically in 2 seconds...')
    time.sleep(2)
    
    node.get_logger().info('🎵 Let\'s dance! 🎵')
    node.perform_dance()
    
    # Keep node alive for service calls
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
