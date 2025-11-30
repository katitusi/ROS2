#!/usr/bin/env python3
"""
ReBeL Mover Node
Provides a service to move the igus ReBeL robot between predefined poses.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import moveit_commander
import sys


class RebelMover(Node):
    """
    MoveIt-based controller for igus ReBeL robot.
    Provides /rebel_safety_demo/set_mode service to switch between poses.
    """

    def __init__(self):
        super().__init__('rebel_mover')
        
        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Create MoveGroup for rebel_6dof
        self.move_group = moveit_commander.MoveGroupCommander('rebel_6dof')
        
        # Configure velocity and acceleration scaling
        self.move_group.set_max_velocity_scaling_factor(0.1)  # 10% speed
        self.move_group.set_max_acceleration_scaling_factor(0.1)  # 10% accel
        
        # Define named poses (6 joint values for igus ReBeL 6DOF)
        # Joint order: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        self.home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.safe_retract_pose = [0.0, -0.5, 0.8, 0.0, 0.3, 0.0]
        
        self.current_mode = 'HOME'
        
        # Create service
        self.service = self.create_service(
            SetBool,
            '/rebel_safety_demo/set_mode',
            self.set_mode_callback
        )
        
        self.get_logger().info('ReBeL Mover Node started')
        self.get_logger().info(f'MoveIt group: {self.move_group.get_name()}')
        self.get_logger().info(f'Planning frame: {self.move_group.get_planning_frame()}')
        self.get_logger().info(f'End effector link: {self.move_group.get_end_effector_link()}')
        self.get_logger().info(f'Velocity scaling: 10%, Acceleration scaling: 10%')
        self.get_logger().info('Service /rebel_safety_demo/set_mode ready')
        self.get_logger().info('  - data=True  -> SAFE_RETRACT pose')
        self.get_logger().info('  - data=False -> HOME pose')

    def set_mode_callback(self, request, response):
        """
        Service callback to set robot mode.
        
        Args:
            request.data (bool): True = SAFE_RETRACT, False = HOME
        
        Returns:
            response.success (bool): True if motion executed successfully
            response.message (str): Status message
        """
        try:
            if request.data:
                # Move to SAFE_RETRACT pose
                target_pose = self.safe_retract_pose
                mode_name = 'SAFE_RETRACT'
            else:
                # Move to HOME pose
                target_pose = self.home_pose
                mode_name = 'HOME'
            
            self.get_logger().info(f'Moving to {mode_name} pose...')
            
            # Set joint target
            self.move_group.set_joint_value_target(target_pose)
            
            # Plan and execute
            success = self.move_group.go(wait=True)
            
            # Stop robot after motion
            self.move_group.stop()
            
            if success:
                self.current_mode = mode_name
                response.success = True
                response.message = f'Successfully moved to {mode_name} pose'
                self.get_logger().info(f'✓ Reached {mode_name} pose')
            else:
                response.success = False
                response.message = f'Failed to reach {mode_name} pose'
                self.get_logger().error(f'✗ Motion planning or execution failed for {mode_name}')
            
        except Exception as e:
            response.success = False
            response.message = f'Error during motion: {str(e)}'
            self.get_logger().error(f'Exception in set_mode: {str(e)}')
        
        return response

    def shutdown(self):
        """Clean shutdown of MoveIt commander."""
        moveit_commander.roscpp_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RebelMover()
    
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
