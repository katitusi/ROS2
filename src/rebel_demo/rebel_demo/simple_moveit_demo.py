#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions

class RebelSimpleDemo(Node):
    def __init__(self):
        super().__init__('rebel_simple_demo')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.current_joints = {}
        self.joint_names = []
        self.get_logger().info('Waiting for move_action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('move_action server available')

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]
        if not self.joint_names:
            self.joint_names = msg.name

    def run_demo(self):
        # Wait for some joint states
        while not self.current_joints and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Got joint states')

        # Create goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'rebel_6dof'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Create constraints
        constraints = Constraints()
        
        target_joints = self.current_joints.copy()
        
        # Modify joint 2
        if 'joint2' in target_joints:
            target_joints['joint2'] += 0.2
        
        for name, position in target_joints.items():
            # Only add constraints for joints we want to control
            if name in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']: 
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = position
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        # Options
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        
        self.get_logger().info('Sending goal...')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f'Result: {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    node = RebelSimpleDemo()
    node.run_demo()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
