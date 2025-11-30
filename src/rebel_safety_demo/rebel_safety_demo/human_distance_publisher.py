#!/usr/bin/env python3
"""
Human Distance Publisher Node
Simulates a human approaching the robot by publishing distance data.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class HumanDistancePublisher(Node):
    """
    Publishes simulated human distance to /human_distance topic.
    Distance decreases from 2.0m to 0.2m over ~20 seconds, then resets.
    """

    def __init__(self):
        super().__init__('human_distance_publisher')
        
        # Publisher
        self.publisher_ = self.create_publisher(Float32, '/human_distance', 10)
        
        # Simulation parameters
        self.max_distance = 2.0  # meters
        self.min_distance = 0.2  # meters
        self.cycle_duration = 20.0  # seconds
        self.publish_rate = 10.0  # Hz
        
        # State
        self.elapsed_time = 0.0
        self.timer_period = 1.0 / self.publish_rate
        
        # Timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('Human Distance Publisher started')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')
        self.get_logger().info(f'Distance range: {self.min_distance}m - {self.max_distance}m')
        self.get_logger().info(f'Cycle duration: {self.cycle_duration}s')

    def timer_callback(self):
        """
        Calculate and publish current distance based on elapsed time.
        """
        # Calculate current distance (linear decrease)
        progress = (self.elapsed_time % self.cycle_duration) / self.cycle_duration
        current_distance = self.max_distance - (self.max_distance - self.min_distance) * progress
        
        # Publish
        msg = Float32()
        msg.data = current_distance
        self.publisher_.publish(msg)
        
        # Log every second (approximately)
        if int(self.elapsed_time * self.publish_rate) % int(self.publish_rate) == 0:
            self.get_logger().info(f'Human distance: {current_distance:.2f} m')
        
        # Update elapsed time
        self.elapsed_time += self.timer_period


def main(args=None):
    rclpy.init(args=args)
    node = HumanDistancePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
