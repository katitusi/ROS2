#!/usr/bin/env python3
"""
LLM Safety Supervisor Node
Monitors human distance and controls robot safety behavior.
Optionally integrates with external LLM via HTTP.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
import os
import json


class LLMSafetySupervisor(Node):
    """
    Safety supervisor with threshold-based logic and optional LLM integration.
    Subscribes to /human_distance and calls /rebel_safety_demo/set_mode service.
    """

    # State machine states
    STATE_IDLE = 'IDLE'
    STATE_HUMAN_CLOSE = 'HUMAN_CLOSE'
    STATE_SAFE_RETRACTED = 'SAFE_RETRACTED'

    # Safety thresholds (meters)
    WARN_DISTANCE = 1.0
    DANGER_DISTANCE = 0.6

    def __init__(self):
        super().__init__('llm_safety_supervisor')
        
        # Current state
        self.current_state = self.STATE_IDLE
        self.last_distance = None
        
        # LLM integration
        self.llm_endpoint = os.environ.get('LLM_ENDPOINT', None)
        self.use_llm = self.llm_endpoint is not None
        
        if self.use_llm:
            try:
                import requests
                self.requests = requests
                self.get_logger().info(f'LLM integration enabled: {self.llm_endpoint}')
            except ImportError:
                self.get_logger().warn('requests library not found, LLM disabled')
                self.get_logger().warn('Install with: pip3 install requests')
                self.use_llm = False
        else:
            self.get_logger().info('LLM integration disabled (LLM_ENDPOINT not set)')
            self.get_logger().info('Using threshold-based logic only')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Float32,
            '/human_distance',
            self.distance_callback,
            10
        )
        
        # Service client
        self.set_mode_client = self.create_client(SetBool, '/rebel_safety_demo/set_mode')
        
        # Wait for service
        self.get_logger().info('Waiting for /rebel_safety_demo/set_mode service...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('LLM Safety Supervisor started')
        self.get_logger().info(f'Thresholds: WARN={self.WARN_DISTANCE}m, DANGER={self.DANGER_DISTANCE}m')
        self.get_logger().info('Ready to monitor human distance')

    def distance_callback(self, msg):
        """
        Process incoming distance messages and trigger safety actions.
        
        Args:
            msg (Float32): Distance in meters
        """
        distance = msg.data
        self.last_distance = distance
        
        # Determine desired command
        if self.use_llm:
            command = self.get_llm_command(distance)
        else:
            command = self.get_threshold_command(distance)
        
        # Execute command
        self.execute_command(command, distance)

    def get_threshold_command(self, distance):
        """
        Threshold-based decision logic.
        
        Args:
            distance (float): Current distance in meters
        
        Returns:
            str: 'HOME' or 'SAFE_RETRACT'
        """
        if distance <= self.DANGER_DISTANCE:
            return 'SAFE_RETRACT'
        elif distance > self.WARN_DISTANCE:
            return 'HOME'
        else:
            # In warning zone, maintain current state
            if self.current_state == self.STATE_SAFE_RETRACTED:
                return 'SAFE_RETRACT'
            else:
                return 'HOME'

    def get_llm_command(self, distance):
        """
        Query external LLM for decision.
        Falls back to threshold logic on error.
        
        Args:
            distance (float): Current distance in meters
        
        Returns:
            str: 'HOME' or 'SAFE_RETRACT'
        """
        try:
            # Build payload
            payload = {
                'distance_m': float(distance),
                'state': self.current_state
            }
            
            # Send HTTP POST
            response = self.requests.post(
                self.llm_endpoint,
                json=payload,
                timeout=2.0
            )
            
            if response.status_code == 200:
                data = response.json()
                command = data.get('command', None)
                
                if command in ['HOME', 'SAFE_RETRACT']:
                    self.get_logger().info(f'LLM decision: {command}')
                    return command
                else:
                    self.get_logger().warn(f'Invalid LLM response: {command}')
            else:
                self.get_logger().warn(f'LLM HTTP error: {response.status_code}')
        
        except Exception as e:
            self.get_logger().warn(f'LLM error: {str(e)}')
        
        # Fallback to threshold logic
        self.get_logger().info('Falling back to threshold logic')
        return self.get_threshold_command(distance)

    def execute_command(self, command, distance):
        """
        Execute the decided command by calling the set_mode service.
        
        Args:
            command (str): 'HOME' or 'SAFE_RETRACT'
            distance (float): Current distance for logging
        """
        # Map command to service request
        request = SetBool.Request()
        
        if command == 'SAFE_RETRACT':
            request.data = True
            new_state = self.STATE_SAFE_RETRACTED
        else:  # HOME
            request.data = False
            new_state = self.STATE_IDLE
        
        # Only call service if state change is needed
        if new_state != self.current_state:
            self.get_logger().info(
                f'Distance: {distance:.2f}m | State: {self.current_state} -> {new_state} | '
                f'Command: {command} | LLM: {self.use_llm}'
            )
            
            # Call service asynchronously
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(lambda f: self.service_response_callback(f, new_state))
        else:
            # State unchanged, just log distance occasionally
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
            
            if self._log_counter % 50 == 0:  # Log every ~5 seconds at 10Hz
                self.get_logger().info(
                    f'Distance: {distance:.2f}m | State: {self.current_state} (stable)'
                )

    def service_response_callback(self, future, new_state):
        """
        Handle service response.
        
        Args:
            future: Service call future
            new_state (str): Target state
        """
        try:
            response = future.result()
            if response.success:
                self.current_state = new_state
                self.get_logger().info(f'✓ {response.message}')
            else:
                self.get_logger().error(f'✗ {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMSafetySupervisor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
