#!/usr/bin/env python3
"""
LLM Sicherheits-Überwachungs-Node
Überwacht die menschliche Entfernung und steuert das Sicherheitsverhalten des Roboters.
Integriert optional externes LLM über HTTP.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
import os
import json


class LLMSafetySupervisor(Node):
    """
    Sicherheitsüberwachung mit schwellenwertbasierter Logik und optionaler LLM-Integration.
    Abonniert /human_distance und ruft den /rebel_safety_demo/set_mode Service auf.
    """

    # Zustandsmaschinen-Zustände
    STATE_IDLE = 'IDLE'
    STATE_HUMAN_CLOSE = 'HUMAN_CLOSE'
    STATE_SAFE_RETRACTED = 'SAFE_RETRACTED'

    # Sicherheitsschwellenwerte (Meter)
    WARN_DISTANCE = 1.0
    DANGER_DISTANCE = 0.6

    def __init__(self):
        super().__init__('llm_safety_supervisor')
        
        # Aktueller Zustand
        self.current_state = self.STATE_IDLE
        self.last_distance = None
        
        # LLM-Integration
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
        
        # Abonnent
        self.subscription = self.create_subscription(
            Float32,
            '/human_distance',
            self.distance_callback,
            10
        )
        
        # Service-Client
        self.set_mode_client = self.create_client(SetBool, '/rebel_safety_demo/set_mode')
        
        # Auf Service warten
        self.get_logger().info('Warte auf /rebel_safety_demo/set_mode Service...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('LLM Safety Supervisor started')
        self.get_logger().info(f'Thresholds: WARN={self.WARN_DISTANCE}m, DANGER={self.DANGER_DISTANCE}m')
        self.get_logger().info('Ready to monitor human distance')

    def distance_callback(self, msg):
        """
        Eingehende Entfernungsnachrichten verarbeiten und Sicherheitsaktionen auslösen.
        
        Args:
            msg (Float32): Entfernung in Metern
        """
        distance = msg.data
        self.last_distance = distance
        
        # Gewünschten Befehl bestimmen
        if self.use_llm:
            command = self.get_llm_command(distance)
        else:
            command = self.get_threshold_command(distance)
        
        # Befehl ausführen
        self.execute_command(command, distance)

    def get_threshold_command(self, distance):
        """
        Schwellenwertbasierte Entscheidungslogik.
        
        Args:
            distance (float): Aktuelle Entfernung in Metern
        
        Returns:
            str: 'HOME' oder 'SAFE_RETRACT'
        """
        if distance <= self.DANGER_DISTANCE:
            return 'SAFE_RETRACT'
        elif distance > self.WARN_DISTANCE:
            return 'HOME'
        else:
            # In Warnzone, aktuellen Zustand beibehalten
            if self.current_state == self.STATE_SAFE_RETRACTED:
                return 'SAFE_RETRACT'
            else:
                return 'HOME'

    def get_llm_command(self, distance):
        """
        Externes LLM für Entscheidung abfragen.
        Fällt bei Fehler auf Schwellenwert-Logik zurück.
        
        Args:
            distance (float): Aktuelle Entfernung in Metern
        
        Returns:
            str: 'HOME' oder 'SAFE_RETRACT'
        """
        try:
            # Payload erstellen
            payload = {
                'distance_m': float(distance),
                'state': self.current_state
            }
            
            # HTTP POST senden
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
        
        # Zurückfallen auf Schwellenwert-Logik
        self.get_logger().info('Falling back to threshold logic')
        return self.get_threshold_command(distance)

    def execute_command(self, command, distance):
        """
        Beschlossenen Befehl durch Aufruf des set_mode Services ausführen.
        
        Args:
            command (str): 'HOME' oder 'SAFE_RETRACT'
            distance (float): Aktuelle Entfernung für Protokollierung
        """
        # Befehl auf Service-Anfrage abbilden
        request = SetBool.Request()
        
        if command == 'SAFE_RETRACT':
            request.data = True
            new_state = self.STATE_SAFE_RETRACTED
        else:  # HOME
            request.data = False
            new_state = self.STATE_IDLE
        
        # Service nur bei erforderlicher Zustandsänderung aufrufen
        if new_state != self.current_state:
            self.get_logger().info(
                f'Distance: {distance:.2f}m | State: {self.current_state} -> {new_state} | '
                f'Command: {command} | LLM: {self.use_llm}'
            )
            
            # Service asynchron aufrufen
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(lambda f: self.service_response_callback(f, new_state))
        else:
            # Zustand unverändert, Entfernung gelegentlich protokollieren
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
            
            if self._log_counter % 50 == 0:  # Alle ~5 Sekunden bei 10Hz protokollieren
                self.get_logger().info(
                    f'Distance: {distance:.2f}m | State: {self.current_state} (stable)'
                )

    def service_response_callback(self, future, new_state):
        """
        Service-Antwort verarbeiten.
        
        Args:
            future: Service-Aufruf Future
            new_state (str): Zielzustand
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
