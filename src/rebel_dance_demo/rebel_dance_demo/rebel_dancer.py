#!/usr/bin/env python3
"""
ReBeL Tänzer Node
30-sekündige Tanzchoreographie für den igus ReBeL Roboter.
Inspiriert von Robotertänzen von Boston Dynamics und industriellen Manipulator-Choreographien.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from pymoveit2 import MoveIt2
import time
from threading import Thread


class RebelDancer(Node):
    """
    Choreographischer Tanz-Controller für den igus ReBeL Roboter.
    Führt eine 30-sekündige Tanzroutine mit verschiedenen Bewegungsmustern aus.
    """

    def __init__(self):
        super().__init__('rebel_dancer')
        
        # Gelenknamen für igus ReBeL 6DOF (passend zum /joint_states Topic)
        joint_names = [
            'joint1',
            'joint2', 
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]
        
        # MoveIt2 Schnittstelle initialisieren
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name='base_link',
            end_effector_name='tcp',
            group_name='rebel_6dof'
        )
        
        # Geschwindigkeits- und Beschleunigungsskalierung konfigurieren (flüssiges Tanzen)
        self.default_velocity = 1.0  # 100% Geschwindigkeit (MAXIMUM - 5x schneller als Original)
        self.default_acceleration = 1.0  # 100% Beschleunigung (MAXIMUM)
        
        # MoveIt2 in separatem Thread starten
        self.executor_thread = Thread(target=self._spin_moveit2, daemon=True)
        self.executor_thread.start()
        
        # Choreographie-Posen definieren (6 Gelenkwerte in Radiant)
        self.poses = {
            'neutral': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'greeting': [0.0, -0.8, 1.2, 0.0, 0.5, 0.0],
            
            # Wellen-Bewegungsposen
            'wave_1': [0.3, -0.3, 0.6, 0.2, 0.4, 0.5],
            'wave_2': [-0.3, -0.5, 0.8, -0.2, 0.3, -0.5],
            'wave_3': [0.0, -0.4, 0.7, 0.0, 0.35, 0.0],
            
            # Drehposen
            'twist_1': [1.0, -0.4, 0.7, 0.0, 0.3, 0.0],
            'twist_2': [1.0, -0.4, 0.7, 1.2, 0.3, 0.0],
            'twist_3': [1.0, -0.4, 0.7, 1.2, 0.3, 1.5],
            'twist_4': [1.0, -0.4, 0.7, 0.0, 0.3, 1.5],
            
            # Finale-Posen (dynamisch)
            'finale_1': [0.5, -0.9, 1.3, 0.8, 0.6, 1.0],
            'finale_2': [-0.5, -0.3, 0.4, -0.8, 0.2, -1.0],
            'finale_3': [0.8, -0.7, 1.0, 0.5, 0.8, 0.5],
            'finale_4': [-0.8, -0.5, 0.8, -0.5, 0.4, -0.5],
            'finale_5': [0.0, -0.6, 0.9, 0.0, 0.5, 0.0],
            
            # Verbeugungspose
            'bow': [0.0, 0.3, -0.3, 0.0, -0.2, 0.0],
        }
        
        self.is_dancing = False
        
        # Service zum Auslösen des Tanzes erstellen
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
        """MoveIt2 in separatem Thread ausführen."""
        # MoveIt2 benötigt kein Spinning, es ist keine Node
        pass

    def start_dance_callback(self, request, response):
        """
        Service-Callback zum Starten der Tanzroutine.
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
        Zu einer spezifischen Gelenkkonfiguration bewegen.
        
        Args:
            joint_values: Liste von 6 Gelenkwinkeln in Radiant
            velocity_scale: Standardgeschwindigkeit überschreiben (0.0-1.0)
            acceleration_scale: Standardbeschleunigung überschreiben (0.0-1.0)
        
        Returns:
            bool: Erfolgsstatus
        """
        # Geschwindigkeit und Beschleunigung setzen, falls angegeben
        if velocity_scale is not None:
            self.moveit2.max_velocity = velocity_scale
        else:
            self.moveit2.max_velocity = self.default_velocity
            
        if acceleration_scale is not None:
            self.moveit2.max_acceleration = acceleration_scale
        else:
            self.moveit2.max_acceleration = self.default_acceleration
        
        # Zur Gelenkkonfiguration bewegen
        self.moveit2.move_to_configuration(joint_positions=joint_values)
        self.moveit2.wait_until_executed()
        return True

    def interpolate_poses(self, start_pose, end_pose, steps):
        """
        Interpolierte Posen zwischen Start und Ende erstellen.
        
        Args:
            start_pose: Start-Gelenkkonfiguration
            end_pose: End-Gelenkkonfiguration
            steps: Anzahl der Zwischenschritte
        
        Returns:
            Liste interpolierter Posen
        """
        interpolated = []
        for i in range(steps + 1):
            t = i / steps
            pose = [start_pose[j] + (end_pose[j] - start_pose[j]) * t for j in range(6)]
            interpolated.append(pose)
        return interpolated

    def perform_dance(self):
        """
        Die komplette 30-sekündige Tanzroutine ausführen.
        """
        self.is_dancing = True
        start_time = time.time()
        
        try:
            # ========== Phase 1: Eröffnung (0-0.4 Sek) ==========
            self.get_logger().info('🎭 Phase 1: Eröffnung - Begrüßung (0-0.4s)')
            self.move_to_pose(self.poses['neutral'], velocity_scale=1.0)
            time.sleep(0.1)
            self.move_to_pose(self.poses['greeting'], velocity_scale=1.0)
            time.sleep(0.06)
            self.log_progress(start_time)
            
            # ========== Phase 2: Wellenbewegung (0.4-1.6 Sek) ==========
            self.get_logger().info('🌊 Phase 2: Wellenbewegung (0.4-1.6s)')
            wave_sequence = ['wave_1', 'wave_2', 'wave_3', 'wave_2', 'wave_1', 'neutral']
            for wave_pose in wave_sequence:
                self.move_to_pose(self.poses[wave_pose], velocity_scale=1.0)
                time.sleep(0.04)
            self.log_progress(start_time)
            
            # ========== Phase 3: Achter-Muster (1.6-2.8 Sek) ==========
            self.get_logger().info('∞ Phase 3: Achter-Muster (1.6-2.8s)')
            # Sanfte Übergänge für Achter-Muster erstellen
            figure8_poses = [
                [0.5, -0.6, 0.9, 0.3, 0.5, 0.8],
                [0.0, -0.7, 1.0, 0.0, 0.4, 1.2],
                [-0.5, -0.6, 0.9, -0.3, 0.5, 0.8],
                [0.0, -0.5, 0.8, 0.0, 0.6, 0.4],
            ]
            for _ in range(2):  # 2 Zyklen
                for pose in figure8_poses:
                    self.move_to_pose(pose, velocity_scale=1.0)
                    time.sleep(0.03)
            self.log_progress(start_time)
            
            # ========== Phase 4: Roboter-Drehung (2.8-4 Sek) ==========
            self.get_logger().info('🌀 Phase 4: Roboter-Drehung (2.8-4s)')
            twist_sequence = ['twist_1', 'twist_2', 'twist_3', 'twist_4', 'twist_1', 'neutral']
            for twist_pose in twist_sequence:
                self.move_to_pose(self.poses[twist_pose], velocity_scale=1.0)
                time.sleep(0.06)
            self.log_progress(start_time)
            
            # ========== Phase 5: Großes Finale (4-5.6 Sek) ==========
            self.get_logger().info('💥 Phase 5: Großes Finale - ULTRA SCHNELLE Kombination (4-5.6s)')
            finale_sequence = ['finale_1', 'finale_2', 'finale_3', 'finale_4', 'finale_5']
            for finale_pose in finale_sequence:
                self.move_to_pose(self.poses[finale_pose], velocity_scale=1.0)  # Max speed!
                time.sleep(0.04)
            
            # Zusätzliche Verzierung
            self.move_to_pose(self.poses['greeting'], velocity_scale=1.0)
            time.sleep(0.06)
            self.move_to_pose(self.poses['finale_3'], velocity_scale=1.0)
            self.log_progress(start_time)
            
            # ========== Phase 6: Verbeugung (5.6-6 Sek) ==========
            self.get_logger().info('🙇 Phase 6: Verbeugung - Abschließende Begrüßung (5.6-6s)')
            self.move_to_pose(self.poses['neutral'], velocity_scale=1.0)
            time.sleep(0.06)
            self.move_to_pose(self.poses['bow'], velocity_scale=1.0)
            time.sleep(0.1)
            self.move_to_pose(self.poses['neutral'], velocity_scale=1.0)
            
            elapsed = time.time() - start_time
            self.get_logger().info(f'✨ Dance completed! Total time: {elapsed:.1f} seconds')
            
        finally:
            self.is_dancing = False

    def log_progress(self, start_time):
        """Aktuellen Fortschritts-Zeitstempel protokollieren."""
        elapsed = time.time() - start_time
        self.get_logger().info(f'⏱️  Progress: {elapsed:.1f}s elapsed')

    def shutdown(self):
        """Sauberes Herunterfahren von MoveIt."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RebelDancer()
    
    # Tanz automatisch nach 2 Sekunden starten
    node.get_logger().info('⏳ Tanz startet automatisch in 2 Sekunden...')
    time.sleep(2)
    
    node.get_logger().info('🎵 Let\'s dance! 🎵')
    node.perform_dance()
    
    # Node für Service-Aufrufe am Leben halten
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
