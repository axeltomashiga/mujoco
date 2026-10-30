#!/usr/bin/env python3
"""
Controller Node for Unitree G1 Humanoid in ROS 2.

Implements humanoid motion generators communicating over DDS:
- Subscribes: /joint_states (sensor_msgs/msg/JointState) [Best Effort QoS]
- Publishes: /joint_commands (std_msgs/msg/Float64MultiArray) [Reliable QoS]

Supported Modes:
  - 'wave': Right arm waving greeting motion beside head while maintaining balance
  - 'squat': Periodic squatting motion lowering and raising COM
  - 'stand': Fixed standing posture
  - 'bow': Respectful greeting bow flexing waist pitch
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from mujoco_ros2_sim.dds_config import get_sensor_state_qos, get_command_qos


class UnitreeG1ControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Parametros de configuracion
        self.declare_parameter('mode', 'wave')  # 'wave', 'squat', 'stand', 'bow'
        self.declare_parameter('control_rate', 100.0)  # Hz

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value

        self.get_logger().info(f"Iniciando controlador para Unitree G1 en modo: '{self.mode}'")

        # DDS QoS matching
        sensor_qos = get_sensor_state_qos()
        cmd_qos = get_command_qos()

        # Suscriptor al estado articular
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            sensor_qos
        )

        # Publicador de comandos a los 29 actuadores
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            cmd_qos
        )

        # Postura base oficial de pie ('stand' keyframe) para los 29 actuadores:
        # 0..5 (pierna izq), 6..11 (pierna der), 12..14 (cintura), 15..21 (brazo izq), 22..28 (brazo der)
        self.base_posture = np.zeros(29, dtype=float)
        # Brazos en posicion de reposo natural
        self.base_posture[15] = 0.2    # left_shoulder_pitch
        self.base_posture[16] = 0.2    # left_shoulder_roll
        self.base_posture[18] = 1.28   # left_elbow
        self.base_posture[22] = 0.2    # right_shoulder_pitch
        self.base_posture[23] = -0.2   # right_shoulder_roll
        self.base_posture[25] = 1.28   # right_elbow

        self.start_time = None
        self.last_log_time = 0.0

        # Timer de control
        dt = 1.0 / self.control_rate
        self.timer = self.create_timer(dt, self.control_loop)

        self.current_q = None
        self.current_names = None

    def state_callback(self, msg: JointState):
        """Almacena el estado mas reciente recibido por DDS."""
        self.current_q = np.array(msg.position, dtype=float)
        self.current_names = msg.name

    def control_loop(self):
        """Genera consignas articulares segun el modo de movimiento."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now

        t = now - self.start_time
        target_ctrl = np.copy(self.base_posture)

        if self.mode == 'wave':
            # Transicion suave al inicio
            alpha = min(1.0, t / 1.5)

            # Saludo fluido con el brazo derecho:
            # 1. Hombro elevado arriba y adelante: pitch = -1.4 rad
            # 2. Hombro abducido lateralmente: roll = -1.3 rad
            # 3. Codo flexionado apuntando el antebrazo hacia arriba: ~0.6 rad
            # 4. Palma orientada al frente: wrist_roll = -0.8 rad
            # 5. Oscilacion de vaiven del antebrazo y la mano:
            wave_freq = 4.5
            forearm_wave = 0.35 * math.sin(wave_freq * t)
            hand_flap = 0.45 * math.sin(wave_freq * t)

            w_pitch = -1.4
            w_roll = -1.3
            w_yaw = 0.0
            w_elbow = 0.6 + forearm_wave
            w_wrist_roll = -0.8
            w_wrist_pitch = hand_flap

            target_ctrl[22] = (1.0 - alpha) * self.base_posture[22] + alpha * w_pitch
            target_ctrl[23] = (1.0 - alpha) * self.base_posture[23] + alpha * w_roll
            target_ctrl[24] = (1.0 - alpha) * self.base_posture[24] + alpha * w_yaw
            target_ctrl[25] = (1.0 - alpha) * self.base_posture[25] + alpha * w_elbow
            target_ctrl[26] = (1.0 - alpha) * self.base_posture[26] + alpha * w_wrist_roll
            target_ctrl[27] = (1.0 - alpha) * self.base_posture[27] + alpha * w_wrist_pitch

        elif self.mode == 'squat':
            # Sentadilla suave manteniendo el centro de gravedad
            flex = 0.15 * (1.0 - math.cos(1.8 * t))  # Rango suave 0 a 0.3 rad
            # Pierna izquierda: cadera, rodilla, tobillo
            target_ctrl[0] -= flex
            target_ctrl[3] += 2.0 * flex
            target_ctrl[4] -= flex
            # Pierna derecha:
            target_ctrl[6] -= flex
            target_ctrl[9] += 2.0 * flex
            target_ctrl[10] -= flex

        elif self.mode == 'bow':
            # Reverencia con la cintura
            bow_pitch = 0.2 * (1.0 - math.cos(1.2 * t))
            target_ctrl[14] = bow_pitch  # waist_pitch

        elif self.mode == 'stand':
            # Postura fija de pie
            pass

        # Publicar consignas al bus DDS
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(v) for v in target_ctrl]
        self.cmd_pub.publish(cmd_msg)

        # Logging periodico
        if t - self.last_log_time > 2.5:
            self.last_log_time = t
            self.get_logger().info(
                f"[t={t:.1f}s | Modo: {self.mode}] Comandando 29 articulaciones | "
                f"Brazo: pitch={target_ctrl[22]:.2f}, roll={target_ctrl[23]:.2f}, elbow={target_ctrl[25]:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeG1ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
