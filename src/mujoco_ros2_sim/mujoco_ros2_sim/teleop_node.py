#!/usr/bin/env python3
"""
Interactive Teleoperation Node for Unitree G1 Humanoid in ROS 2.

Allows commanding predefined gestures (wave, squat, stand, bow) or custom
joint angles to /joint_commands over DDS.
"""

import sys
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from mujoco_ros2_sim.dds_config import get_command_qos

ACTUATOR_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'
]


class UnitreeG1TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        cmd_qos = get_command_qos()
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', cmd_qos)

        # Postura base por defecto
        self.current_ctrl = np.zeros(len(ACTUATOR_NAMES), dtype=float)
        self.current_ctrl[15] = 0.2
        self.current_ctrl[16] = 0.2
        self.current_ctrl[18] = 1.28
        self.current_ctrl[22] = 0.2
        self.current_ctrl[23] = -0.2
        self.current_ctrl[25] = 1.28

        self.get_logger().info("Nodo Teleop para Unitree G1 listo.")

    def publish_ctrl(self):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.current_ctrl]
        self.cmd_pub.publish(msg)

    def set_preset(self, preset_name: str):
        preset = preset_name.lower()
        if preset == 'stand':
            self.current_ctrl[:] = 0.0
            self.current_ctrl[15] = 0.2
            self.current_ctrl[16] = 0.2
            self.current_ctrl[18] = 1.28
            self.current_ctrl[22] = 0.2
            self.current_ctrl[23] = -0.2
            self.current_ctrl[25] = 1.28
            print(">> Preset aplicado: STAND (Postura de pie)")

        elif preset == 'wave':
            self.set_preset('stand')
            self.current_ctrl[22] = -1.4   # Hombro pitch arriba
            self.current_ctrl[23] = -1.3   # Hombro roll lateral
            self.current_ctrl[24] = 0.0    # Hombro yaw
            self.current_ctrl[25] = 0.6    # Codo flexionado (apuntando arriba)
            self.current_ctrl[26] = -0.8   # Palma al frente
            self.current_ctrl[27] = 0.3    # Muñeca inclinada para saludar
            print(">> Preset aplicado: WAVE (Postura de saludo con mano alzada)")

        elif preset == 'squat':
            self.set_preset('stand')
            flex = 0.25
            self.current_ctrl[0] -= flex
            self.current_ctrl[3] += 2.0 * flex
            self.current_ctrl[4] -= flex
            self.current_ctrl[6] -= flex
            self.current_ctrl[9] += 2.0 * flex
            self.current_ctrl[10] -= flex
            print(">> Preset aplicado: SQUAT (Sentadilla)")

        elif preset == 'bow':
            self.set_preset('stand')
            self.current_ctrl[14] = 0.35  # Inclinacion cintura
            print(">> Preset aplicado: BOW (Reverencia)")

        elif preset == 'tpose':
            self.current_ctrl[:] = 0.0
            self.current_ctrl[16] = 1.57   # Brazo izq horizontal
            self.current_ctrl[23] = -1.57  # Brazo der horizontal
            print(">> Preset aplicado: T-POSE")
        else:
            print(f"Preset desconocido '{preset}'. Disponibles: stand, wave, squat, bow, tpose")
            return

        self.publish_ctrl()


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeG1TeleopNode()

    # Si se pasan argumentos por CLI: ros2 run mujoco_ros2_sim teleop_node -- wave
    cli_args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    if len(cli_args) >= 1:
        cmd = cli_args[0].lower()
        node.set_preset(cmd)
        timer = node.create_timer(0.05, node.publish_ctrl)
        rclpy.spin_once(node, timeout_sec=2.0)
        return

    print("\n========================================================")
    print("   Consola Interactiva de Teleoperacion - Unitree G1   ")
    print("========================================================")
    print("Comandos rapidos disponibles:")
    print("  stand  -> Postura de pie estable")
    print("  wave   -> Saludo con brazo derecho")
    print("  squat  -> Postura de sentadilla")
    print("  bow    -> Reverencia con la cintura")
    print("  tpose  -> Postura en T")
    print("  list   -> Listar las 29 articulaciones")
    print("  set <indice> <radianes> -> Comandar una articulacion especifica")
    print("  q      -> Salir\n")

    timer = node.create_timer(0.05, node.publish_ctrl)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            line = input("Unitree-G1 > ").strip()
            if not line:
                continue
            if line.lower() == 'q':
                break
            elif line.lower() == 'list':
                print("\nArticulaciones disponibles:")
                for i, name in enumerate(ACTUATOR_NAMES):
                    print(f" [{i:2d}] {name}: {node.current_ctrl[i]:.2f} rad")
                print()
            elif line.lower().startswith('set '):
                parts = line.split()
                if len(parts) == 3:
                    try:
                        idx = int(parts[1])
                        val = float(parts[2])
                        if 0 <= idx < len(ACTUATOR_NAMES):
                            node.current_ctrl[idx] = val
                            node.publish_ctrl()
                            print(f">> Articulacion [{idx}] {ACTUATOR_NAMES[idx]} = {val:.2f} rad")
                        else:
                            print(f"Indice fuera de rango (0 a {len(ACTUATOR_NAMES)-1})")
                    except ValueError:
                        print("Uso: set <indice_entero> <valor_float>")
            else:
                node.set_preset(line)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
