#!/usr/bin/env python3
"""
MuJoCo Simulation Node for ROS 2 - Unitree G1 Humanoid Robot.

Bridges the Unitree G1 humanoid physics simulation in MuJoCo with ROS 2 and DDS:
- Publishes:
    /joint_states (sensor_msgs/msg/JointState) [DDS QoS: Best Effort, 100Hz]
    /robot_pose (geometry_msgs/msg/PoseStamped) [DDS QoS: Best Effort]
    /mujoco/status (std_msgs/msg/String) [DDS QoS: Transient Local]
- Subscribes:
    /joint_commands (std_msgs/msg/Float64MultiArray) [DDS QoS: Reliable]
"""

import os
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String
from ament_index_python.packages import get_package_share_directory

import mujoco
try:
    import mujoco.viewer
    VIEWER_AVAILABLE = True
except ImportError:
    VIEWER_AVAILABLE = False

from mujoco_ros2_sim.dds_config import get_sensor_state_qos, get_command_qos, get_parameter_qos


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        # Parametros configurables
        self.declare_parameter('model_path', '')
        self.declare_parameter('sim_rate', 100.0)  # Frecuencia de publicacion ROS 2 (Hz)
        self.declare_parameter('headless', False)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        self.headless = self.get_parameter('headless').get_parameter_value().bool_value

        # Resolver ruta del modelo (prioridad: Unitree G1 scene.xml)
        if not model_path or not os.path.exists(model_path):
            try:
                pkg_share = get_package_share_directory('mujoco_ros2_sim')
                candidate = os.path.join(pkg_share, 'models', 'unitree_g1', 'scene.xml')
                if os.path.exists(candidate):
                    model_path = candidate
            except Exception:
                pass

        if not model_path or not os.path.exists(model_path):
            candidate = os.path.join(os.getcwd(), 'models', 'unitree_g1', 'scene.xml')
            if os.path.exists(candidate):
                model_path = candidate
            else:
                self.get_logger().error(f"No se encontro el archivo XML de Unitree G1 en '{model_path}'!")
                sys.exit(1)

        self.get_logger().info(f"Cargando modelo Unitree G1: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Inicializar postura en keyframe oficial de pie ('stand')
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
            self.default_ctrl = np.copy(self.model.key(0).ctrl)
            self.get_logger().info(f"Postura inicial fijada a keyframe: '{self.model.key(0).name}'")
        else:
            self.default_ctrl = np.zeros(self.model.nu)

        self.num_actuators = self.model.nu
        self.current_cmd = np.copy(self.default_ctrl)
        self.data.ctrl[:] = self.current_cmd

        # Identificar articulaciones controlables (excluyendo la junta libre flotante)
        self.controllable_joints = []
        self.qpos_addrs = []
        self.dof_addrs = []

        for i in range(self.model.njnt):
            jnt = self.model.joint(i)
            # Tipo 0 es junta libre flotante (6 DOF)
            if jnt.type[0] != 0:
                self.controllable_joints.append(jnt.name)
                self.qpos_addrs.append(jnt.qposadr[0])
                self.dof_addrs.append(jnt.dofadr[0])

        self.get_logger().info(
            f"Robot inicializado | Actuadores: {self.num_actuators} | Articulaciones 1-DOF: {len(self.controllable_joints)}"
        )

        self.last_cmd_time = time.time()
        self.cmd_timeout = 1.0  # Si no hay comandos por 1s, mantener postura segura

        # --- Publicadores DDS ---
        sensor_qos = get_sensor_state_qos()
        param_qos = get_parameter_qos()

        # 1. Estado articular (DDS Best Effort para streaming rapido @ 100 Hz)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            sensor_qos
        )

        # 2. Pose de la base flotante del humanoide en el espacio
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            sensor_qos
        )

        # 3. Estado general del simulador
        self.status_pub = self.create_publisher(
            String,
            '/mujoco/status',
            param_qos
        )

        # --- Suscriptores DDS ---
        cmd_qos = get_command_qos()
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            cmd_qos
        )

        # Iniciar visor pasivo
        self.viewer = None
        if not self.headless and VIEWER_AVAILABLE:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer.cam.distance = 2.2
                self.viewer.cam.elevation = -12
                self.viewer.cam.azimuth = 135
                self.viewer.cam.lookat[:] = [0.0, 0.0, 0.75]
                self.get_logger().info("Visor 3D de MuJoCo iniciado correctamente.")
            except Exception as e:
                self.get_logger().warn(f"Visor grafico no disponible ({e}). Modo headless activado.")
                self.viewer = None
        else:
            self.get_logger().info("Modo headless activo (sin ventana grafica).")

        # Temporizador de ejecucion
        dt = 1.0 / self.sim_rate
        self.timer = self.create_timer(dt, self.simulation_step)

        # Mensaje de estado inicial
        status_msg = String()
        status_msg.data = f"Unitree G1 Sim Active | Nu={self.num_actuators} | Rate={self.sim_rate}Hz"
        self.status_pub.publish(status_msg)

    def command_callback(self, msg: Float64MultiArray):
        """Callback al recibir consignas articulares por DDS."""
        if len(msg.data) >= self.num_actuators:
            self.current_cmd = np.array(msg.data[:self.num_actuators], dtype=float)
            self.last_cmd_time = time.time()
        else:
            self.get_logger().warn_throttle(
                2.0,
                f"Comando recibido con tamano incorrecto ({len(msg.data)} vs {self.num_actuators})"
            )

    def simulation_step(self):
        """Paso de fisica en MuJoCo y sincronizacion DDS."""
        # Watchdog: si no hay comandos recientes, mantener postura por defecto
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.data.ctrl[:] = self.default_ctrl
        else:
            self.data.ctrl[:] = self.current_cmd

        # Integrar fisica MuJoCo
        mujoco_dt = self.model.opt.timestep
        ros_dt = 1.0 / self.sim_rate
        substeps = max(1, int(round(ros_dt / mujoco_dt)))

        for _ in range(substeps):
            mujoco.mj_step(self.model, self.data)

        # Actualizar visor si esta activo
        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.get_logger().info("Visor cerrado por el usuario.")
                self.destroy_node()
                rclpy.shutdown()
                return

        now_msg = self.get_clock().now().to_msg()

        # Publicar JointState
        js_msg = JointState()
        js_msg.header.stamp = now_msg
        js_msg.header.frame_id = 'pelvis'
        js_msg.name = self.controllable_joints
        js_msg.position = [float(self.data.qpos[addr]) for addr in self.qpos_addrs]
        js_msg.velocity = [float(self.data.qvel[addr]) for addr in self.dof_addrs]
        js_msg.effort = [float(self.data.ctrl[i]) for i in range(self.num_actuators)]
        self.joint_state_pub.publish(js_msg)

        # Publicar Pose de la base flotante (Pelvis del Humanoide)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(self.data.qpos[0])
        pose_msg.pose.position.y = float(self.data.qpos[1])
        pose_msg.pose.position.z = float(self.data.qpos[2])
        pose_msg.pose.orientation.w = float(self.data.qpos[3])
        pose_msg.pose.orientation.x = float(self.data.qpos[4])
        pose_msg.pose.orientation.y = float(self.data.qpos[5])
        pose_msg.pose.orientation.z = float(self.data.qpos[6])
        self.pose_pub.publish(pose_msg)

    def destroy_node(self):
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
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
