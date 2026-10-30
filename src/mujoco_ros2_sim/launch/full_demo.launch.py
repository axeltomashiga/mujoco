import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    try:
        pkg_share = get_package_share_directory('mujoco_ros2_sim')
        default_model_path = os.path.join(pkg_share, 'models', 'unitree_g1', 'scene.xml')
    except Exception:
        default_model_path = os.path.join(os.getcwd(), 'models', 'unitree_g1', 'scene.xml')

    model_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Ruta al modelo XML de Unitree G1'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Ejecutar sin ventana grafica'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='wave',
        description='Modo de movimiento del humanoide: wave, squat, stand, bow'
    )

    mujoco_node = Node(
        package='mujoco_ros2_sim',
        executable='mujoco_node',
        name='mujoco_sim_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'headless': LaunchConfiguration('headless'),
            'sim_rate': 100.0,
        }]
    )

    controller_node = Node(
        package='mujoco_ros2_sim',
        executable='controller_node',
        name='robot_controller_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'control_rate': 100.0,
        }]
    )

    return LaunchDescription([
        model_arg,
        headless_arg,
        mode_arg,
        mujoco_node,
        controller_node,
    ])
