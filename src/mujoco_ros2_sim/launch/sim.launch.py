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
        description='Ruta absoluta al modelo XML de MuJoCo (Unitree G1)'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Ejecutar sin ventana grafica'
    )

    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate',
        default_value='100.0',
        description='Frecuencia de sincronizacion con ROS 2 (Hz)'
    )

    mujoco_node = Node(
        package='mujoco_ros2_sim',
        executable='mujoco_node',
        name='mujoco_sim_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'headless': LaunchConfiguration('headless'),
            'sim_rate': LaunchConfiguration('sim_rate'),
        }]
    )

    return LaunchDescription([
        model_arg,
        headless_arg,
        sim_rate_arg,
        mujoco_node,
    ])
