from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mujoco_ros2_sim'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ]
    
    # Agregar todos los archivos y mallas 3D recursivamente de models/
    for root, dirs, files in os.walk('models'):
        if files:
            target_dir = os.path.join('share', package_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((target_dir, file_list))
            
    return data_files

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Developer',
    maintainer_email='developer@example.com',
    description='Paquete de integracion y simulacion del robot humanoide Unitree G1 en MuJoCo con ROS 2 y DDS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_node = mujoco_ros2_sim.mujoco_node:main',
            'controller_node = mujoco_ros2_sim.controller_node:main',
            'teleop_node = mujoco_ros2_sim.teleop_node:main',
        ],
    },
)
