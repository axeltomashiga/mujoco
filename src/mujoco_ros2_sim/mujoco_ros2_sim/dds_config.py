"""
DDS (Data Distribution Service) Quality of Service (QoS) profiles for MuJoCo <-> ROS 2.

In ROS 2, DDS governs the transport layer (reliability, latency, queueing, and discovery).
Selecting appropriate QoS profiles is critical for robotics physics simulation:
- Sensor state feedback requires low latency and must not buffer old packets.
- Actuator control commands must be reliable so safety/control limits are not lost.
"""

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
)
from rclpy.duration import Duration


def get_sensor_state_qos() -> QoSProfile:
    """
    QoS Profile for MuJoCo high-frequency sensor publishing (e.g. /joint_states @ 100+ Hz).
    
    - Reliability: BEST_EFFORT (DDS drops late packets instead of stalling or retransmitting)
    - History: KEEP_LAST (depth = 1, always use newest sample)
    - Durability: VOLATILE (no persistence for late joiners)
    - Deadline: 20 ms (warn if simulation drops below 50 Hz)
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=0.02),
    )


def get_command_qos() -> QoSProfile:
    """
    QoS Profile for actuator torque / position commands (e.g. /joint_commands).
    
    - Reliability: RELIABLE (DDS ensures commands reach the simulation engine)
    - History: KEEP_LAST (depth = 10, accommodates minor controller bursts)
    - Durability: VOLATILE (commands do not persist across restarts)
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def get_parameter_qos() -> QoSProfile:
    """
    QoS Profile for configuration / status topics.
    
    - Reliability: RELIABLE
    - History: KEEP_LAST (depth = 1)
    - Durability: TRANSIENT_LOCAL (latched: late-joining nodes immediately receive last published state)
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
