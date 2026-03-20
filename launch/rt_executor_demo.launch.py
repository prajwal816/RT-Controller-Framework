"""
Launch file for the standalone RT Executor demo node.

Runs the RT executor node which demonstrates a 1kHz deterministic loop
with jitter measurement and real-time metrics publishing.

Usage:
    ros2 launch rt_controller_framework rt_executor_demo.launch.py
    ros2 launch rt_controller_framework rt_executor_demo.launch.py frequency:=500 cycles:=20000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Launch Arguments ────────────────────────────────────────────────
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='1000',
        description='Control loop frequency in Hz'
    )

    cycles_arg = DeclareLaunchArgument(
        'cycles',
        default_value='10000',
        description='Number of cycles to run (0 = infinite)'
    )

    # ─── RT Executor Node ────────────────────────────────────────────────
    rt_executor_node = Node(
        package='rt_controller_framework',
        executable='rt_executor_node',
        name='rt_executor_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        frequency_arg,
        cycles_arg,
        rt_executor_node,
    ])
