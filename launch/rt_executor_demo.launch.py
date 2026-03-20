# Copyright 2024 RT Controller Framework Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file for the standalone RT Executor demo node.

Runs the RT executor node which demonstrates a 1kHz deterministic loop
with jitter measurement and real-time metrics publishing.

Usage:
    ros2 launch rt_controller_framework rt_executor_demo.launch.py
    ros2 launch rt_controller_framework rt_executor_demo.launch.py \
        frequency:=500 cycles:=20000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
