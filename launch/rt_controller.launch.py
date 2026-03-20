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

"""\
Launch file for the RT Controller with mock hardware via ros2_control.

Spawns controller_manager with the mock actuator system and RT controller.
Loads the robot description from the standalone URDF file.

Usage:
    ros2 launch rt_controller_framework rt_controller.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rt_controller_framework')

    # ─── Launch Arguments ────────────────────────────────────────────────
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    # ─── Robot Description from standalone URDF ──────────────────────────
    urdf_path = os.path.join(pkg_share, 'urdf', 'mock_robot.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = {'robot_description': robot_description_content}

    controller_config = os.path.join(
        pkg_share, 'config', 'rt_controller_config.yaml')

    # ─── Controller Manager Node ─────────────────────────────────────────
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # ─── Spawn RT Controller ────────────────────────────────────────────
    rt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rt_controller',
            '--controller-manager',
            '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time,
        controller_manager_node,
        rt_controller_spawner,
    ])
