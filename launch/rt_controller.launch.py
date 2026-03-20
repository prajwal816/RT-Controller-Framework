"""
Launch file for the RT Controller with mock hardware via ros2_control.

Spawns controller_manager with the mock actuator system and RT controller.

Usage:
    ros2 launch rt_controller_framework rt_controller.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rt_controller_framework')

    # ─── Launch Arguments ────────────────────────────────────────────────
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    # ─── Robot Description (URDF with ros2_control tags) ─────────────────
    robot_description_content = """<?xml version="1.0"?>
<robot name="rt_mock_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <ros2_control name="MockActuatorSystem" type="system">
    <hardware>
      <plugin>rt_controller_framework/MockActuatorSystem</plugin>
      <param name="inertia">1.0</param>
      <param name="damping">0.1</param>
      <param name="control_period">0.001</param>
    </hardware>
    <joint name="joint_1">
      <param name="initial_position">0.0</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint_2">
      <param name="initial_position">0.0</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint_3">
      <param name="initial_position">0.0</param>
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

  <!-- Minimal URDF links/joints for controller_manager -->
  <link name="base_link"/>
  <link name="link_1"/>
  <link name="link_2"/>
  <link name="link_3"/>

  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" velocity="6.28" effort="100"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" velocity="6.28" effort="100"/>
  </joint>
  <joint name="joint_3" type="revolute">
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" velocity="3.14" effort="50"/>
  </joint>
</robot>
"""

    robot_description = {'robot_description': robot_description_content}

    controller_config = os.path.join(pkg_share, 'config', 'rt_controller_config.yaml')

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
        arguments=['rt_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time,
        controller_manager_node,
        rt_controller_spawner,
    ])
