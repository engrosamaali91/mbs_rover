#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2024, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of MYBOTSHOP GmbH nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
    
    # File locations
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('rover_description'), '..'))
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = PathJoinSubstitution(
        [get_package_share_directory('rover_gazebo'), 'worlds', 'fortress-plane.sdf'])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover_description"), "xacro", "robot.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Gazebo Configuration
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.6'
    spawn_yaw_val = '0.00'

    # Nodes
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [
            '-r -s -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 ',
                          'on_exit_shutdown': 'true'}.items()
    )

    gzrosbridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('rover_gazebo'), 'config', 'ros_gz.yaml'),
        }],
        output='screen'
    )

    # Spawn Robot Model
    node_gazebo_robot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_content,
            '-name', 'rover',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val
        ],
        output='screen',
    )

    # Initiate Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
                    {"publish_frequency": 30.0},
                    {'use_sim_time': True},
                    ],
    )
    node_joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    node_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    node_tilt_unit_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jnt_pos_cont"],
    )

    ld = LaunchDescription()

    ld.add_action(set_env_vars_resources)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(gzrosbridge_cmd)
    
    ld.add_action(node_gazebo_robot_spawn)
    ld.add_action(node_robot_state_publisher)
    # ld.add_action(node_joint_state_publisher)
    ld.add_action(node_diff_drive_spawner)
    ld.add_action(node_tilt_unit_spawner)

    return ld
