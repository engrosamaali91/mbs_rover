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
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Gazebo Configuration 
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.6'
    spawn_yaw_val = '0.00'
    

    world = os.path.join(get_package_share_directory('rover_gazebo'), 'worlds', 'plane.world')

    # Controller Configuration
    controller_config = os.path.join(get_package_share_directory("rover_gazebo"), "config", "robot_control.yaml")

    # Robot Description
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

    # Set the path to different files and folders.  
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('rover_description')).
                                                    parent.resolve())])
    # Initiate Gazebo
    command_gazebo_launch = ExecuteProcess(
            cmd=['gazebo', '--verbose', '--pause',
                 world, '-s', 'libgazebo_ros_factory.so'],
            output='screen',
    )

    command_gazebo_sim_param = ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', 'true'],
            output='screen')

    # Spawn Robot Model
    node_gazebo_robot_spawn =  Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'rover',  
                   '-topic', 'robot_description',
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val], 
        output='screen')

    # Initiate Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, 
                    {"publish_frequency": 30.0}, 
                    {'use_sim_time': True},
                    controller_config],
    )


    return LaunchDescription([ 
      gz_resource_path,
      node_robot_state_publisher,
      node_gazebo_robot_spawn, 
      command_gazebo_launch,
      command_gazebo_sim_param,   
    ])

