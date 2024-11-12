#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ns = LaunchConfiguration('namespace')
    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='robot namespace'
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_viz"), "rviz", "robot.rviz"]
    )

    node_rviz = GroupAction([
        PushRosNamespace(ns),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz_config_file],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             parameters=[{'use_sim_time': True}],
             output='screen')
    ])

    ld = LaunchDescription()
    
    ld.add_action(arg_namespace)
    ld.add_action(node_rviz)

    return ld