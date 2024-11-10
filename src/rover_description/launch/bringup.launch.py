
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

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

    # Launch Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # namespace="rover",
        output="screen",
        parameters=[robot_description],
    )
    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        # namespace="rover",
        output="screen",
        parameters=[robot_description],
    )
    # Launch Joint State Publisher GUI
    node_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        # namespace="rover",
        output="screen",
    )

    # Launch RViz
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace="rover",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("rover_viz"), "rviz", "model.rviz"])],
    )

    ld = LaunchDescription()

    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher_gui)
    ld.add_action(node_rviz)

    return ld