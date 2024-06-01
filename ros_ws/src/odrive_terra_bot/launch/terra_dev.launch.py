from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    # ROS2 bridge server "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    bridge = new Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        arguments: ['--ros-args', '--log-level', 'info']
    )

    # Human detection node
    human_detection = new Node(
        package='human_detection',
        executable='human_detection',
        arguments: ['--ros-args', '--log-level', 'info']
    )

    # Human following node
    human_following = new Node(
        package='human_following',
        executable='human_following',
        arguments: ['--ros-args', '--log-level', 'info']
    )


    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        realsense_node,
        teleop_node,
        twist_stamper,
        joy
    ]

    return LaunchDescription(declared_arguments + nodes)
