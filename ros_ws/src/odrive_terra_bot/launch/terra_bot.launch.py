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
    joy_params = PathJoinSubstitution([FindPackageShare("terra_bot"), "config", "joy.yaml"])

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("terra_bot"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("terra_bot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/terra_bot/cmd_vel", "/cmd_vel"),
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["terra_bot", "--controller-manager", "/controller_manager"],
    )

    # Create a new Node instance for the RealSense camera
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        parameters=[{
            "depth_module.profile": "640x480x15",
            "rgb_camera.profile": "640x480x15",
            "_image_transport": "compressed",
            "align_depth.enable": False,
            #"rgb_camera.enable_auto_exposure": False,
            "enable_depth": False,
            

        }],
        output="screen",
    )

    # Joy node
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    # Create a new Node instance for the teleop_twist_joy
    # Use the joy.yaml file to configure the joystick
    teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name = 'teleop_twist_joy_node',
        parameters=[joy_params],
    )

    # Create a new Node instance for the twist_stamper
    # This node will stamp the twist message with the current time and frame_id
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'frame_id': 'base_link'}],
        remappings=[('/cmd_vel_in','/cmd_vel'),
                    ('/cmd_vel_out','/terra_bot/cmd_vel')]
      )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
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
