from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare argument for detection type
    detection_type_arg = DeclareLaunchArgument(
        'detection_type', default_value='hog',
        description='Select detection type: hog, yolo, aruco, or wave')

    # Declare argument for configuration file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=[FindPackageShare('odrive_terra_dev'), '/config/tracker.yaml'],
        description='Path to the configuration file')

    # Conditional nodes to launch based on detection type
    detect_human_hog = Node(
        package='odrive_terra_dev',
        executable='detect_human',
        name='detect_human_hog',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('detection_type'), "' == 'hog'"])),
        arguments=['--ros-args', '--log-level', 'debug']
    )

    detect_human_yolo = Node(
        package='odrive_terra_dev',
        executable='detect_human_yolo',
        name='detect_human_yolo',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('detection_type'), "' == 'yolo'"])),
        remappings=[
            ('/image_in', '/color/image_raw/compressed')  # Remap /image_in to /color/image_raw/compressed
        ]
    )

    detect_human_yolo_wave = Node(
        package='odrive_terra_dev',
        executable='detect_human_yolo_wave',
        name='detect_human_yolo_wave',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('detection_type'), "' == 'yolo_wave'"])),
        remappings=[
            ('/image_in', '/color/image_raw/compressed')  # Remap /image_in to /color/image_raw/compressed
        ]
    )

    detect_human_wave = Node(
        package='odrive_terra_dev',
        executable='detect_human_with_wave',
        name='detect_human_wave',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('detection_type'), "' == 'wave'"])),
        arguments=['--ros-args', '--log-level', 'debug']
    )

    detect_aruco = Node(
        package='odrive_terra_dev',
        executable='detect_aruco',
        name='detect_aruco',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('detection_type'), "' == 'aruco'"])),
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # rqt_image_view Node to visualize /image_out/compressed
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        arguments=['/image_out/compressed'],  # Use /image_out/compressed for compressed images
        output='screen'
    )

    delayed_image = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[rqt_image_view_node]
    )

    return LaunchDescription([
        detection_type_arg,
        config_file_arg,
        detect_human_hog,
        detect_human_yolo,
        detect_human_yolo_wave,
        detect_human_wave,
        detect_aruco,
        delayed_image,
    ])
