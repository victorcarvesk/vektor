from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    teleoperation_pkg = get_package_share_directory('vektor_teleoperation')

    gamepad_params = os.path.join(teleoperation_pkg, 'config', 'gamepad.yaml')

    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        description='Use ros2_control if true',
        choices=['true', 'false'],
        default_value='false',
        )

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    gamepad_node = Node(
        package='joy',
        executable='joy_node',
        name='gamepad',
        parameters=[
            gamepad_params,
            ],
        output='screen',
        )
    
    teleoperation_plugin_node = Node(
        package='vektor_teleoperation',
        executable='teleop_node',
        name='teleoperation',
        output='screen',
        condition=UnlessCondition(use_ros2_control),
        )
    
    teleoperation_ros2_control_node = Node(
        package='vektor_teleoperation',
        executable='teleop_node',
        name='teleoperation',
        output='screen',
        remappings=[
            ('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'),
            ],
        condition=IfCondition(use_ros2_control),
        )

    tags = ['arg', 'node', 'launch', 'controller']

    ld = LaunchDescription()

    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld
