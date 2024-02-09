from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params = os.path.join(
        get_package_share_directory('vektor_teleoperation'),
        'config',
        'joystick.yaml',
        )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            joy_params,
        ],
        output='screen',
    )

    teleop_node = Node(
        package='vektor_teleoperation',
        executable='teleop_node',
        output='screen',
    )

    tags = ['arg', 'node', 'launch']

    ld = LaunchDescription()

    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld