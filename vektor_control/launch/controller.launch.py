from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control',
            'load_controller', 'joint_state_broadcaster',
            '--set-state', 'active',
            ],
        output='screen',
        )

    diff_drive_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control',
            'load_controller', 'diff_drive_base_controller',
            '--set-state', 'active',
            ],
        output='screen',
        )
    
    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'controller']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld