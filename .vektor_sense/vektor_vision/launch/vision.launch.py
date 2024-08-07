import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    vision_pkg = get_package_share_directory("vektor_vision")

    vision_params = os.path.join(vision_pkg, 'config', 'vision.yaml')

    vision_node = Node(
        package = 'vektor_vision',
        executable = 'vision_node.py',
        parameters = [vision_params],
        output = 'screen',
        )

    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'controller']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld
