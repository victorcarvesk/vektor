from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'vektor_pkg'
    package_path = get_package_share_directory(package_name)

    use_joy_arg = DeclareLaunchArgument(
        name='use_joy',
        description='Use joy if true',
        choices=['true', 'false'],
        default_value='false',
        )
    
    joy_arg = DeclareLaunchArgument(
        name='joy_port',
        description='Port of joy connection',
        default_value='js0',
        )
    
    use_joy = LaunchConfiguration('use_joy')
    joy_port = LaunchConfiguration('joy_port')
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        condition=IfCondition(use_joy),
        )
    
    teleop_node = Node(
        package='vektor_pkg',
        executable='vektor_node',
        name='teleop_joy',
        condition=IfCondition(use_joy),
        )
    
    # Criar command node!

    tags = ['arg', 'node', 'launch']

    ld = LaunchDescription()

    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld
