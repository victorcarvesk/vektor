from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    description_pkg = get_package_share_directory("vektor_description")
    
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        description='A display config file (.rviz) to load',
        default_value='rsp.rviz',
        )
    
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    rviz_config_path = PathJoinSubstitution([
        description_pkg, 'rviz', rviz_config_file
        ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_path],
        )
    
    tags = ['arg', 'node', 'launch', 'controller']

    ld = LaunchDescription()

    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld