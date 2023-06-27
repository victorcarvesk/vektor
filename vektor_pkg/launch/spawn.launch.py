import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'vektor_pkg'
    package_path = os.path.join(get_package_share_directory(package_name))

    robot_name = LaunchConfiguration('robot_name')
    
    rsp_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
            os.path.join(package_path, 'launch', 'rsp.launch.py'),
			]),
        launch_arguments={
            'use_ign': 'true',
            'rviz_config': 'vektor_ign.rviz',
            }.items()
		)
    
    bridge_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
            os.path.join(package_path, 'launch', 'bridge.launch.py'),
			]),
		)
    
    spawn_node = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=[
            '-name', robot_name,
			'-topic', 'robot_description',
			],
		output='screen',
		)
    
    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'delay']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld