from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution



def generate_launch_description():

	# Package settings -------------------------------------------------------
    package_name = 'vektor_pkg'
    package_path = get_package_share_directory(package_name)
    ignition_path = get_package_share_directory('ros_gz_sim')
    robot_name = 'vektor'

    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='base_world.sdf',
        description='SDF world file',
        )
    
    world_name = LaunchConfiguration('world')
    
    world_path = PathJoinSubstitution([package_path, 'worlds', world_name])
        
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        	os.path.join(ignition_path, 'launch', 'gz_sim.launch.py')
            ),
        launch_arguments={'gz_args': [world_path, ' -r ']}.items(),
        )

    spawn_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
            os.path.join(package_path, 'launch', 'spawn.launch.py'),
			]),
		)
    
    joy_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
            os.path.join(package_path, 'launch', 'joy.launch.py'),
			]),
		)

	# joystick_launch = IncludeLaunchDescription(

	# 	PythonLaunchDescriptionSource([
	# 		os.path.join(
	# 			get_package_share_directory(package_name),
	# 			'launch',
	# 			'joystick.launch.py'
	# 			)
	# 		]),

	# 	launch_arguments={
	# 		'use_sim_time': 'true'
	# 		}.items()
	# 	)

    ign_resource_path_env = 'IGN_GAZEBO_RESOURCE_PATH'

    resource_paths = os.path.join(package_path, 'worlds')
    print(f"raw_path: {resource_paths}")

    if ign_resource_path_env in os.environ:
        resource_paths += ':' + os.environ[ign_resource_path_env]
        print(f"path: {resource_paths}")

    ld = LaunchDescription()

    ld.add_action(
        SetEnvironmentVariable(
            name=ign_resource_path_env,
            value=resource_paths,
            )
        )

    tags = ['arg', 'node', 'launch']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            print(entity)
            ld.add_action(eval(entity, locals()))
    
    return ld