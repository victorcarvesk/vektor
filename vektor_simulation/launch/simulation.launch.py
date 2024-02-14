import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    simulation_pkg = get_package_share_directory('vektor_simulation')
    world_pkg = get_package_share_directory('vektor_world')

    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='base_world.sdf',
        description='SDF world file',
    )

    world_name = LaunchConfiguration('world')

    world_path = PathJoinSubstitution([world_pkg, 'worlds', world_name])

    declare_gazebo_arg = DeclareLaunchArgument(
        name='gazebo_config',
        default_value='simulation.config',
        description='Gazebo GUI config file',
    )

    gazebo_config_file = LaunchConfiguration('gazebo_config')

    gazebo_config_path = PathJoinSubstitution([
        simulation_pkg, 'config', gazebo_config_file
        ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [
            ' -r ', world_path,
            ' --gui-config ', gazebo_config_path,
            ]}.items(),
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                simulation_pkg, 'launch', 'spawn.launch.py')
        ]),
    )

    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'controller']
    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))

    return ld
