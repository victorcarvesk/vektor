import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Package settings -------------------------------------------------------
    package_name = os.path.basename(os.path.dirname(os.path.dirname(__file__)))

    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='base_world.sdf',
        description='SDF world file',
    )

    world_name = LaunchConfiguration('world')

    world_path = PathJoinSubstitution([
        get_package_share_directory('vektor_world'),
        'worlds',
        world_name,
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [world_path, ' -r ']}.items(),
        # launch_arguments={'gz_args': [world_path, ' -r ', '--gui-config ', gui_config]}.items(),
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'spawn.launch.py')
        ]),
    )

    ign_resource_path_env = 'IGN_GAZEBO_RESOURCE_PATH'

    resource_paths = os.path.join(get_package_share_directory(package_name), 'world')
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

    tags = ['arg', 'node', 'launch', 'controller']
    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))

    return ld
