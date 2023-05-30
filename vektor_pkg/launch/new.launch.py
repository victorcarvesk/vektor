
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    vektor_description_dir = get_package_share_directory('vektor_pkg')
    
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world_file_config = LaunchConfiguration('world')

    declare_world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(vektor_description_dir,'world','house.world')],
        description='SDF world file'
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file_config}.items(),
    )

    spawn_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vektor_description_dir,
                         'launch', 'spawn_vektor_gazebo_description.launch.py')
            )
    )

    return LaunchDescription([
            declare_world_file_arg,
            gz_sim_launch,
            spawn_gazebo_launch
    ])
