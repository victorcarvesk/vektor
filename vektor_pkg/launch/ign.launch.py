import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable)

from launch_ros.actions import Node

def generate_launch_description():    

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # package_name='vektor_pkg' #<--- CHANGE ME
    pkg_path = get_package_share_directory('vektor_pkg')
    gz_path = get_package_share_directory('ros_gz_sim')




    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-name', 'vektor'
    #     ],
    #     output='screen')
    
    world_config = LaunchConfiguration('world')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=['house.world'],
        description='House, tree and car.'
    )

    # world_file = PathJoinSubstitution([
    #     get_package_share_directory('vektor_pkg'),
    #     'world',
    #     world_config
    # ])

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_config}.items(),
            # launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_path,'launch','ign_rsp.launch.py')
        ])
    )







    # Launch them all!
    return LaunchDescription([
        # joystick,
        # twist_mux,
        declare_world_arg,
        gz_sim_launch,
        rsp,
        # spawn_entity,
        # rviz,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])