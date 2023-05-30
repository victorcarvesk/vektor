import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Package PATH
    pkg_path = os.path.join(get_package_share_directory('vektor_pkg'))

    # Check if we're told to use sim time
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')
    vektor_name_config = LaunchConfiguration('vektor_robot_name')

    # Process the URDF file
    xacro_file = os.path.join(pkg_path,'description','vektor.xacro')


    # ------------------ INSERTED
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use sim time if true'
    )


    robot_description_config = Command([
        'xacro ',
        xacro_file,
        ' gazebo:=ignition',
    ])
    
    params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'robot_description': robot_description_config
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
        spawn_model,
    ])