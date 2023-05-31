from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package settings -------------------------------------------------------
    package_name = 'vektor_pkg'
    robot_name = 'vektor_core'
    package_path = os.path.join(get_package_share_directory(package_name))


    # Arguments settings -----------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use sim time if true'
        )


    # use_ros2_control_arg = DeclareLaunchArgument(
    #     name='use_ros2_control',
    #     choices=['true', 'false'],
    #     default_value='true',
    #     description='Use ros2_control if true'
    #     )

    # Xacro settings ---------------------------------------------------------
    xacro_file = os.path.join(package_path,'description','vektor.xacro')

    robot_description_config = Command([
        'xacro ',
        xacro_file,
        ' gazebo:=ignition',
        ' namespace:=', robot_name
        ])

    # Create a robot_state_publisher node ------------------------------------
    params = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_description_config
        }

    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name='robot_state_publisher',
        output = 'screen',
        parameters = [params]
        )

    spawn_entity = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=[
            '-name', robot_name,
			'-topic', 'robot_description'
			],
		output='screen'
		)

    # Return the launch description ------------------------------------------
    return LaunchDescription([
        use_sim_time_arg,
        # use_ros2_control_arg,
        node_robot_state_publisher,
        # joint_state_node,
        spawn_entity
        ])

