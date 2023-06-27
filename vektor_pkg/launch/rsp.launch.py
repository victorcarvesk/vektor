import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'vektor_pkg'
    package_path = os.path.join(get_package_share_directory(package_name))

    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        description='Set robot name',
        default_value='vektor',
        )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        description='Use sim time if true',
        choices=['true', 'false'],
        default_value='true',
        )

    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        description='Use ros2_control if true',
        choices=['true', 'false'],
        default_value='false',
        )
    
    use_ign_arg = DeclareLaunchArgument(
        name='use_ign',
        description='Use ign if true',
        choices=['true', 'false'],
        default_value='false',
        )
    
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ign = LaunchConfiguration('use_ign')

    xacro_file = os.path.join(package_path,'description','vektor.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' gazebo:=ignition',
            ' use_ros2_control:=', use_ros2_control,
            ' sim_mode:=', use_sim_time,
            ]),
        value_type=str,
        )

    rsp_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name='robot_state_publisher',
        parameters = [{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            }],
        output = 'screen',
        )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='jsp_node',
        output='screen',
        condition=UnlessCondition(use_ign),
    )
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_path, 'launch', 'rviz.launch.py')
            ]),
        )

    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'delay']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld