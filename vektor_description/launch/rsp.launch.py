import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
    )
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    description_pkg = get_package_share_directory("vektor_description")

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        description='Use RViz if true',
        choices=['true', 'false'],
        default_value='false',
        )
    
    sim_mode_arg = DeclareLaunchArgument(
        name='sim_mode',
        description='Enable simulation mode if true',
        choices=['true', 'false'],
        default_value='false',
        )
    
    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        description='Use ros2_control if true',
        choices=['true', 'false'],
        default_value='false',
        )

    use_rviz = LaunchConfiguration('use_rviz')
    sim_mode = LaunchConfiguration('sim_mode')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    xacro_file = os.path.join(description_pkg, 'urdf', 'vektor.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' sim_mode:=', sim_mode,
            ' ros2_control:=', use_ros2_control,
            ]),
        value_type=str,
        )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': sim_mode,
            }],
        output='screen',
        )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=UnlessCondition(sim_mode),
        )

    rviz_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        description_pkg, 'launch', 'rviz.launch.py'
                        )
                    ]),
                condition=IfCondition(use_rviz),
                ),
            ]
        )

    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'controller']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld
