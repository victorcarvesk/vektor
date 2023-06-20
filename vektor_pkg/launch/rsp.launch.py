import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # Package settings -------------------------------------------------------
    package_name = 'vektor_pkg'
    robot_name = 'vektor'
    package_path = os.path.join(get_package_share_directory(package_name))

    # Arguments settings -----------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use sim time if true',
        )


    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        choices=['true', 'false'],
        default_value='true',
        description='Use ros2_control if true',
        )
    
    use_rviz_arg = DeclareLaunchArgument(
        name='rviz',
        choices=['true', 'false'],
        default_value='true',
        description='Use RViz if true',
        )

    # Xacro settings ---------------------------------------------------------
    xacro_file = os.path.join(package_path,'description','vektor.xacro')

    robot_description_config = Command([
        'xacro ', xacro_file,
        ' gazebo:=ignition',
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time,
        ' namespace:=', robot_name,
        ])

    # Create a robot_state_publisher node ------------------------------------
    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time,
        }

    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name='robot_state_publisher',
        output = 'screen',
        parameters = [params],
        )
    
    rviz_config = os.path.join(
        package_path,
        'config',
        'vektor_depth.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config,
            ],
        condition=IfCondition(
            LaunchConfiguration('rviz'),
            ),
        
    )

    # Return the launch description ------------------------------------------

    ld = LaunchDescription()

    entities = [
        use_sim_time_arg,
        use_ros2_control_arg,
        use_rviz_arg,
        node_robot_state_publisher,
        rviz,
        ]
    
    for entity in entities:
        ld.add_action(entity)

    return ld