import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    
    package_name = os.path.basename(os.path.dirname(os.path.dirname(__file__)))
    package_path = get_package_share_directory(package_name)

    sim_mode_arg = DeclareLaunchArgument(
        name='sim_mode',
        description='Enable simulation mode if true',
        choices=['true', 'false'],
        default_value='true',
        )

    use_bridge_arg = DeclareLaunchArgument(
        name='use_bridge',
        description='Use ignition bridge if true',
        choices=['true', 'false'],
        default_value='true',
        )
    
    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        description='Use ros2_control if true',
        choices=['true', 'false'],
        default_value='false',
        )

    robot_name = LaunchConfiguration('robot_name')
    use_bridge = LaunchConfiguration('use_bridge')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('vektor_bringup'), 'launch', 'rsp.launch.py'),
            ]),
        )
    
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_path, 'launch', 'bridge.launch.py'),
            ]),
        condition=IfCondition(use_bridge),
        )
    
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            ],
        output='screen',
        )
    
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_path, 'launch', 'controllers.launch.py'),
            ]),
        condition=IfCondition(use_ros2_control),
        )

    # ESTAVA ATIVO
    # teleop_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     output='screen',
    # )
    
    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'controller']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld