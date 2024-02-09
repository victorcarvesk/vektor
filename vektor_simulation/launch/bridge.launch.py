import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = os.path.basename(os.path.dirname(os.path.dirname(__file__)))
    package_path = get_package_share_directory(package_name)

    ros2_control = LaunchConfiguration('use_ros2_control')

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
        output='screen',
        )

    odom_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
        output='screen',
        condition=UnlessCondition(ros2_control),
        )

    tf_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
        output='screen',
        condition=UnlessCondition(ros2_control),
        )

    jsp_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            ],
        output='screen',
        condition=UnlessCondition(ros2_control),
        )

    lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            ],
        output='screen',
        )

    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/rgbd/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            ],
        output='screen',
        )

    diff_drive_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        remappings=[
            ("/cmd_vel", "/diff_cont/cmd_vel_unstamped"),
            ],
        condition=IfCondition(ros2_control),
        )
    
    tags = ['arg', 'node', 'launch']

    ld = LaunchDescription()

    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld