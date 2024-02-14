import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ros2_control = LaunchConfiguration('use_ros2_control')

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
        output='screen',
        )

    odom_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
        output='screen',
        condition=UnlessCondition(ros2_control),
        )

    tf_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
        output='screen',
        condition=UnlessCondition(ros2_control),
        )

    jsp_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='jsp_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            ],
        output='screen',
        condition=UnlessCondition(ros2_control),
        )

    lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            ],
        output='screen',
        )

    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            ],
        output='screen',
        )

    cmd_vel_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            ],
        condition=UnlessCondition(ros2_control),
        )
    
    # unstamped_bridge_node = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='cmd_vel_bridge',
    #     arguments=[
    #         '/diff_drive_base_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    #         ],
    #     remappings=[
    #         ('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'),
    #         ],
    #     condition=IfCondition(ros2_control),
    #     )
    
    # remapping_bridge_node = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='remapping_bridge',
    #     arguments=[
    #         '/diff_drive_base_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    #         ],
    #     remappings=[
    #         ('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'),
    #         ],
    #     condition=IfCondition(ros2_control),
    # )
    
    tags = ['arg', 'node', 'launch', 'controller']

    ld = LaunchDescription()

    entities = locals().copy()

    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld