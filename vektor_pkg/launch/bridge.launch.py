from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    package_name = 'vektor_pkg'
    package_path = get_package_share_directory(package_name)
    robot_name = 'vektor'

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',

            '/rgbd/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',

            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            ],
        output='screen',
    )

    ld = LaunchDescription()

    tags = ['arg', 'node', 'launch', 'delay']
    entities = locals().copy()
   
    for entity in entities:
        if any(tag in entity for tag in tags):
            ld.add_action(eval(entity, locals()))
    
    return ld