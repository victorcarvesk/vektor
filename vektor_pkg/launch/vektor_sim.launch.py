from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution



def generate_launch_description():

	# Package settings -------------------------------------------------------
    package_name = 'vektor_pkg'
    package_path = get_package_share_directory(package_name)
    robot_name = 'vektor'

    # world_path = LaunchConfiguration('world', default=)

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value='base_world.sdf',
        description='SDF world file',
        )
        
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        	os.path.join(
            	get_package_share_directory('ros_gz_sim'),
            	'launch',
            	'gz_sim.launch.py'
            	)
        	),
        launch_arguments={
            'gz_args':
                [LaunchConfiguration('world')]
                + [' -r '] 
            }.items(),
     )


    #  # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
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

    rsp_launch = IncludeLaunchDescription(
		
		PythonLaunchDescriptionSource(
            os.path.join(package_path, 'launch', 'rsp.launch.py'),
			),
        launch_arguments = {
            'use_sim_time': 'true',
            'use_ros2_control': 'true',
            }.items(),

		)



	# joystick_launch = IncludeLaunchDescription(

	# 	PythonLaunchDescriptionSource([
	# 		os.path.join(
	# 			get_package_share_directory(package_name),
	# 			'launch',
	# 			'joystick.launch.py'
	# 			)
	# 		]),

	# 	launch_arguments={
	# 		'use_sim_time': 'true'
	# 		}.items()
	# 	)

    # joint_state_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen'
	# )

    spawn_entity = Node(
		package='ros_gz_sim',
		executable='create',
		arguments=[
            '-name', robot_name,
			'-topic', 'robot_description',
			],
		output='screen'
		)





    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        resource_paths = (
            os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':'
            + package_path + '/worlds'
            )
    else:
        resource_paths = (package_path + '/worlds')
        # resource_paths = os.path.join(package_path, '/world:', package_path, '/model')

    ign_rsc = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
                               value=resource_paths)

    ld = LaunchDescription()

    entities = [
        declare_world_arg,
        ign_rsc,
        gazebo_launch,
        bridge,
        rsp_launch,
        spawn_entity,
    ]

    for entity in entities:
        ld.add_action(entity)
    
    return ld