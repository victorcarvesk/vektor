from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution



def generate_launch_description():

	# Package settings -------------------------------------------------------
    package_name = 'vektor_pkg'
    package_path = get_package_share_directory(package_name)
    robot_name = 'ign_test'

    world_config = LaunchConfiguration('world')

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=['base_world.sdf'],
        description='SDF world file',
        )
        

    world_file = PathJoinSubstitution([package_path,
                                       'worlds', world_config])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        	os.path.join(
            	get_package_share_directory('ros_gz_sim'),
            	'launch',
            	'gz_sim.launch.py'
            	)
        	),
            launch_arguments={'gz_args': world_file}.items(),
     )


    #  # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/ign_test/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/ign_test/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
        remappings=[
            ('/model/ign_test/odometry', '/odom'),
            ('/model/ign_test/tf', '/tf'),
            ('/lidar', '/scan'),
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
            # '-name', robot_name,
			'-topic', 'robot_description'
			],
		output='screen'
		)



    # rviz_config = os.path.join(
    #     get_package_share_directory('vektor_pkg'),
    #     'config',
    #     'vektor_lidar.rviz'
    # )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config]
    # )

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        resource_paths = (
            os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':'
            + package_path + '/worlds'
            )
    else:
        resource_paths = (package_path + '/worlds')
        # resource_paths = os.path.join(package_path, '/world:', package_path, '/model')



    # Launch them all!
    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
                               value=resource_paths),
        declare_world_arg,
        gazebo_launch,
        bridge,
        rsp_launch,
        # joint_state_node,
        spawn_entity,
        # joystick,
        # twist_mux,
        # rviz,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])