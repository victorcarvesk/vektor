from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():

	# Package settings -------------------------------------------------------
    package_name = 'vektor_pkg'
    package_path = os.path.join(get_package_share_directory(package_name))

    rsp_launch = IncludeLaunchDescription(
		
		PythonLaunchDescriptionSource([
			os.path.join(
				get_package_share_directory(package_name),
				'launch',
				'rsp.launch.py'
				)
			]),

		launch_arguments={
			'use_sim_time': 'true',
			'use_ros2_control': 'true'
			}.items()
		)


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        	os.path.join(
            	get_package_share_directory('ros_gz_sim'),
            	'launch',
            	'gz_sim.launch.py'
            	)
        	]),
            # launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
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
			'-topic', 'robot_description',
			'-entity', 'vektor'
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



    # Launch them all!
    return LaunchDescription([
        rsp_launch,
        # joint_state_node,
        # joystick,
        # twist_mux,
        gazebo_launch,
        spawn_entity,
        # rviz,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])