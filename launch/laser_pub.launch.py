from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	zmoab_uros_utils_dir = get_package_share_directory('zmoab_uros_utils')
	launch_dir = os.path.join(zmoab_uros_utils_dir, 'launch')

	cmd_group = GroupAction([

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pc2l_livox.launch.py'))),

		Node(package='tf2_ros',
				executable='static_transform_publisher',
				name='static_tf_pub_imu',
				arguments=['0.36', '0', '0','0', '0', '0', '1','base_link','laser_frame'],
				),

		Node(
				package='zmoab_uros_utils',
				executable='laserscan_relay',
				name='laserscan_relay',
				output='screen',
			)
	])

	ld.add_action(cmd_group)

	return ld