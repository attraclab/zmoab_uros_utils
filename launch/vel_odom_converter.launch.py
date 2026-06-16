from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	ld = LaunchDescription()

	show_log = LaunchConfiguration('show_log')
	wheel_sep = LaunchConfiguration('wheel_sep')
	odom_frame = LaunchConfiguration('odom_frame')
	base_frame = LaunchConfiguration('base_frame')

	declare_show_log = DeclareLaunchArgument(
		'show_log', default_value='True', description='Show logging'
	)

	declare_wheel_sep = DeclareLaunchArgument(
		'wheel_sep', default_value='0.49', description='Wheels separate distance'
	)

	declare_odom_frame = DeclareLaunchArgument(
		'odom_frame', default_value='odom', description='odom frame id'
	)

	declare_base_frame = DeclareLaunchArgument(
		'base_frame', default_value='base_link', description='base frame id'
	)

	### JMOAB ATCART8 ###
	vel_odom_converter_node = Node(
		package="zmoab_uros_utils",
		executable="vel_odom_converter",
		name="vel_odom_converter_node",
		parameters=[{'show_log': show_log,
						'wheel_sep': wheel_sep,
						'odom_frame': odom_frame,
						'base_frame': base_frame}])


	ld.add_action(declare_show_log)
	ld.add_action(declare_wheel_sep)
	ld.add_action(declare_odom_frame)
	ld.add_action(declare_base_frame)
	ld.add_action(vel_odom_converter_node)

	return ld