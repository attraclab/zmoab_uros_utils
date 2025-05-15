from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
	ld = LaunchDescription()

	zmoab_dir = get_package_share_directory('zmoab_uros_utils')
	launch_dir = os.path.join(zmoab_dir, 'launch')

	show_log = LaunchConfiguration('show_log')
	wheel_sep = LaunchConfiguration('wheel_sep')

	declare_show_log = DeclareLaunchArgument(
		'show_log', default_value='True', description='Show logging'
	)

	declare_wheel_sep = DeclareLaunchArgument(
		'wheel_sep', default_value='0.49', description='Wheels separate distance'
	)

	### JMOAB ATCART8 ###
	cmd_group = GroupAction([

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(launch_dir,'vel_odom_converter.launch.py')),
			launch_arguments={'show_log': show_log,
							  'wheel_sep': wheel_sep}.items()),

		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(os.path.join(launch_dir,'odom_ekf.launch.py'))),

	])

	ld.add_action(declare_show_log)
	ld.add_action(declare_wheel_sep)
	ld.add_action(cmd_group)

	return ld