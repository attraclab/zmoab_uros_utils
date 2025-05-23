from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

	ld = LaunchDescription()

	simple_nav_node = Node(
		package="zmoab_uros_utils",
		executable="simple_navigation",
		name="simple_navigation_node",
		output='screen',
		parameters=[os.path.join(get_package_share_directory('zmoab_uros_utils'),'config','simple_nav.yaml')])

	ld.add_action(simple_nav_node)

	return ld