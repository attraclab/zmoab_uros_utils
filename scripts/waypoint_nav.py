import time
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from lifecycle_msgs.srv import GetState
import numpy as np

class WaypointNav(Node):

	def __init__(self):

		super().__init__("waypoint_nav_node")

		self.follow_waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

	def print_(self, msg):
		self.get_logger().info(msg)

	#####################################
	### FollowWaypoint action client ###
	#####################################

	def followWaypoint_send_goal(self, poses):

		self.print_("Waiting for 'FollowWaypoint' action server")
		while not self.follow_waypoint_client.wait_for_server(timeout_sec=1.0):
			self.print_("FollowWaypoint action server not available, waiting...")

		goal_msg =  FollowWaypoints.Goal() #FollowWaypoint.Goal()
		goal_msg.poses = poses
		#goal_msg.behavior_tree = ''

		self.got_followWaypoint_goal = True
		self.followWaypoint_done = False
		self.nav2pose_done = False

		self.print_(f'FollowWaypoint Navigating with {len(goal_msg.poses)} goals....')
		send_goal_future = self.follow_waypoint_client.send_goal_async(goal_msg, self.followWaypoint_feedbackCallback)
		send_goal_future.add_done_callback(self.followWaypoint_goal_response_callback)

	def followWaypoint_goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.print_('FollowWaypoint Goal rejected :(')
			return

		self.print_('FollowWaypoint Goal accepted :)')
		self.print_("followWaypoint_done", self.followWaypoint_done)
		self.followWaypoint_goal_handle = goal_handle

		get_result_future = goal_handle.get_result_async()
		get_result_future.add_done_callback(self.followWaypoint_get_result_callback)

	def followWaypoint_get_result_callback(self, future):
		result = future.result().result
		# self.print_('Result: {0}'.format(result.result))
		self.print_('FollowWaypoint result {:}'.format(result))
		if self.current_waypoint == (self.goal_poses_len-1):
			self.followWaypoint_done = True
			self.print_("followWaypoint_done", self.followWaypoint_done)
			# Loop over once finished the last point
			self.followWaypoint_send_goal(self.goal_poses)

	def followWaypoint_feedbackCallback(self, msg):
		# self.print_('FollowWaypoint feedback {:}'.format(msg.feedback))
		self.current_waypoint = msg.feedback.current_waypoint

		if ((self.closest_idx + self.current_waypoint) >= self.goal_poses_len):
			self.map_current_waypoint = self.closest_idx + self.current_waypoint - self.goal_poses_len 
		else:
			self.map_current_waypoint = self.closest_idx + self.current_waypoint

		if (time.time() - self.feedback_last_stamp) > 1.0:
			self.print_("goal_len {} current_wp {} map_current_wp {}".format(self.goal_poses_len, self.current_waypoint, self.map_current_waypoint))
			self.feedback_last_stamp = time.time()
		# feedback = msg.feedback
		return

	def followWaypoint_cancel_done(self, future):
		cancel_response = future.result()
		self.last_cancel_stamp = time.time()

		if len(cancel_response.goals_canceling) > 0:
			self.print_('FollowWaypoint Goal successfully canceled')
		else:
			self.print_('FollowWaypoint Goal failed to cancel')