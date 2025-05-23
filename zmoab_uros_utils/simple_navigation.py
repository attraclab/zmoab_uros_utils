import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8MultiArray, Int8
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import time
import numpy as np
from simple_pid import PID
import zmoab_uros_utils.nav_utils  as nv
import copy

class SimpleNavigation(Node):

	def __init__(self):

		super().__init__("simple_navigation_node")

		self.declare_parameter('vx_trace', 0.5)
		self.declare_parameter('wz_trace', 0.5)
		self.declare_parameter('wz_turning', 0.30)
		self.declare_parameter('slowdown_dist', 1.0)
		self.declare_parameter('vx_slowdown', 0.2)
		self.declare_parameter('accel_period', 3.0)

		self.declare_parameter('min_turning_angle', 3.0)
		self.declare_parameter('goal_dist_thresh', 0.1)
		self.declare_parameter('hdg_diff_thresh', 1.0)

		self.declare_parameter('turn_p', 1.0)
		self.declare_parameter('turn_i', 0.0)
		self.declare_parameter('turn_d', 0.0)

		self.declare_parameter('steer_p', 1.0)
		self.declare_parameter('steer_i', 0.0)
		self.declare_parameter('steer_d', 0.0)


		self.add_on_set_parameters_callback(self.parameter_callback)

		self.vx_trace = self.get_parameter('vx_trace').get_parameter_value().double_value
		self.wz_trace = self.get_parameter('wz_trace').get_parameter_value().double_value
		self.wz_turning = self.get_parameter('wz_turning').get_parameter_value().double_value
		self.slowdown_dist = self.get_parameter('slowdown_dist').get_parameter_value().double_value
		self.vx_slowdown = self.get_parameter('vx_slowdown').get_parameter_value().double_value
		self.accel_period = self.get_parameter('accel_period').get_parameter_value().double_value
		self.min_turning_angle = self.get_parameter('min_turning_angle').get_parameter_value().double_value
		self.goal_dist_thresh = self.get_parameter('goal_dist_thresh').get_parameter_value().double_value
		self.hdg_diff_thresh = self.get_parameter('hdg_diff_thresh').get_parameter_value().double_value
		self.turn_p = self.get_parameter('turn_p').get_parameter_value().double_value
		self.turn_i = self.get_parameter('turn_i').get_parameter_value().double_value
		self.turn_d = self.get_parameter('turn_d').get_parameter_value().double_value
		self.steer_p = self.get_parameter('steer_p').get_parameter_value().double_value
		self.steer_i = self.get_parameter('steer_i').get_parameter_value().double_value
		self.steer_d = self.get_parameter('steer_d').get_parameter_value().double_value

		self.print_("Using parameters as below")
		self.print_("vx_trace: {}".format(self.vx_trace))
		self.print_("wz_trace: {}".format(self.wz_trace))
		self.print_("wz_turning: {}".format(self.wz_turning))
		self.print_("slowdown_dist: {}".format(self.slowdown_dist))
		self.print_("vx_slowdown: {}".format(self.vx_slowdown))
		self.print_("accel_period: {}".format(self.accel_period))
		self.print_("min_turning_angle: {}".format(self.min_turning_angle))
		self.print_("goal_dist_thresh: {}".format(self.goal_dist_thresh))
		self.print_("hdg_diff_thresh: {}".format(self.hdg_diff_thresh))
		self.print_("turn_p: {}".format(self.turn_p))
		self.print_("turn_i: {}".format(self.turn_i))
		self.print_("turn_d: {}".format(self.turn_d))
		self.print_("steer_p: {}".format(self.steer_p))
		self.print_("steer_i: {}".format(self.steer_i))
		self.print_("steer_d: {}".format(self.steer_d))

		## PID ##
		self.pid_turn = PID(self.turn_p, self.turn_i, self.turn_d, setpoint=0.0)
		self.pid_turn.tunings = (self.turn_p, self.turn_i, self.turn_d)
		self.pid_turn.sample_time = 0.001
		self.pid_turn.output_limits = (-100.0, 100.0)
		self.prev_turn_p = self.turn_p
		self.prev_turn_i = self.turn_i
		self.prev_turn_d = self.turn_d

		self.pid_steer = PID(self.steer_p, self.steer_i, self.steer_d, setpoint=0.0)
		self.pid_steer.tunings = (self.steer_p, self.steer_i, self.steer_d)
		self.pid_steer.sample_time = 0.001
		self.pid_steer.output_limits = (-100.0, 100.0)
		self.prev_steer_p = self.steer_p
		self.prev_steer_i = self.steer_i
		self.prev_steer_d = self.steer_d

		## Variables ##
		self.got_tf = False
		self.mb_yaw = 0.0
		self.map_base_x = 0.0
		self.map_base_y = 0.0
		self.wp_count = 0
		self.total_point = 0
		self.button_press = 0
		self.last_wait_for_button_stamp  = time.time()
		self.prev_button_press = 0
		self.got_button_pressed = False
		self.store_poses_x = []
		self.store_poses_y = []
		self.last_pub_auto_run_stamp = time.time()
		self.cart_mode = 1
		self.prev_cart_mode = 1

		## Nav ##
		self.init_nav_params()

		## Pub/Sub ###
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		self.guided_point_sub = self.create_subscription(Pose, '/nav/guided_point', self.guided_point_callback, 10)
		self.guided_cancel_sub = self.create_subscription(Bool, '/nav/cancel', self.guided_cancel_callback, 10)
		self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
		self.goal_poses_sub = self.create_subscription(PoseArray, '/goal_poses', self.goal_poses_callback, 10)


		self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
		path_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											durability=rclpy.qos.DurabilityPolicy.VOLATILE, \
											depth=10)
		self.path_pub = self.create_publisher(Path, "/goal_path", path_qos)
		uros_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											durability=rclpy.qos.DurabilityPolicy.VOLATILE, \
											depth=1)
		self.switch_sub = self.create_subscription(Int8MultiArray, "/zmoab/switch", self.switch_callback, uros_qos)
		self.auto_run_state_pub = self.create_publisher(Int8, "/zmoab/auto_run_state", uros_qos)
		self.cart_mode_sub = self.create_subscription(Int8, "/zmoab/cart_mode", self.cart_mode_callback, uros_qos)
		self.cart_mode_cmd_pub = self.create_publisher(Int8, "/zmoab/cart_mode_cmd", uros_qos)

		## Loop ##
		self.print_("Start simple_navigation ")
		self.timer = self.create_timer(0.02, self.timer_callback)


	########################
	### Helper functions ###
	########################
	def print_(self, msg):
		self.get_logger().info('{}'.format(msg))

	def init_nav_params(self):
		self.nav_step = 1
		self.got_goal_point = False
		self.soft_start_tic = time.time()
		self.got_error = False
		self.last_stamp_nav_step = time.time()
		self.goal_poses_x = []
		self.goal_poses_y = []
		self.wp_count = 0
		self.total_point = 0
		self.got_button_pressed = False

	#####################
	### ROS callbacks ###
	#####################
	def parameter_callback(self, params):
		for param in params:
			# self.print_(param.name, param.type_)
			if (param.name == 'vx_trace') and (param.type_ == Parameter.Type.DOUBLE):
				self.vx_trace = param.value
			elif (param.name == 'wz_trace') and (param.type_ == Parameter.Type.DOUBLE):
				self.wz_trace = param.value
			elif (param.name == 'wz_turning') and (param.type_ == Parameter.Type.DOUBLE):
				self.wz_turning = param.value
			elif (param.name == 'slowdown_dist') and (param.type_ == Parameter.Type.DOUBLE):
				self.slowdown_dist = param.value
			elif (param.name == 'vx_slowdown') and (param.type_ == Parameter.Type.DOUBLE):
				self.vx_slowdown = param.value
			elif (param.name == 'accel_period') and (param.type_ == Parameter.Type.DOUBLE):
				self.accel_period = param.value
			elif (param.name == 'min_turning_angle') and (param.type_ == Parameter.Type.DOUBLE):
				self.min_turning_angle = param.value
			elif (param.name == 'goal_dist_thresh') and (param.type_ == Parameter.Type.DOUBLE):
				self.goal_dist_thresh = param.value
			elif (param.name == 'hdg_diff_thresh') and (param.type_ == Parameter.Type.DOUBLE):
				self.hdg_diff_thresh = param.value
			elif (param.name == 'turn_p') and (param.type_ == Parameter.Type.DOUBLE):
				self.prev_turn_p = self.turn_p
				self.turn_p = param.value
			elif (param.name == 'turn_i') and (param.type_ == Parameter.Type.DOUBLE):
				self.prev_turn_i = self.turn_i
				self.turn_i = param.value
			elif (param.name == 'turn_d') and (param.type_ == Parameter.Type.DOUBLE):
				self.prev_turn_d = self.turn_d
				self.turn_d = param.value
			elif (param.name == 'steer_p') and (param.type_ == Parameter.Type.DOUBLE):
				self.prev_steer_p = self.steer_p
				self.steer_p = param.value
			elif (param.name == 'steer_i') and (param.type_ == Parameter.Type.DOUBLE):
				self.prev_steer_i = self.steer_i
				self.steer_i = param.value
			elif (param.name == 'steer_d') and (param.type_ == Parameter.Type.DOUBLE):
				self.prev_steer_d = self.steer_d
				self.steer_d = param.value

		self.print_("Updated parameters")

		if (self.prev_turn_p != self.turn_p) or (self.prev_turn_i != self.turn_i) or (self.prev_turn_d != self.turn_d):
			self.prev_turn_p = self.turn_p
			self.prev_turn_i = self.turn_i
			self.prev_turn_d = self.turn_d
			# self.update_pid_gains(self.cross_p, self.cross_i, self.cross_d)
			self.print_("Update Turning PID as {:f} {:f} {:f}".format(self.turn_p,self.turn_i,self.turn_d))
			self.pid_turn.tunings = (self.turn_p, self.turn_i, self.turn_d)

		if (self.prev_steer_p != self.steer_p) or (self.prev_steer_i != self.steer_i) or (self.prev_steer_d != self.steer_d):
			self.prev_steer_p = self.steer_p
			self.prev_steer_i = self.steer_i
			self.prev_steer_d = self.steer_d
			# self.update_pid_gains(self.cross_p, self.cross_i, self.cross_d)
			self.print_("Update Steering PID as {:f} {:f} {:f}".format(self.steer_p,self.steer_i,self.steer_d))
			self.pid_steer.tunings = (self.steer_p, self.steer_i, self.steer_d)

		return SetParametersResult(successful=True)
	
	#####################
	### ROS callbacks ###
	#####################
	def publish_path(self, goal_pose, start_pose):

		
		start_pose_stamped = PoseStamped()
		start_pose_stamped.header.stamp = self.get_clock().now().to_msg()
		start_pose_stamped.header.frame_id = "odom"
		start_pose_stamped.pose.position.x = start_pose[0]
		start_pose_stamped.pose.position.y = start_pose[1]

		goal_pose_stamped = PoseStamped()
		goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
		goal_pose_stamped.header.frame_id = "odom"
		goal_pose_stamped.pose.position.x = goal_pose[0]
		goal_pose_stamped.pose.position.y = goal_pose[1]

		# X = np.linspace(start_pose[0], goal_pose[0], 10)
		# Y = np.linspace(start_pose[1], goal_pose[1], 10)

		path_msg = Path()
		path_msg.header.stamp = self.get_clock().now().to_msg()
		path_msg.header.frame_id = "map"
		path_msg.poses.append(start_pose_stamped)
		path_msg.poses.append(goal_pose_stamped)

		# for x,y in zip(X,Y):
		# 	pose_stamped = PoseStamped()
		# 	pose_stamped.header.stamp = self.get_clock().now().to_msg()
		# 	pose_stamped.header.frame_id = "map"
		# 	pose_stamped.pose.position.x = x
		# 	pose_stamped.pose.position.y = y

		# 	path_msg.poses.append(pose_stamped)

		self.path_pub.publish(path_msg)

	def guided_point_callback(self, msg):
		self.goal_x = msg.position.x
		self.goal_y = msg.position.y
		self.got_goal_point = True
		self.goal_poses_x = []
		self.goal_poses_y = []

		if (self.cart_mode != 2):
			self.pub_cart_mode(2)

	def goal_pose_callback(self, msg):
		self.goal_x = msg.pose.position.x
		self.goal_y = msg.pose.position.y
		self.got_goal_point = True
		self.goal_poses_x = []
		self.goal_poses_y = []

		if (self.cart_mode != 2):
			self.pub_cart_mode(2)

	def goal_poses_callback(self, msg):

		self.goal_poses_x = []
		self.goal_poses_y = []
		for pose in msg.poses:
			self.goal_poses_x.append(pose.position.x)
			self.goal_poses_y.append(pose.position.y)

		self.total_point = len(self.goal_poses_x)
		self.got_goal_point = True
		self.wp_count = 0
		self.goal_x = self.goal_poses_x[0]
		self.goal_y = self.goal_poses_y[0]

		self.store_poses_x = copy.deepcopy(self.goal_poses_x)
		self.store_poses_y = copy.deepcopy(self.goal_poses_y)

		if (self.cart_mode != 2):
			self.pub_cart_mode(2)

	def guided_cancel_callback(self, msg):
		self.got_goal_point = False
		self.init_nav_params()
		self.publish_cmd_vel(0.0, 0.0)

	def publish_cmd_vel(self, vx, wz):
		cmd_vel_msg = Twist()
		cmd_vel_msg.linear.x = vx
		cmd_vel_msg.angular.z = wz
		self.cmd_vel_pub.publish(cmd_vel_msg)

	def switch_callback(self, msg):
		self.prev_button_press = self.button_press
		self.button_press = msg.data[0]

		if (self.prev_button_press != self.button_press) and (self.button_press == 0):
			self.got_button_pressed = True

			if ((self.total_point == 0) and (len(self.store_poses_x) != 0)):
				self.goal_poses_x = copy.deepcopy(self.store_poses_x)
				self.goal_poses_y = copy.deepcopy(self.store_poses_y)
				self.goal_x = self.goal_poses_x[0]
				self.goal_y = self.goal_poses_y[0]
				self.total_point = len(self.goal_poses_x)
				self.got_goal_point = True
				self.wp_count = 0
				self.got_button_pressed = False

	def pub_auto_run_state(self, state):

		if (time.time() - self.last_pub_auto_run_stamp) > 1.0:
			auto_run_state_msg = Int8()
			auto_run_state_msg.data = state
			self.auto_run_state_pub.publish(auto_run_state_msg)
			self.last_pub_auto_run_stamp = time.time()

	def cart_mode_callback(self, msg):
		self.cart_mode = msg.data

	def pub_cart_mode(self, mode):
		cart_mode_msg = Int8()
		cart_mode_msg.data = mode
		self.cart_mode_cmd_pub.publish(cart_mode_msg)


	############
	### Loop ###
	############

	def timer_callback(self):

		if (self.got_goal_point and self.got_tf):

			goal_pose = [self.goal_x, self.goal_y]
			bot_pose = [self.map_base_x, self.map_base_y]
			self.last_stamp_nav_step = time.time()

			

			#######################
			### nav_step 1      ###
			### turning to goal ###
			#######################
			if self.nav_step == 1:
				
				turn_done, diff_ang, wz = nv.turning_to_goalPoint(goal_pose, bot_pose, self.mb_yaw, 
																	self.pid_turn, self.wz_turning, self.min_turning_angle)
				# turn_done, diff_ang, wz = self.turning_to_goalPoint(goal_pose, bot_pose, self.mb_yaw)

				if turn_done:
					self.publish_cmd_vel(0.0, 0.0)
					self.print_("Done turning")
					self.nav_step = 2
					self.soft_start_tic = time.time()

					self.start_pose = [self.map_base_x, self.map_base_y]
					self.goal_dist = nv.get_distance_between_points(goal_pose, bot_pose)
				else:
					self.publish_cmd_vel(0.0, wz)
					self.print_("nav_step 1 | gx {:.1f} gy {:.1f} bx {:.1f} by: {:.1f} diff_ang: {:.2f} hdg: {:.2f} wz: {:.2f}".format(\
					goal_pose[0], goal_pose[1], bot_pose[0], bot_pose[1], diff_ang, np.degrees(self.mb_yaw), wz))

			########################
			### nav_step 2       ###
			### running straight ###
			########################
			elif (self.nav_step == 2):
				# self.init_nav_params()

				dist_from_start = nv.get_distance_between_points(self.start_pose, bot_pose)
				dist_remain_to_go = self.goal_dist - dist_from_start

				### calculate wz ###
				steer_diff_ang, dir_sign = nv.smallestDiffAng_fromPath(goal_pose, bot_pose, self.mb_yaw)
				pid_in = steer_diff_ang*dir_sign
				pid_steer_out = -self.pid_steer(pid_in)
				if steer_diff_ang <= self.hdg_diff_thresh:
					wz = 0.0
				else:
					wz = nv.clamp(pid_steer_out, -self.wz_trace, self.wz_trace)

				### calculate vx ###
				if (time.time() - self.soft_start_tic) > self.accel_period:
					speed_scale = 1.0
				else:
					t = time.time() - self.soft_start_tic
					speed_scale = (6*(t/self.accel_period)**5 - 15*(t/self.accel_period)**4 + 10*(t/self.accel_period)**3)

				if dist_remain_to_go <= self.slowdown_dist:
					vx = self.vx_slowdown
				else:
					vx = self.vx_trace*speed_scale

				### logging ###
				self.print_("nav_step 2 | distFromStart: {:.1f} distRemain: {:.2f} pid_i: {:.2f} pid_o: {:.2f} vx: {:.2f} wz: {:.2f}".format(\
					dist_from_start, dist_remain_to_go, pid_in, pid_steer_out, vx, wz))

				### Check reach goal ###
				if dist_remain_to_go <= self.goal_dist_thresh:
					self.publish_cmd_vel(0.0, 0.0)
					self.print_("Reach goal point")
					if (len(self.goal_poses_x) == 0):
						self.init_nav_params()
					else:
						
						if (self.wp_count == (self.total_point - 1)):
							self.print_("Mission Done")
							self.init_nav_params()
						else:
							self.got_goal_point = False
						
					# self.pub_mission_done()
				else:
					self.publish_cmd_vel(vx, wz)


			## continuously publish path ##
			self.publish_path(goal_pose, bot_pose)
			self.pub_auto_run_state(1)

		elif ((not self.got_goal_point) and self.got_tf and (not self.got_button_pressed) and (self.total_point != 0)):

			self.pub_auto_run_state(2)

			if ((time.time() - self.last_wait_for_button_stamp) > 1.0):
				self.print_("wait for button press...")
				self.last_wait_for_button_stamp = time.time()

			
		elif ((not self.got_goal_point) and self.got_tf and (self.got_button_pressed)):

			self.print_("Button press go next")
			self.wp_count += 1
			self.goal_x = self.goal_poses_x[self.wp_count]
			self.goal_y = self.goal_poses_y[self.wp_count]
			self.got_button_pressed = False
			self.got_goal_point = True
			self.nav_step = 1

		else:
			self.pub_auto_run_state(3)


		## Getting TF ##
		try:
			tf_map_base = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

			self.map_base_x = tf_map_base.transform.translation.x
			self.map_base_y = tf_map_base.transform.translation.y

			mb_qx = tf_map_base.transform.rotation.x
			mb_qy = tf_map_base.transform.rotation.y
			mb_qz = tf_map_base.transform.rotation.z
			mb_qw = tf_map_base.transform.rotation.w
			mb_q_list = [mb_qx, mb_qy, mb_qz, mb_qw]
			(mb_roll, mb_pitch, self.mb_yaw) = euler_from_quaternion(mb_q_list)

			# self.map_to_base_TF.translation.x = tf_map_base.transform.translation.x
			# self.map_to_base_TF.translation.y = tf_map_base.transform.translation.y
			# self.map_to_base_TF.translation.z = tf_map_base.transform.translation.z
			# self.map_to_base_TF.rotation.x = tf_map_base.transform.rotation.x
			# self.map_to_base_TF.rotation.y = tf_map_base.transform.rotation.y
			# self.map_to_base_TF.rotation.z = tf_map_base.transform.rotation.z
			# self.map_to_base_TF.rotation.w = tf_map_base.transform.rotation.w

			self.got_tf = True

			# self.print_("yaw {:.2f}".format(np.degrees(self.mb_yaw)))


			# if (time.time() - self.last_tf_pub_stamp) >= self.tf_pub_time:
			# 	self.map_base_pub.publish(self.map_to_base_TF)
			# 	self.last_tf_pub_stamp = time.time()

		except TransformException as ex:
			self.print_('Could not transform map to base_link')
			return

	

def main(args=None):
	rclpy.init(args=args)
	node = SimpleNavigation()
	rclpy.spin(node)
	node.destroy()
	rclpy.shutdown()

if __name__ == '__main__':
	main()