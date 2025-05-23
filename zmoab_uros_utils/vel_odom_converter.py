import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import time
import numpy as np

class VelOdomConverter(Node):

	def __init__(self):

		super().__init__('vel_odom_converter_node')
		self.get_logger().info('start vel_odom_converter')

		#################
		### ROS Param ###
		#################
		self.declare_parameter('show_log', True)
		self.declare_parameter('wheel_sep', 0.49)
		# self.declare_parameter('release_brake', False)
		self.add_on_set_parameters_callback(self.parameter_callback)
		self.show_log = self.get_parameter('show_log').get_parameter_value().bool_value
		self.wheel_sep = self.get_parameter('wheel_sep').get_parameter_value().double_value
		# self.release_brake = self.get_parameter('release_brake').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("show_log: {}".format(self.show_log))
		self.get_logger().info("wheel_sep: {}".format(self.wheel_sep))
		# self.get_logger().info("release_brake: {}".format(self.release_brake))


		## cart params ##
		self.L = self.wheel_sep
		self.R_wheel = 0.105

		self.rpm_fb_left = 0.0
		self.rpm_fb_right = 0.0

		## odom ##
		self.br = TransformBroadcaster(self)
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.period = 0.05

		self.imu_ang_z = 0.0
		self.prev_imu_ang_z = 0.0
		self.max_z = 0.0

		## Pub/Sub ##
		qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=1)
		self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
		self.rpm_cmd_pub = self.create_publisher(Int16MultiArray, '/zmoab/rpm_cmd', qos)
		self.rpm_fb_sub = self.create_subscription(Int16MultiArray, '/zmoab/rpm_fb', self.rpm_fb_callback, qos)
		self.odom_pub = self.create_publisher(Odometry, '/zmoab/odom', 10)
		self.ahrs_pub = self.create_publisher(Float32MultiArray, '/zmoab/ahrs', 10)
		self.imu_sub = self.create_subscription(Imu, '/zmoab/imu', self.imu_callback, qos)
		self.imu_with_time_pub = self.create_publisher(Imu, '/zmoab/imu_with_time', qos)
		self.imu_error_pub = self.create_publisher(Bool, '/zmoab/imu_error', qos)

		## Loop ##
		self.timer = self.create_timer(self.period, self.timer_callback)

	def print_(self, msg):
		if self.show_log:
			self.get_logger().info("{}".format(msg))


	#####################
	### ROS callbacks ###
	#####################
	def parameter_callback(self, params):
		for param in params:
			# print(param.name, param.type_)
			if (param.name == 'show_log') and (param.type_ == Parameter.Type.BOOL):
				self.show_log = param.value
			elif (param.name == 'wheel_sep') and (param.type_ == Parameter.Type.DOUBLE):
				self.wheel_sep = param.value
				self.L = self.wheel_sep

		self.get_logger().info("Updated parameter")

		return SetParametersResult(successful=True)


	def cmd_vel_callback(self, msg):

		vx = msg.linear.x
		wz = msg.angular.z

		if (vx != 0.0) and (wz == 0.0):
			vl = vx
			vr = vx

		elif (vx == 0.0) and (wz != 0.0):

			vl = -wz * self.L/2.0
			vr = wz * self.L/2.0

		elif (vx != 0.0) and (wz != 0.0):
			R_icc = abs(vx)/abs(wz)
			sign_vx = vx/abs(vx)
			if wz > 0.0:
				# print("curve left")
				vl = (sign_vx)*(wz*(R_icc - self.L/2.0)) #/2.0
				vr = (sign_vx)*(wz*(R_icc + self.L/2.0)) #/2.0
			elif wz < 0.0:
				# print("curve right")
				vl = (sign_vx)*(abs(wz)*(R_icc + self.L/2.0)) #/2.0
				vr = (sign_vx)*(abs(wz)*(R_icc - self.L/2.0)) #/2.0
		else:
			vl = 0.0
			vr = 0.0

		left_rpm = int(self.linear_to_rpm(vl))
		right_rpm = int(self.linear_to_rpm(vr))

		rpm_cmd_msg = Int16MultiArray()
		rpm_cmd_msg.data = [left_rpm, right_rpm]

		self.rpm_cmd_pub.publish(rpm_cmd_msg)
		self.print_("vx: {:.2f} wz: {:.2f} vl: {:.2f} vr: {:.2f} rpmL: {:d} rpmR: {:d}".format(\
			vx, wz, vl, vr, left_rpm, right_rpm))

	def rpm_fb_callback(self, msg):
		self.rpm_fb_left = msg.data[0]
		self.rpm_fb_right = msg.data[1]

	def imu_callback(self, msg):
		qw = msg.orientation.w
		qx = msg.orientation.x
		qy = msg.orientation.y
		qz = msg.orientation.z

		q_list = [qx, qy, qz, qw]
		(roll, pitch, hdg) = euler_from_quaternion(q_list)

		ahrs_msg = Float32MultiArray()
		ahrs_msg.data = [np.degrees(roll), np.degrees(pitch), np.degrees(hdg)]
		self.ahrs_pub.publish(ahrs_msg)

		imu_msg = Imu()
		imu_msg.header.stamp = self.get_clock().now().to_msg()
		imu_msg.header.frame_id = "imu_link"
		imu_msg.orientation = msg.orientation
		imu_msg.orientation_covariance = msg.orientation_covariance
		imu_msg.angular_velocity = msg.angular_velocity
		# imu_msg.angular_velocity.z = msg.angular_velocity.z%(2*np.pi)
		imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
		imu_msg.linear_acceleration = msg.linear_acceleration
		imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
		self.imu_with_time_pub.publish(imu_msg)

		if ((qx == 0.0) and (qy == 0.0) and (qz == 0.0) and (qw == 0.0)):
			self.print_("BNO055 not detected")
			imu_error_msg = Bool()
			imu_error_msg.data = True
			self.imu_error_pub.publish(imu_error_msg)

		# self.prev_imu_ang_z = self.imu_ang_z%(2*np.pi)
		# self.imu_ang_z = msg.angular_velocity.z
		# if (self.imu_ang_z > self.prev_imu_ang_z):
		# 	self.max_z = self.imu_ang_z
		# 	self.print_("max_z {:.2f}".format(self.max_z))


	######################
	### Math functions ###
	######################
	def map(self, val, in_min, in_max, out_min, out_max):
		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min
		return out

	def linear_to_rpm(self, v):
		rpm = (60.0/(2.0*np.pi))*(v/self.R_wheel)
		return rpm

	def rpm_to_linear(self, rpm):
		v = rpm * ((2.0*np.pi)/60.0) * self.R_wheel
		return v

	############
	### Loop ###
	############
	def timer_callback(self):

		vl = round(self.rpm_to_linear(self.rpm_fb_left), 3)
		vr = round(self.rpm_to_linear(self.rpm_fb_right), 3)

		V = (vl + vr)/2.0

		### Calculate ODOM ###
		if (vl > 0.0) and (vr < 0.0) and (abs(V) < 0.1):
			## rotatiing CW
			V = 0.0
			Wz = (vr - vl)/self.L
			self.theta = self.theta + Wz*self.period

			path = "skid_right"

		elif (vr > 0.0) and (vl < 0.0) and (abs(V) < 0.1):
			## rotatiing CCW
			V = 0.0
			Wz = (vr - vl)/self.L
			self.theta = self.theta + Wz*self.period

			path = "skid_left"

		elif (abs(vl) > abs(vr)) or (abs(vl) < abs(vr)):
			## curving CW
			# V = (vl + vr)/2.0
			Wz = (vr-vl)/self.L
			# R_ICC = (self.L/2.0)*((vl+vr)/(vl-vr))
			R_ICC = (self.L/2.0)*((vl+vr)/(vr-vl))

			self.x = self.x - R_ICC*np.sin(self.theta) + R_ICC*np.sin(self.theta + Wz*self.period)
			self.y = self.y + R_ICC*np.cos(self.theta) - R_ICC*np.cos(self.theta + Wz*self.period)
			self.theta = self.theta + Wz*self.period

			if abs(vl) > abs(vr):
				path = "curve_right"
			else:
				path = "curve_left"

		elif vl == vr:
			V = (vl + vr)/2.0
			Wz = 0.0
			self.x = self.x + V*np.cos(self.theta)*self.period
			self.y = self.y + V*np.sin(self.theta)*self.period
			self.theta = self.theta
			path = "straight"

		else:
			V = 0.0
			Wz = 0.0
			R_ICC = 0.0

		q = quaternion_from_euler(0,0, self.theta)
		odom_msg = Odometry()
		odom_msg.header.stamp = self.get_clock().now().to_msg()
		odom_msg.header.frame_id = "odom"
		odom_msg.child_frame_id = "base_link"	#"base_footprint"	#"base_link"
		odom_msg.pose.pose.position.x = self.x
		odom_msg.pose.pose.position.y = self.y
		odom_msg.pose.pose.position.z = 0.0
		odom_msg.pose.pose.orientation.x = q[0]
		odom_msg.pose.pose.orientation.y = q[1]
		odom_msg.pose.pose.orientation.z = q[2]
		odom_msg.pose.pose.orientation.w = q[3]
		odom_msg.pose.covariance[0] = 0.0001
		odom_msg.pose.covariance[7] = 0.0001
		odom_msg.pose.covariance[14] = 0.000001	#1e12
		odom_msg.pose.covariance[21] = 0.000001	#1e12
		odom_msg.pose.covariance[28] = 0.000001	#1e12
		odom_msg.pose.covariance[35] = 0.0001
		odom_msg.twist.twist.linear.x = V #vx_odom #V
		odom_msg.twist.twist.linear.y = 0.0 #vy_odom #0.0
		odom_msg.twist.twist.angular.z = Wz #%(2*np.pi)
		self.odom_pub.publish(odom_msg)

		if self.show_log:
			self.print_("path: {:} vL: {:.3f} vR: {:.3f} V: {:.3f} W: {:.3f}  theta: {:.2f}".format(\
				path, vl,  vr, V, Wz, self.theta))

def main(args=None):

	rclpy.init(args=None)
	node = VelOdomConverter()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
