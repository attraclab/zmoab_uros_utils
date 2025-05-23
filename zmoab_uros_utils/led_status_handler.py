import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
import time

class LEDHandler(Node):

	def __init__(self):
		
		super().__init__("led_status_handler_node")

		self.cart_mode = 1

		self.last_imu_coming_stamp = time.time()
		self.prev_got_imu = False
		self.got_imu_error = False
		self.set_imu_error = False

		self.last_auto_run_stamp = time.time()
		self.prev_auto_run_state = 0
		self.auto_run_state = 0

		qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=1)
		self.imu_error_sub = self.create_subscription(Bool, "/zmoab/imu_error", self.imu_error_callback, qos)
		self.auto_run_sub = self.create_subscription(Int8, "/zmoab/auto_run_state", self.auto_run_callback, qos)
		self.led_state_pub = self.create_publisher(Int8, "/zmoab/led_state", qos)
		self.cart_mode_sub = self.create_subscription(Int8, "/zmoab/cart_mode", self.cart_mode_callback, qos)

		self.timer = self.create_timer(0.05, self.timer_callback)


	def imu_error_callback(self, msg):
		self.prev_got_imu = self.got_imu_error
		self.last_imu_coming_stamp = time.time()
		self.got_imu_error = msg.data

		if (self.prev_got_imu != self.got_imu_error) and (self.got_imu_error):
			self.set_imu_error = True

	def auto_run_callback(self, msg):
		'''
		1: Running in mission
		2: wait for button
		3: idler
		'''
		self.prev_auto_run_state = self.auto_run_state
		self.last_auto_run_stamp = time.time()
		self.auto_run_state = msg.data

		if (self.prev_auto_run_state != self.auto_run_state) and (self.auto_run_state == 1):
			self.pub_led_state(33)
		elif (self.prev_auto_run_state != self.auto_run_state) and (self.auto_run_state == 2):
			self.pub_led_state(23)
		elif (self.prev_auto_run_state != self.auto_run_state) and (self.auto_run_state == 3):
			self.pub_led_state(3)

	def cart_mode_callback(self, msg):

		self.prev_cart_mode = self.cart_mode
		self.cart_mode = msg.data

		if (self.prev_cart_mode != self.cart_mode) and (self.cart_mode == 2):
			self.pub_led_state(3)

		elif (self.prev_cart_mode != self.cart_mode) and (self.cart_mode != 2):
			self.pub_led_state(2)

	def pub_led_state(self, state):
		led_state_msg = Int8()
		led_state_msg.data = state
		self.led_state_pub.publish(led_state_msg)

	def timer_callback(self):

		if (self.set_imu_error):
			self.set_imu_error = False
			self.pub_led_state(11)

		if (time.time() - self.last_imu_coming_stamp) > 3.0:
			self.got_imu_error = False
			self.prev_got_imu = False


def main(args=None):

	rclpy.init(args=None)
	node = LEDHandler()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()