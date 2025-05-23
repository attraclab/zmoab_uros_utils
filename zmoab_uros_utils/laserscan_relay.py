import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserscanRelay(Node):

	def __init__(self):

		super().__init__("laserscan_relay_node")

		scan_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
											history=rclpy.qos.HistoryPolicy.KEEP_LAST, \
											depth=5)
		self.laser_pub = self.create_subscription(LaserScan, "/lidar_scan", self.lidar_scan_callback, qos_profile=scan_qos)
		self.laser_repeat_pub = self.create_publisher(LaserScan, "/scan", qos_profile=scan_qos)

	def lidar_scan_callback(self, msg):
		laser_msg = LaserScan()
		laser_msg = msg
		laser_msg.header.stamp = msg.header.stamp #msg.header.stamp self.get_clock().now().to_msg()
		laser_msg.header.frame_id = "laser_frame"
		self.laser_repeat_pub.publish(laser_msg)

def main(args= None):
	rclpy.init(args=None)
	node = LaserscanRelay()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":

	main()