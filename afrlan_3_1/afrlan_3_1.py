import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import isinf

class afNode(Node):
	def __init__(self):
		super().__init__('afrlan_3_1')
		self.create_subscription(
			LaserScan,
			'/scan',
			self.read_scan,
			10
		)
		self.buffer = Buffer()
		self.transform_listener = TransformListener(self.buffer, self)
	
	def get_transform(self):
		from_frame = 'odom'
		to_frame = 'base_link'

		try:
		    now = rclpy.time.Time()
		    tf = self.buffer.lookup_transform(
		        to_frame,
		        from_frame,
		        now
		    )
		except TransformException as e:
		    self.get_logger().error(f'Could not get transform {e}')
		    tf = None
		return tf
	
	
	def read_scan(self, data):
		udaljenosti = []
		for i in data.ranges:
			if not isinf(i):
				udaljenosti.append(i)
		self.get_logger().info(str(data))
		tf = self.get_transform()
		if tf is None:
			return
		
		
			
		

def main(args=None):
	rclpy.init(args=args)
	node = afNode()
	rclpy.spin(node)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
