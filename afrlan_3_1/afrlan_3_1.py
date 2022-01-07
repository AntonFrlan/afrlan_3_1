import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import isinf, cos, sin, atan2, asin
import numpy as np


class afNode(Node):
    def __init__(self):
        super().__init__('afrlan_3_1')
        self.create_subscription(LaserScan, '/scan', self.read_scan, 10)
        self.buffer = Buffer()
        self.transform_listener = TransformListener(self.buffer, self)
        self.udaljenost = None
        self.kut = None
        self.x = None
        self.y = None

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

    def calc_angle(self, kut_min, kut_max):
        raz1 = abs(kut_min - kut_max)
        raz2 = abs(kut_min - (kut_max - 6.28))
        kut_min, kut_max = (kut_min, kut_max) if raz1 < raz2 else (kut_min, kut_max - 6.28)

        kut_tocke = (kut_min + kut_max) / 2
        kut_tocke = kut_tocke if kut_tocke >= 0 else (kut_tocke + 6.28)

        return (kut_tocke + self.yaw_z) % 6.28


    def calc_kut_max(self, kut_min, kut_mid, kut_max, x):
        if kut_mid is None or x is None:
            return (kut_max, kut_min)
        return (kut_max, kut_min) if abs(kut_max - 6.28 - kut_min) < abs(kut_mid - 6.28 - kut_min) else (kut_mid, x)

    def calc_kut_min(self, kut_min, kut_mid, kut_max):
        if kut_mid is None:
            return kut_min
        return kut_min if kut_max - kut_min > kut_max - kut_mid else kut_mid

    def read_scan(self, data):
        udaljenosti = []
        tmp = 0
        kut_min, kut_max, kut_mid_min, kut_mid_max = None, 0, None, None
        for i in data.ranges:
            if not isinf(i):
                if kut_min is None:
                    kut_min = tmp
                if kut_mid_max is None and tmp >= 3.14:
                    kut_mid_max = tmp
                if tmp <= 3.14:
                    kut_mid_min = tmp
                kut_max = tmp
                udaljenosti.append(i)
            tmp += data.angle_increment
        tf = self.get_transform()
        if tf is None:
            return
        self.euler_from_quaternion(tf.transform.rotation)
        kut_max, kut_min = self.calc_kut_max(kut_min, kut_mid_max, kut_max, kut_mid_min)

        self.udaljenost = np.mean(udaljenosti)
        self.calculate_cilj(tf.transform.translation)
        self.kut = self.calc_angle(kut_min, kut_max)
        self.get_logger().info(str((self.x, self.y)))


    def calculate_cilj(self, pos):
        if self.udaljenost is None or self.kut is None:
            return
        self.x = self.udaljenost * cos(self.kut + self.pitch_y) + pos.x
        self.y = self.udaljenost * sin(self.kut + self.pitch_y) + pos.y

    def euler_from_quaternion(self, r):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x, y, z, w = r.x, r.y, r.z, r.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        self.roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.yaw_z = atan2(t3, t4)


def main(args=None):
    rclpy.init(args=args)
    node = afNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()