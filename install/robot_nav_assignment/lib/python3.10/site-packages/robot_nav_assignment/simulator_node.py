import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math


class DiffDriveSimulator(Node):
    def __init__(self):
        super().__init__('diff_drive_simulator')
        # initial pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.omega = 0.0

        self.last_time = None

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/actual_path', 10)
        self.pose_timer = self.create_timer(0.05, self.update)  # 20 Hz
        self.path = []
        self.path_publish_count = 0

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # simple Euler integration
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.omega * dt

        # publish odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        # yaw -> quaternion
        half = self.yaw / 2.0
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        odom.twist.twist.linear.x = float(self.v)
        odom.twist.twist.angular.z = float(self.omega)

        self.odom_pub.publish(odom)

        # accumulate path and periodically publish
        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = 'odom'
        ps.pose.position.x = float(self.x)
        ps.pose.position.y = float(self.y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.z = math.sin(half)
        ps.pose.orientation.w = math.cos(half)
        self.path.append(ps)
        self.path_publish_count += 1
        if self.path_publish_count >= 5:  # publish at ~4 Hz
            path_msg = Path()
            path_msg.header.stamp = now.to_msg()
            path_msg.header.frame_id = 'odom'
            path_msg.poses = list(self.path)
            self.path_pub.publish(path_msg)
            self.path_publish_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
